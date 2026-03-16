import pigpio
import serial
import time
import csv
import struct

# =======================
# USER CONFIG
# =======================

# Receiver (SBUS) on Pi UART
RC_PORT = "/dev/serial0"
RC_BAUD = 420000

# Flight controller (Betaflight) via USB (MSP)
MSP_PORT = "/dev/ttyACM0"
MSP_BAUD = 115200

# Control loop rate
LOOP_HZ = 200.0
DT_TARGET = 1.0 / LOOP_HZ

# Motor GPIO pins (ESC signal into Pi)
M1, M2, M3, M4 = 27, 26, 23, 16
MOTORS = [M1, M2, M3, M4]

PWM_MIN = 1000
PWM_MAX = 2000

# Stick -> desired rate (deg/s)  (reduced for rig)
MAX_RATE_DPS_ROLL  = 120.0
MAX_RATE_DPS_PITCH = 120.0
MAX_RATE_DPS_YAW   = 90.0

# MSP RAW_IMU gyro scale:
# Betaflight MSP_RAW_IMU gyro often: gyro_dps * 16  (1 unit = 1/16 deg/s)
GYRO_SCALE = 1.0 / 16.0  # units -> deg/s

# Safety
IMU_TIMEOUT_S = 0.10
ARM_THROTTLE_MAX_US = 1100
THROTTLE_IDLE_US = 1050
LOG_FLUSH_EVERY_N = 20

# PID gains (start smaller; P-only first)
PID = {
    "roll":  {"kp": 1.0, "ki": 0.0, "kd": 0.0, "i_lim": 200.0},
    "pitch": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "i_lim": 200.0},
    "yaw":   {"kp": 0.8, "ki": 0.0, "kd": 0.0, "i_lim": 200.0},
}

# Direction flips if axes feel inverted
ROLL_DIR  = +1
PITCH_DIR = +1
YAW_DIR   = +1

# Stick deadband (increase if still drifting)
STICK_DEADBAND_US = 70

# =======================
# SBUS / RC decode
# =======================

def read_channels_sbus(ser_rc):
    ser_rc.reset_input_buffer()
    data = ser_rc.read(26)
    if len(data) >= 24 and data[0] == 0xC8:
        ch1 = ((data[3] | data[4] << 8) & 0x07FF)
        ch2 = ((data[4] >> 3 | data[5] << 5) & 0x07FF)
        ch3 = ((data[5] >> 6 | data[6] << 2 | data[7] << 10) & 0x07FF)
        ch4 = ((data[7] >> 1 | data[8] << 7) & 0x07FF)
        ch5 = ((data[8] >> 4 | data[9] << 4) & 0x07FF)
        ch6 = ((data[9] >> 7 | data[10] << 1 | data[11] << 9) & 0x07FF)
        ch7 = ((data[11] >> 2 | data[12] << 6) & 0x07FF)
        ch8 = ((data[12] >> 5 | data[13] << 3) & 0x07FF)
        return [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8]
    return None

def sbus_to_us(v):
    return 1000 + (v / 2047.0) * 1000

def us_to_norm(us, deadband_us=70):
    # 1000..2000 -> -1..+1, with center deadband and rescale outside deadband
    x = us - 1500.0
    if abs(x) <= deadband_us:
        return 0.0
    if x > 0:
        x -= deadband_us
        span = 500.0 - deadband_us
    else:
        x += deadband_us
        span = 500.0 - deadband_us
    return max(-1.0, min(1.0, x / span))

# =======================
# MSP helpers (Betaflight)
# =======================

MSP_RAW_IMU = 102  # payload: 9x int16 = 18 bytes

def msp_request(cmd: int) -> bytes:
    length = 0
    checksum = length ^ cmd
    return b"$M<" + bytes([length, cmd, checksum])

def read_msp_response(ser_msp, timeout_s=0.05):
    t0 = time.perf_counter()
    # Find header "$M>"
    while time.perf_counter() - t0 < timeout_s:
        b = ser_msp.read(1)
        if not b:
            continue
        if b == b"$":
            hdr = ser_msp.read(2)
            if hdr == b"M>":
                break
    else:
        return None

    size_b = ser_msp.read(1)
    cmd_b  = ser_msp.read(1)
    if not size_b or not cmd_b:
        return None

    size = size_b[0]
    cmd  = cmd_b[0]

    payload = ser_msp.read(size)
    chk_b   = ser_msp.read(1)
    if len(payload) != size or len(chk_b) != 1:
        return None

    # checksum verify
    c = size ^ cmd
    for x in payload:
        c ^= x
    if c != chk_b[0]:
        return None

    return cmd, payload

def get_raw_imu(ser_msp):
    ser_msp.write(msp_request(MSP_RAW_IMU))
    resp = read_msp_response(ser_msp, timeout_s=0.05)
    if resp is None:
        return None
    cmd, payload = resp
    if cmd != MSP_RAW_IMU or len(payload) != 18:
        return None

    ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack("<hhhhhhhhh", payload)

    gx_dps = gx * GYRO_SCALE
    gy_dps = gy * GYRO_SCALE
    gz_dps = gz * GYRO_SCALE

    return (ax, ay, az, gx_dps, gy_dps, gz_dps)

# =======================
# PID
# =======================

class RatePID:
    def __init__(self, kp, ki, kd, i_lim):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_lim = abs(i_lim)
        self.i = 0.0
        self.prev_err = 0.0

    def reset(self):
        self.i = 0.0
        self.prev_err = 0.0

    def update(self, err, dt):
        self.i += err * dt
        if self.i > self.i_lim: self.i = self.i_lim
        if self.i < -self.i_lim: self.i = -self.i_lim

        derr = (err - self.prev_err) / dt if dt > 1e-6 else 0.0
        self.prev_err = err

        out = (self.kp * err) + (self.ki * self.i) + (self.kd * derr)
        return out, (self.kp * err), (self.ki * self.i), (self.kd * derr)

# =======================
# Motor helpers
# =======================

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def set_motor(pi, pin, pw):
    pi.set_servo_pulsewidth(pin, int(pw))

def set_all(pi, pw):
    for m in MOTORS:
        set_motor(pi, m, pw)

# =======================
# Main
# =======================

def main():
    ser_rc  = serial.Serial(RC_PORT, RC_BAUD, timeout=0.01)
    ser_msp = serial.Serial(MSP_PORT, MSP_BAUD, timeout=0.01)

    pi = pigpio.pi()
    if not pi.connected:
        print("Run: sudo pigpiod")
        return

    pid_roll  = RatePID(**PID["roll"])
    pid_pitch = RatePID(**PID["pitch"])
    pid_yaw   = RatePID(**PID["yaw"])

    log_name = f"rig_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    f = open(log_name, "w", newline="")
    w = csv.writer(f)
    w.writerow([
        "t", "armed",
        "thr_us", "roll_sp_dps", "pitch_sp_dps", "yaw_sp_dps",
        "gx_dps", "gy_dps", "gz_dps",
        "roll_u", "roll_p", "roll_i", "roll_d",
        "pitch_u", "pitch_p", "pitch_i", "pitch_d",
        "yaw_u", "yaw_p", "yaw_i", "yaw_d",
        "m1_us", "m2_us", "m3_us", "m4_us",
        "rc_raw_roll", "rc_raw_pitch", "rc_raw_thr", "rc_raw_yaw", "rc_raw_arm"
    ])

    print("Hybrid Rate Controller (Pi motors + F722 IMU over MSP) running...")
    print(f"Logging to: {log_name}")

    armed = False
    last_imu_time = 0.0
    set_all(pi, PWM_MIN)

    t_start = time.perf_counter()
    t_prev = time.perf_counter()
    flush_ctr = 0

    try:
        while True:
            loop_t0 = time.perf_counter()
            dt = loop_t0 - t_prev
            t_prev = loop_t0
            t_rel = loop_t0 - t_start

            ch = read_channels_sbus(ser_rc)
            if ch is None:
                armed = False
                set_all(pi, PWM_MIN)
                pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset()
                time.sleep(0.005)
                continue

            roll_raw  = ch[0]
            pitch_raw = ch[1]
            thr_raw   = ch[2]
            yaw_raw   = ch[3]
            arm_raw   = ch[5]

            roll_us  = sbus_to_us(roll_raw)
            pitch_us = sbus_to_us(pitch_raw)
            thr_us   = sbus_to_us(thr_raw)
            yaw_us   = sbus_to_us(yaw_raw)

            # --- ARM/KILL (kept as you had it) ---
            if arm_raw > 200 and armed:
                armed = False
                set_all(pi, PWM_MIN)
                pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset()
                print("\nDISARMED (switch)")

            if not armed:
                if thr_us <= ARM_THROTTLE_MAX_US and arm_raw < 200:
                    armed = True
                    last_imu_time = loop_t0
                    pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset()
                    print("\nARMED")
                else:
                    set_all(pi, PWM_MIN)
                    print(f"DISARMED | thr={int(thr_us)} arm_raw={arm_raw}        ", end="\r")
                    time.sleep(0.002)
                    continue

            # --- IMU (gyro) ---
            imu = get_raw_imu(ser_msp)
            if imu is None:
                if (loop_t0 - last_imu_time) > IMU_TIMEOUT_S:
                    armed = False
                    set_all(pi, PWM_MIN)
                    pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset()
                    print("\nDISARMED (IMU timeout)")
                time.sleep(0.001)
                continue

            last_imu_time = loop_t0
            ax, ay, az, gx_dps, gy_dps, gz_dps = imu

            # --- Setpoints from sticks (rate mode) ---
            roll_sp  = us_to_norm(roll_us,  STICK_DEADBAND_US) * MAX_RATE_DPS_ROLL  * ROLL_DIR
            pitch_sp = us_to_norm(pitch_us, STICK_DEADBAND_US) * MAX_RATE_DPS_PITCH * PITCH_DIR
            yaw_sp   = us_to_norm(yaw_us,   STICK_DEADBAND_US) * MAX_RATE_DPS_YAW   * YAW_DIR

            # Common axis mapping
            roll_rate  = gx_dps
            pitch_rate = gy_dps
            yaw_rate   = gz_dps

            roll_err  = roll_sp  - roll_rate
            pitch_err = pitch_sp - pitch_rate
            yaw_err   = yaw_sp   - yaw_rate

            roll_u,  roll_p,  roll_i,  roll_d  = pid_roll.update(roll_err, dt)
            pitch_u, pitch_p, pitch_i, pitch_d = pid_pitch.update(pitch_err, dt)
            yaw_u,   yaw_p,   yaw_i,   yaw_d   = pid_yaw.update(yaw_err, dt)

            thr_cmd = max(thr_us, THROTTLE_IDLE_US)

            # Quad X mixer
            m1 = clamp(thr_cmd + pitch_u + roll_u + yaw_u, PWM_MIN, PWM_MAX)
            m2 = clamp(thr_cmd + pitch_u - roll_u - yaw_u, PWM_MIN, PWM_MAX)
            m3 = clamp(thr_cmd - pitch_u + roll_u - yaw_u, PWM_MIN, PWM_MAX)
            m4 = clamp(thr_cmd - pitch_u - roll_u + yaw_u, PWM_MIN, PWM_MAX)

            set_motor(pi, M1, m1)
            set_motor(pi, M2, m2)
            set_motor(pi, M3, m3)
            set_motor(pi, M4, m4)

            w.writerow([
                f"{t_rel:.6f}", int(armed),
                int(thr_cmd), f"{roll_sp:.3f}", f"{pitch_sp:.3f}", f"{yaw_sp:.3f}",
                f"{roll_rate:.3f}", f"{pitch_rate:.3f}", f"{yaw_rate:.3f}",
                f"{roll_u:.3f}", f"{roll_p:.3f}", f"{roll_i:.3f}", f"{roll_d:.3f}",
                f"{pitch_u:.3f}", f"{pitch_p:.3f}", f"{pitch_i:.3f}", f"{pitch_d:.3f}",
                f"{yaw_u:.3f}", f"{yaw_p:.3f}", f"{yaw_i:.3f}", f"{yaw_d:.3f}",
                int(m1), int(m2), int(m3), int(m4),
                roll_raw, pitch_raw, thr_raw, yaw_raw, arm_raw
            ])
            flush_ctr += 1
            if flush_ctr >= LOG_FLUSH_EVERY_N:
                f.flush()
                flush_ctr = 0

            print(
                f"ARMED | thr={int(thr_cmd)} "
                f"rate(r,p,y)=({roll_rate:6.1f},{pitch_rate:6.1f},{yaw_rate:6.1f}) "
                f"sp=({roll_sp:6.1f},{pitch_sp:6.1f},{yaw_sp:6.1f}) "
                f"m=({int(m1)},{int(m2)},{int(m3)},{int(m4)})    ",
                end="\r"
            )

            elapsed = time.perf_counter() - loop_t0
            sleep_t = DT_TARGET - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\nExiting...")

    finally:
        set_all(pi, PWM_MIN)
        pi.stop()
        ser_rc.close()
        ser_msp.close()
        f.flush()
        f.close()
        print(f"Log saved: {log_name}")

if __name__ == "__main__":
    main()
