import pigpio
import serial
import struct
import time

# ------------------ RC Serial Setup ------------------
ser = None
while ser is None:
    try:
        ser = serial.Serial('/dev/serial0', 420000, timeout=0.01)
    except Exception as e:
        print(f"Waiting for RC serial... {e}")
        time.sleep(1)

bad_read_count = 0
MAX_BAD_READS = 50

def read_channels():
    global ser, bad_read_count
    try:
        ser.reset_input_buffer()
        data = ser.read(26)
        if len(data) >= 24 and data[0] == 0xC8:
            bad_read_count = 0
            ch1 = ((data[3] | data[4] << 8) & 0x07FF)
            ch2 = ((data[4] >> 3 | data[5] << 5) & 0x07FF)
            ch3 = ((data[5] >> 6 | data[6] << 2 | data[7] << 10) & 0x07FF)
            ch4 = ((data[7] >> 1 | data[8] << 7) & 0x07FF)
            ch5 = ((data[8] >> 4 | data[9] << 4) & 0x07FF)
            ch6 = ((data[9] >> 7 | data[10] << 1 | data[11] << 9) & 0x07FF)
            ch7 = ((data[11] >> 2 | data[12] << 6) & 0x07FF)
            ch8 = ((data[12] >> 5 | data[13] << 3) & 0x07FF)
            return [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8]
        bad_read_count += 1
        if bad_read_count >= MAX_BAD_READS:
            print("Too many bad reads, reopening serial...")
            bad_read_count = 0
            ser.close()
            time.sleep(1)
            ser = serial.Serial('/dev/serial0', 420000, timeout=0.01)
        return None
    except Exception:
        try:
            ser.close()
            time.sleep(1)
            ser = serial.Serial('/dev/serial0', 420000, timeout=0.01)
        except Exception:
            pass
        return None

def map_channel(value):
    return 1000 + (value / 2047.0) * 1000

# ------------------ IMU Setup ------------------
IMU_PORT = "/dev/ttyACM0"
IMU_BAUD = 115200
MSP_ATTITUDE = 108

def msp_request(cmd):
    length = 0
    checksum = length ^ cmd
    return b"$M<" + bytes([length, cmd, checksum])

def read_msp_response(imu):
    while True:
        b = imu.read(1)
        if not b:
            return None
        if b == b"$":
            hdr = imu.read(2)
            if hdr == b"M>":
                break
    size = imu.read(1)
    cmd  = imu.read(1)
    if not size or not cmd:
        return None
    size = size[0]
    cmd  = cmd[0]
    payload = imu.read(size)
    chk = imu.read(1)
    if len(payload) != size or len(chk) != 1:
        return None
    c = size ^ cmd
    for x in payload:
        c ^= x
    if c != chk[0]:
        return None
    return cmd, payload

imu = serial.Serial(IMU_PORT, IMU_BAUD, timeout=0.2)

def get_imu():
    imu.write(msp_request(MSP_ATTITUDE))
    resp = read_msp_response(imu)
    if resp is None:
        return None
    cmd, payload = resp
    if cmd == MSP_ATTITUDE and len(payload) == 6:
        roll_tenth, pitch_tenth, heading = struct.unpack("<hhh", payload)
        return roll_tenth / 10.0, pitch_tenth / 10.0, float(heading)
    return None

# ------------------ Motor GPIO Mapping ------------------
M1 = 23  # Rear Right
M2 = 16  # Front Right
M3 = 26  # Rear Left
M4 = 27  # Front Left
MOTORS = [M1, M2, M3, M4]

LED_R = 24
LED_G = 25
LED_B = 5

PWM_MIN = 1000
PWM_MAX = 2000

ROLL_GAIN  = 0.1
PITCH_GAIN = 0.4
YAW_GAIN   = 0.7
ROLL_DIR   = +1
PITCH_DIR  = +1
YAW_DIR    = -1

pi = pigpio.pi()
if not pi.connected:
    print("Run: sudo pigpiod")
    exit()

for pin in [LED_R, LED_G, LED_B]:
    pi.set_mode(pin, pigpio.OUTPUT)

def set_led(r, g, b):
    pi.write(LED_R, int(r))
    pi.write(LED_G, int(g))
    pi.write(LED_B, int(b))

def rainbow_led(hue):
    h = hue * 6.0
    i = int(h)
    f = h - i
    if i == 0:   r, g, b = 1, f, 0
    elif i == 1: r, g, b = 1-f, 1, 0
    elif i == 2: r, g, b = 0, 1, f
    elif i == 3: r, g, b = 0, 1-f, 1
    elif i == 4: r, g, b = f, 0, 1
    else:        r, g, b = 1, 0, 1-f
    pi.write(LED_R, int(r))
    pi.write(LED_G, int(g))
    pi.write(LED_B, int(b))

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def set_motor(pin, pw):
    pi.set_servo_pulsewidth(pin, int(pw))

def set_all(pw):
    for m in MOTORS:
        set_motor(m, pw)

# ------------------ PID ------------------
class PID:
    def __init__(self, kp, ki, kd, limit=200.0, d_filter=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.d_filter = d_filter
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_d = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_d = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = clamp(self.integral, -self.limit, self.limit)
        raw_d = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.filtered_d = self.d_filter * self.filtered_d + (1.0 - self.d_filter) * raw_d
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * self.filtered_d
        return clamp(output, -self.limit, self.limit)

pid_pitch = PID(kp=0.4, ki=0.0, kd=0.15, d_filter=0.8)
pid_roll  = PID(kp=0.4, ki=0.0, kd=0.15, d_filter=0.8)
pid_yaw   = PID(kp=0.2, ki=0.0, kd=0.05, d_filter=0.8)

# ------------------ State ------------------
HOLD_THRESHOLD = 1000

hold_mode = False
hold_mode_prev = False
sp_pitch = 0.0
sp_roll  = 0.0
sp_yaw   = 0.0

handoff_active = False
handoff_start_time = 0.0
HANDOFF_DURATION = 0.5
handoff_start_thr = 1000.0

rainbow_hue = 0.0
last_time = time.time()
armed = False
flash_state = False
last_flash_time = time.time()
FLASH_INTERVAL = 0.5

print("RC + Hold Mode Controller")
print("CH5 low  = normal RC flight")
print("CH5 high = hold current angle (PID)")
print("Arm: throttle low + yaw low + arm switch low")
set_all(PWM_MIN)
set_led(0, 0, 0)

try:
    while True:
        ch = read_channels()

        if ch is None:
            now = time.time()
            if now - last_flash_time >= FLASH_INTERVAL:
                flash_state = not flash_state
                last_flash_time = now
            set_led(0, 0, 1 if flash_state else 0)
            continue

        roll_raw  = ch[0]
        pitch_raw = ch[1]
        thr_raw   = ch[2]
        yaw_raw   = ch[3]
        hold_raw  = ch[4]  # CH5
        arm_raw   = ch[5]  # CH6

        # ------------------ Arming ------------------
        if arm_raw > 200 and armed:
            armed = False
            hold_mode = False
            hold_mode_prev = False
            set_all(PWM_MIN)
            set_led(1, 0, 0)
            pid_pitch.reset()
            pid_roll.reset()
            pid_yaw.reset()
            print("\nDISARMED")

        if not armed:
            if thr_raw < 200 and yaw_raw < 200 and arm_raw < 200:
                armed = True
                set_led(0, 1, 0)
                last_time = time.time()
                print("\nARMED")

        if not armed:
            set_all(PWM_MIN)
            set_led(1, 0, 0)
            print(f"DISARMED | thr={thr_raw} arm={arm_raw}   ", end="\r")
            time.sleep(0.005)
            continue

        # ------------------ Hold mode detection ------------------
        hold_mode = hold_raw > HOLD_THRESHOLD

        if hold_mode and not hold_mode_prev:
            imu_data = get_imu()
            if imu_data is not None:
                sp_roll, sp_pitch, sp_yaw = imu_data
                print(f"\nHOLD ENGAGED: P={sp_pitch:.1f}° R={sp_roll:.1f}° Y={sp_yaw:.0f}°")
            pid_pitch.reset()
            pid_roll.reset()
            pid_yaw.reset()

        if not hold_mode and hold_mode_prev:
            handoff_active = True
            handoff_start_time = time.time()
            handoff_start_thr = map_channel(thr_raw)
            print("\nRC MODE")
            set_led(0, 1, 0)

        hold_mode_prev = hold_mode

        # ------------------ IMU ------------------
        imu_data = get_imu()
        roll_actual = pitch_actual = yaw_actual = 0.0
        if imu_data is not None:
            roll_actual, pitch_actual, yaw_actual = imu_data

        now = time.time()
        dt = now - last_time
        last_time = now

        # ------------------ RC mixer values ------------------
        roll  = map_channel(roll_raw)
        pitch = map_channel(pitch_raw)
        thr   = map_channel(thr_raw)
        yaw   = map_channel(yaw_raw)

        roll_cmd  = (roll  - 1500) * ROLL_GAIN  * ROLL_DIR
        pitch_cmd = (pitch - 1500) * PITCH_GAIN * PITCH_DIR
        yaw_cmd   = (yaw   - 1500) * YAW_GAIN   * YAW_DIR

        rL, rR = +roll_cmd, -roll_cmd
        pF, pB = -pitch_cmd, +pitch_cmd
        Y1, Y2, Y3, Y4 = -1, +1, +1, -1

        if hold_mode:
            # ------------------ PID hold ------------------
            pitch_err = sp_pitch - pitch_actual
            roll_err  = sp_roll  - roll_actual
            yaw_err   = sp_yaw   - yaw_actual
            if yaw_err > 180:    yaw_err -= 360
            elif yaw_err < -180: yaw_err += 360

            pitch_out = pid_pitch.compute(pitch_err, dt)
            roll_out  = pid_roll.compute(roll_err,  dt)
            yaw_out   = pid_yaw.compute(yaw_err,   dt)

            m1 = clamp(thr + pitch_out - roll_out + Y1 * yaw_out, PWM_MIN, PWM_MAX)
            m2 = clamp(thr - pitch_out - roll_out + Y2 * yaw_out, PWM_MIN, PWM_MAX)
            m3 = clamp(thr + pitch_out + roll_out + Y3 * yaw_out, PWM_MIN, PWM_MAX)
            m4 = clamp(thr - pitch_out + roll_out + Y4 * yaw_out, PWM_MIN, PWM_MAX)

            rainbow_hue = (rainbow_hue + 0.005) % 1.0
            rainbow_led(rainbow_hue)

            print(
                f"HOLD | IMU P={pitch_actual:5.1f}° R={roll_actual:5.1f}° | "
                f"SP P={sp_pitch:4.1f}° R={sp_roll:4.1f}° | "
                f"ERR P={pitch_err:4.1f}° R={roll_err:4.1f}° | "
                f"M1={int(m1)} M2={int(m2)} M3={int(m3)} M4={int(m4)}   ",
                end="\r"
            )

        else:
            # ------------------ Normal RC ------------------
            m1 = clamp(thr + pB + rR + Y1 * yaw_cmd, PWM_MIN, PWM_MAX)
            m2 = clamp(thr + pF + rR + Y2 * yaw_cmd, PWM_MIN, PWM_MAX)
            m3 = clamp(thr + pB + rL + Y3 * yaw_cmd, PWM_MIN, PWM_MAX)
            m4 = clamp(thr + pF + rL + Y4 * yaw_cmd, PWM_MIN, PWM_MAX)

            if handoff_active:
                elapsed = now - handoff_start_time
                alpha = min(elapsed / HANDOFF_DURATION, 1.0)
                blend_thr = handoff_start_thr + alpha * (thr - handoff_start_thr)
                m1 = clamp(blend_thr + pB + rR + Y1 * yaw_cmd, PWM_MIN, PWM_MAX)
                m2 = clamp(blend_thr + pF + rR + Y2 * yaw_cmd, PWM_MIN, PWM_MAX)
                m3 = clamp(blend_thr + pB + rL + Y3 * yaw_cmd, PWM_MIN, PWM_MAX)
                m4 = clamp(blend_thr + pF + rL + Y4 * yaw_cmd, PWM_MIN, PWM_MAX)
                if alpha >= 1.0:
                    handoff_active = False

            print(
                f"RC   | THR={int(thr)} YAW={int(yaw)} | "
                f"M1={int(m1)} M2={int(m2)} M3={int(m3)} M4={int(m4)} | "
                f"Raw={roll_raw},{pitch_raw},{thr_raw},{yaw_raw}   ",
                end="\r"
            )

        set_motor(M1, m1)
        set_motor(M2, m2)
        set_motor(M3, m3)
        set_motor(M4, m4)

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    set_all(PWM_MIN)
    set_led(0, 0, 0)
    pi.stop()
    ser.close()
    imu.close()
