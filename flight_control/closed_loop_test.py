import pigpio
import serial
import struct
import time
import threading

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
M1 = 23
M2 = 16
M3 = 26
M4 = 27
MOTORS = [M1, M2, M3, M4]

LED_R = 24
LED_G = 25
LED_B = 5

PWM_MIN = 1000
PWM_MAX = 2000

pi = pigpio.pi()
if not pi.connected:
    print("Run: sudo pigpiod")
    exit()

for pin in [LED_R, LED_G, LED_B]:
    pi.set_mode(pin, pigpio.OUTPUT)

def set_led(r, g, b):
    pi.write(LED_R, r)
    pi.write(LED_G, g)
    pi.write(LED_B, b)

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def set_motor(pin, pw):
    pi.set_servo_pulsewidth(pin, int(pw))

def set_all(pw):
    for m in MOTORS:
        set_motor(m, pw)

# ------------------ PID ------------------
class PID:
    def __init__(self, kp, ki, kd, limit=200.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = clamp(self.integral, -self.limit, self.limit)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(output, -self.limit, self.limit)

pid_pitch = PID(kp=0.3, ki=0.0, kd=0.0)
pid_roll  = PID(kp=0.3, ki=0.0, kd=0.0)
pid_yaw   = PID(kp=0.2, ki=0.0, kd=0.0)

# ------------------ Shared State ------------------
setpoint = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
throttle = 1300
armed = False
setpoint_lock = threading.Lock()
running = True

def input_thread():
    global throttle, armed
    print("Commands: arm | disarm | thr=1400 | pitch=5 roll=0 | kp=2.0 kd=0.1 | pitch.kp=2.0 | reset")
    while running:
        try:
            line = input("> ").strip().lower()
            if line == "arm":
                armed = True
                set_led(0, 1, 0)
                print("ARMED")
            elif line == "disarm":
                armed = False
                set_all(PWM_MIN)
                set_led(1, 0, 0)
                pid_pitch.reset()
                pid_roll.reset()
                pid_yaw.reset()
                print("DISARMED")
            elif line == "reset":
                with setpoint_lock:
                    setpoint["pitch"] = 0.0
                    setpoint["roll"]  = 0.0
                    setpoint["yaw"]   = 0.0
                pid_pitch.reset()
                pid_roll.reset()
                pid_yaw.reset()
                print("Setpoint reset to 0,0,0")
            elif line.startswith("thr="):
                throttle = int(clamp(float(line.split("=")[1]), PWM_MIN, PWM_MAX))
                print(f"Throttle set to {throttle}")
            elif line.startswith("kp=") or line.startswith("ki=") or line.startswith("kd="):
                for part in line.split():
                    k, v = part.split("=")
                    for pid in [pid_pitch, pid_roll, pid_yaw]:
                        setattr(pid, k.strip(), float(v))
                print(f"Gains updated")
            elif "." in line and any(line.startswith(a) for a in ["pitch.", "roll.", "yaw."]):
                axis, rest = line.split(".")
                k, v = rest.split("=")
                pid_map = {"pitch": pid_pitch, "roll": pid_roll, "yaw": pid_yaw}
                setattr(pid_map[axis.strip()], k.strip(), float(v))
                print(f"Updated {axis}.{k} = {v}")
            else:
                with setpoint_lock:
                    for part in line.split():
                        axis, val = part.split("=")
                        axis = axis.strip()
                        if axis in setpoint:
                            setpoint[axis] = float(val)
                with setpoint_lock:
                    print(f"Setpoint: pitch={setpoint['pitch']} roll={setpoint['roll']} yaw={setpoint['yaw']}")
        except (ValueError, Exception):
            pass

t = threading.Thread(target=input_thread, daemon=True)
t.start()

# ------------------ Main Loop ------------------
print("FlowX Closed-Loop Stabilization — no RC")
set_all(PWM_MIN)
set_led(1, 0, 0)

last_time = time.time()

try:
    while True:
        if not armed:
            time.sleep(0.05)
            continue

        imu_data = get_imu()
        if imu_data is None:
            continue

        roll_actual, pitch_actual, yaw_actual = imu_data

        now = time.time()
        dt = now - last_time
        last_time = now

        with setpoint_lock:
            sp_pitch = setpoint["pitch"]
            sp_roll  = setpoint["roll"]
            sp_yaw   = setpoint["yaw"]

        pitch_err = sp_pitch - pitch_actual
        roll_err  = sp_roll  - roll_actual
        yaw_err   = sp_yaw   - yaw_actual

        if yaw_err > 180:    yaw_err -= 360
        elif yaw_err < -180: yaw_err += 360

        pitch_out = pid_pitch.compute(pitch_err, dt)
        roll_out  = pid_roll.compute(roll_err,  dt)
        yaw_out   = pid_yaw.compute(yaw_err,   dt)

        Y1, Y2, Y3, Y4 = -1, +1, +1, -1
        m1 = clamp(throttle + pitch_out - roll_out + Y1 * yaw_out, PWM_MIN, PWM_MAX)
        m2 = clamp(throttle - pitch_out - roll_out + Y2 * yaw_out, PWM_MIN, PWM_MAX)
        m3 = clamp(throttle + pitch_out + roll_out + Y3 * yaw_out, PWM_MIN, PWM_MAX)
        m4 = clamp(throttle - pitch_out + roll_out + Y4 * yaw_out, PWM_MIN, PWM_MAX)

        set_motor(M1, m1)
        set_motor(M2, m2)
        set_motor(M3, m3)
        set_motor(M4, m4)

        print(
            f"P={pitch_actual:5.1f}° R={roll_actual:5.1f}° Y={yaw_actual:5.0f}° | "
            f"ERR P={pitch_err:5.1f}° R={roll_err:5.1f}° | "
            f"M1={int(m1)} M2={int(m2)} M3={int(m3)} M4={int(m4)}"
        )

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    running = False
    set_all(PWM_MIN)
    set_led(0, 0, 0)
    pi.stop()
    imu.close()