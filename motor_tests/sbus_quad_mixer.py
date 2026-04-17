import pigpio
import serial
import time

# RC Serial Setup
ser = None
while ser is None:
    try:
        ser = serial.Serial('/dev/serial0', 420000, timeout=0.01)
    except Exception as e:
        print(f"Waiting for serial port... {e}")
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

# ------------------ Motor GPIO Mapping ------------------
M1 = 23  # Rear Right
M2 = 16  # Front Right
M3 = 26  # Rear Left
M4 = 27  # Front Left
MOTORS = [M1, M2, M3, M4]

# ------------------ RGB LED GPIO Mapping ------------------
LED_R = 24
LED_G = 25
LED_B = 5

PWM_MIN = 1000
PWM_MAX = 2000

ROLL_GAIN  = 0.1
PITCH_GAIN = 0.4
YAW_GAIN   = 0.7

ROLL_DIR  = +1
PITCH_DIR = +1
YAW_DIR   = -1

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

print("Serial RC Mixer Running...")
set_all(PWM_MIN)
set_led(0, 0, 0)
armed = False

flash_state = False
last_flash_time = time.time()
FLASH_INTERVAL = 0.5

try:
    while True:
        ch = read_channels()

        # ------------------ No receiver = flashing blue ------------------
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
        arm_raw   = ch[5]

        # ------------------ ARMING LOGIC ------------------
        if arm_raw > 200 and armed:
            armed = False
            set_all(PWM_MIN)
            print("\nDISARMED")

        if not armed:
            if thr_raw < 200 and yaw_raw < 200 and arm_raw < 200:
                armed = True
                print("\nARMED")

        # ------------------ DISARMED = red ------------------
        if not armed:
            set_led(1, 0, 0)
            set_all(PWM_MIN)
            print(
                f"DISARMED | Raw CH={roll_raw},{pitch_raw},{thr_raw},{yaw_raw},{arm_raw}        ",
                end="\r"
            )
            time.sleep(0.005)
            continue

        # ------------------ ARMED = green ------------------
        set_led(0, 1, 0)

        # ------------------ MIXER ------------------
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

        m1 = clamp(thr + pB + rR + Y1 * yaw_cmd, PWM_MIN, PWM_MAX)
        m2 = clamp(thr + pF + rR + Y2 * yaw_cmd, PWM_MIN, PWM_MAX)
        m3 = clamp(thr + pB + rL + Y3 * yaw_cmd, PWM_MIN, PWM_MAX)
        m4 = clamp(thr + pF + rL + Y4 * yaw_cmd, PWM_MIN, PWM_MAX)

        set_motor(M1, m1)
        set_motor(M2, m2)
        set_motor(M3, m3)
        set_motor(M4, m4)

        print(
            f"ARMED | THR={int(thr)} YAW={int(yaw)} | "
            f"M1={int(m1)} M2={int(m2)} M3={int(m3)} M4={int(m4)} | "
            f"Raw CH={roll_raw},{pitch_raw},{thr_raw},{yaw_raw},{arm_raw}",
            end="\r"
        )

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    set_all(PWM_MIN)
    set_led(0, 0, 0)
    pi.stop()