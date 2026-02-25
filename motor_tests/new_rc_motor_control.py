import pigpio
import serial
import time

# RC Serial Setup
ser = serial.Serial('/dev/serial0', 420000, timeout=0.01)

def read_channels():
    # Read RAW Controller inputs
    ser.reset_input_buffer()
    data = ser.read(26)
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

def map_channel(value):
    # Convert ADC to microseconds (within 1000-2000 microseconds)
    return 1000 + (value / 2047.0) * 1000

# ------------------ Motor GPIO Mapping (YOUR REQUEST) ------------------
# BetaFlight diagram positions:
# M2 = Front Right, M4 = Front Left, M1 = Rear Right, M3 = Rear Left
#
# Your GPIOs:
M1 = 23  # Rear Right
M2 = 16  # Front Right
M3 = 26  # Rear Left
M4 = 27  # Front Left
MOTORS = [M1, M2, M3, M4]

PWM_MIN = 1000
PWM_MAX = 2000

# Gains
ROLL_GAIN  = 0.8
PITCH_GAIN = 0.8
YAW_GAIN   = 0.6

# Directions (flip any of these if a stick does the opposite of what you expect)
ROLL_DIR  = -1
PITCH_DIR = +1
YAW_DIR   = -1

pi = pigpio.pi()
if not pi.connected:
    print("Run: sudo pigpiod")
    exit()

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def set_motor(pin, pw):
    pi.set_servo_pulsewidth(pin, int(pw))

def set_all(pw):
    for m in MOTORS:
        set_motor(m, pw)

# ------------------ Main Logic Loop ------------------
print("Serial RC Mixer Running... Debug Mode ON")
set_all(PWM_MIN)
armed = False

try:
    while True:
        ch = read_channels()
        if ch is None:
            continue

        # Raw Serial Channel Readings (0..2047-ish)
        roll_raw  = ch[0]
        pitch_raw = ch[1]
        thr_raw   = ch[2]   # CH3 (throttle)
        yaw_raw   = ch[3]   # CH4 (yaw)
        arm_raw   = ch[5]   # CH6 (arm switch)

        # ------------------ ARMING LOGIC ------------------
        if arm_raw > 200 and armed:
            armed = False
            set_all(PWM_MIN)
            print("\nDISARMED (SA switched high)")

        if not armed:
            if thr_raw < 200 and yaw_raw < 200 and arm_raw < 200:
                armed = True
                print("\nARMED")

        # ------------------ DISARMED OUTPUT ------------------
        if not armed:
            set_all(PWM_MIN)
            print(
                f"DISARMED | Raw CH={roll_raw},{pitch_raw},{thr_raw},{yaw_raw},{arm_raw}        ",
                end="\r"
            )
            time.sleep(0.005)
            continue

        # ------------------ ONCE ARMED: RUN MIXER ------------------
        roll  = map_channel(roll_raw)
        pitch = map_channel(pitch_raw)
        thr   = map_channel(thr_raw)
        yaw   = map_channel(yaw_raw)

        # Commands about center (µs)
        roll_cmd  = (roll  - 1500) * ROLL_GAIN  * ROLL_DIR
        pitch_cmd = (pitch - 1500) * PITCH_GAIN * PITCH_DIR
        yaw_cmd   = (yaw   - 1500) * YAW_GAIN   * YAW_DIR

        # Roll: left +, right -
        rL, rR = +roll_cmd, -roll_cmd

        # Pitch: rear +, front -
        pF, pB = -pitch_cmd, +pitch_cmd

        # Yaw: you said M1 & M4 are CW, M2 & M3 are CCW
        # Convention here: CCW motors get +yaw_cmd, CW motors get -yaw_cmd
        Y1, Y2, Y3, Y4 = -1, +1, +1, -1

        # Motor outputs
        # M1 = Rear Right, M2 = Front Right, M3 = Rear Left, M4 = Front Left
        m1 = clamp(thr + pB + rR + Y1 * yaw_cmd, PWM_MIN, PWM_MAX)
        m2 = clamp(thr + pF + rR + Y2 * yaw_cmd, PWM_MIN, PWM_MAX)
        m3 = clamp(thr + pB + rL + Y3 * yaw_cmd, PWM_MIN, PWM_MAX)
        m4 = clamp(thr + pF + rL + Y4 * yaw_cmd, PWM_MIN, PWM_MAX)

        set_motor(M1, m1)
        set_motor(M2, m2)
        set_motor(M3, m3)
        set_motor(M4, m4)

        # Print values for debugging
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
    pi.stop()
    ser.close()