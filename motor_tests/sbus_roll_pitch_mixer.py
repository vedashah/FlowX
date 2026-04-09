import pigpio
import serial
import time

# RC Serial Setup
ser = serial.Serial('/dev/serial0', 420000, timeout=0.01)

def read_channels():
    #Read RAW Controller inputs 
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
    #Convert ADC to microseconds (within 1000-2000 microseconds)
    return 1000 + (value / 2047.0) * 1000

# Motor Pinouts and Setup
M1 = 27
M2 = 26
M3 = 23
M4 = 16
MOTORS = [M1, M2, M3, M4]

PWM_MIN = 1000
PWM_MAX = 2000

ROLL_GAIN  = 0.8
PITCH_GAIN = 0.8

ROLL_DIR  = +1
PITCH_DIR = +1

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

#Main Logic Loop
print("Serial RC Mixer Running... Debug Mode ON")
set_all(PWM_MIN)
armed = False

try:
    while True:
        ch = read_channels()
        if ch is None:
            continue

        #Raw Serial Channel Readings
        roll_raw  = ch[0]
        pitch_raw = ch[1]
        thr_raw   = ch[2]   # CH3 (throttle)
        yaw_raw   = ch[3]   # CH4 (yaw)
        arm_raw   = ch[5]   # CH6 (arm switch)

        # ARMING LOGIC
        # Disarm as soon as SA switch is turned MIDDLE or LOW
        if arm_raw > 200 and armed:
            armed = False
            set_all(PWM_MIN)
            print("\nDISARMED (SA switched high)")

        # Arm logic for left stick DOWN to the LEFT and SA set UP
        if not armed:
            if thr_raw < 200 and yaw_raw < 200 and arm_raw < 200:
                armed = True
                print("\nARMED")

        # MOTOR OUTPUT
        if not armed:
            # Disarmed: Motors stay at PWM_MIN
            set_all(PWM_MIN)
            print(
                f"DISARMED | Raw CH={roll_raw},{pitch_raw},{thr_raw},{arm_raw}        ",
                end="\r"
            )
            time.sleep(0.005)
            continue

        #ONCE ARMED: RUN MIXER
        roll  = map_channel(roll_raw)
        pitch = map_channel(pitch_raw)
        thr   = map_channel(thr_raw)

        roll_delta  = (roll  - 1500) * ROLL_GAIN  * ROLL_DIR
        pitch_delta = (pitch - 1500) * PITCH_GAIN * PITCH_DIR

        m1 = clamp(thr + pitch_delta + roll_delta, PWM_MIN, PWM_MAX)
        m2 = clamp(thr + pitch_delta - roll_delta, PWM_MIN, PWM_MAX)
        m3 = clamp(thr - pitch_delta + roll_delta, PWM_MIN, PWM_MAX)
        m4 = clamp(thr - pitch_delta - roll_delta, PWM_MIN, PWM_MAX)

        set_motor(M1, m1)
        set_motor(M2, m2)
        set_motor(M3, m3)
        set_motor(M4, m4)

        #Print values for debugging
        print(
            f"ARMED | THR={int(thr)} | "
            f"M1={int(m1)} M2={int(m2)} M3={int(m3)} M4={int(m4)} | "
            f"Raw CH={roll_raw},{pitch_raw},{thr_raw},{arm_raw}",
            end="\r"
        )

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    set_all(PWM_MIN)
    pi.stop()
    ser.close()
