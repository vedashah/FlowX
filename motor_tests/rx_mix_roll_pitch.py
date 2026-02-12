import pigpio
import time


PPM_PIN = 17

M1_FRONT_LEFT  = 27
M2_FRONT_RIGHT = 26
M3_BACK_LEFT   = 23
M4_BACK_RIGHT  = 16

MOTORS = [M1_FRONT_LEFT, M2_FRONT_RIGHT, M3_BACK_LEFT, M4_BACK_RIGHT]


CH_ROLL     = 0   # CH1
CH_PITCH    = 1   # CH2
CH_THROTTLE = 2   # CH3
CH_ARM      = 4   # CH5

CHANNELS = 8
SYNC_GAP_US = 3000


PWM_MIN = 1000
PWM_MAX = 2000

ARM_THRESHOLD = 1600
THROTTLE_ARM_MAX = 1050
FAILSAFE_TIMEOUT_S = 0.5

DEADBAND_US = 30

ROLL_GAIN  = 0.8     # left/right effect
PITCH_GAIN = 0.8     # front/back effect


ROLL_DIR  = +1       # if wrong, change to -1
PITCH_DIR = +1       # if wrong, change to -1


pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpiod. Run: sudo pigpiod")
    raise SystemExit(1)

channels = [1500] * CHANNELS
last_tick = 0
chan_index = 0
last_frame_time = time.time()

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def apply_deadband(x, center=1500, dead=DEADBAND_US):
    return center if abs(x - center) <= dead else x

def set_motor(pin, pw):
    pi.set_servo_pulsewidth(pin, int(pw))

def set_all(pw):
    for p in MOTORS:
        set_motor(p, pw)

def ppm_callback(gpio, level, tick):
    global last_tick, chan_index, last_frame_time
    if level != 1:  # rising edge only
        return
    dt = pigpio.tickDiff(last_tick, tick)
    last_tick = tick

    # sync gap => new frame
    if dt > SYNC_GAP_US:
        chan_index = 0
        last_frame_time = time.time()
        return

    if chan_index < CHANNELS:
        channels[chan_index] = dt
        chan_index += 1

cb = pi.callback(PPM_PIN, pigpio.RISING_EDGE, ppm_callback)

# Start safe
set_all(PWM_MIN)
print("Mixer running (Throttle + Roll + Pitch). Motors OFF = 1000us.")
print("CH3 throttle, CH1 roll (L/R), CH2 pitch (U/D), CH5 arm. CTRL+C to quit.\n")

armed = False

try:
    while True:
        # FAILSAFE
        if (time.time() - last_frame_time) > FAILSAFE_TIMEOUT_S:
            if armed:
                print("\nFAILSAFE: Lost PPM -> DISARM + motors OFF")
            armed = False
            set_all(PWM_MIN)
            time.sleep(0.05)
            continue

        thr   = clamp(channels[CH_THROTTLE], PWM_MIN, PWM_MAX)
        roll  = clamp(channels[CH_ROLL],     PWM_MIN, PWM_MAX)
        pitch = clamp(channels[CH_PITCH],    PWM_MIN, PWM_MAX)
        arm   = channels[CH_ARM]

        roll  = apply_deadband(roll)
        pitch = apply_deadband(pitch)

        want_arm = arm > ARM_THRESHOLD

        # ARM LOGIC
        if want_arm and not armed:
            if thr <= THROTTLE_ARM_MAX:
                armed = True
                print("\nARMED")
            else:
                print("\nArm blocked: throttle not low")
                armed = False
                set_all(PWM_MIN)
                time.sleep(0.1)
                continue

        if (not want_arm) and armed:
            armed = False
            set_all(PWM_MIN)
            print("\nDISARMED")

        if not armed:
            set_all(PWM_MIN)
            print(f"ARM={armed} thr={thr} roll={roll} pitch={pitch} arm={arm}     ", end="\r")
            time.sleep(0.05)
            continue

      
        roll_delta  = (roll  - 1500) * ROLL_GAIN  * ROLL_DIR
        pitch_delta = (pitch - 1500) * PITCH_GAIN * PITCH_DIR


        m1 = clamp(thr + pitch_delta + roll_delta, PWM_MIN, PWM_MAX)
        m2 = clamp(thr + pitch_delta - roll_delta, PWM_MIN, PWM_MAX)
        m3 = clamp(thr - pitch_delta + roll_delta, PWM_MIN, PWM_MAX)
        m4 = clamp(thr - pitch_delta - roll_delta, PWM_MIN, PWM_MAX)

        set_motor(M1_FRONT_LEFT,  m1)
        set_motor(M2_FRONT_RIGHT, m2)
        set_motor(M3_BACK_LEFT,   m3)
        set_motor(M4_BACK_RIGHT,  m4)

        print(f"ARM={armed} thr={thr} roll={roll} pitch={pitch} | "
              f"M1={int(m1)} M2={int(m2)} M3={int(m3)} M4={int(m4)}     ", end="\r")

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    set_all(PWM_MIN)
    cb.cancel()
    pi.stop()
    print("Motors set to 1000us and pigpio stopped.")
