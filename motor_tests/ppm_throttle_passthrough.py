import pigpio
import time

# ----------------------------
# USER CONFIG
# ----------------------------
PPM_PIN = 17
ESC_PINS = [27, 26, 23, 16]   # Motor1..Motor4
CHANNELS = 8

# Receiver channel mapping (common RC default):
CH_THROTTLE = 2   # CH3 in human terms (0-based index)
CH_ARM     = 4    # CH5 arm switch (0-based index)

# PPM decoding
SYNC_GAP_US = 3000

# Safety / behavior
THROTTLE_MIN = 1000
THROTTLE_MAX = 2000
THROTTLE_ARM_MAX = 1050      # must be <= this to arm
ARM_THRESHOLD = 1600         # switch high = armed
FAILSAFE_TIMEOUT_S = 0.5     # if no valid PPM frames, motors off

# Optional throttle scaling (keeps low end safer)
THROTTLE_OUTPUT_MIN = 1000
THROTTLE_OUTPUT_MAX = 2000

# ----------------------------
# SETUP
# ----------------------------
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

def ppm_callback(gpio, level, tick):
    global last_tick, chan_index, last_frame_time

    if level != 1:  # rising edge only
        return

    dt = pigpio.tickDiff(last_tick, tick)
    last_tick = tick

    # sync gap -> new frame
    if dt > SYNC_GAP_US:
        chan_index = 0
        last_frame_time = time.time()
        return

    if chan_index < CHANNELS:
        channels[chan_index] = dt
        chan_index += 1

cb = pi.callback(PPM_PIN, pigpio.RISING_EDGE, ppm_callback)

def set_all_motors(pw):
    for pin in ESC_PINS:
        pi.set_servo_pulsewidth(pin, pw)

# Start safe: motors OFF / minimum
set_all_motors(THROTTLE_MIN)
print("Started. Motors set to 1000us (OFF).")
print("Safety: CH5 ARM switch required, throttle must be low to arm. CTRL+C to quit.\n")

armed = False

try:
    while True:
        # Failsafe: if no frames recently, kill motors
        if (time.time() - last_frame_time) > FAILSAFE_TIMEOUT_S:
            if armed:
                print("FAILSAFE: Lost PPM frames -> disarming + motors OFF")
            armed = False
            set_all_motors(THROTTLE_MIN)
            time.sleep(0.05)
            continue

        # Read channels (clamped)
        thr_in = clamp(channels[CH_THROTTLE], THROTTLE_MIN, THROTTLE_MAX)
        arm_in = channels[CH_ARM]

        want_arm = arm_in > ARM_THRESHOLD

        # Arming logic: must have throttle low to arm
        if want_arm and not armed:
            if thr_in <= THROTTLE_ARM_MAX:
                armed = True
                print("ARMED")
            else:
                armed = False
                set_all_motors(THROTTLE_MIN)
                print("Arm blocked: throttle not low")
                time.sleep(0.1)
                continue

        if not want_arm and armed:
            armed = False
            set_all_motors(THROTTLE_MIN)
            print("DISARMED")

        # Output
        if armed:
            # optional scaling could go here; currently pass-through
            thr_out = clamp(thr_in, THROTTLE_OUTPUT_MIN, THROTTLE_OUTPUT_MAX)
            set_all_motors(thr_out)
        else:
            set_all_motors(THROTTLE_MIN)

        # Small status print (10 Hz)
        print(f"ARM={armed}  CH3(thr)={thr_in}  CH5(arm)={arm_in}      ", end="\r")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    # Always stop motors on exit
    set_all_motors(THROTTLE_MIN)
    cb.cancel()
    pi.stop()
    print("Motors set to 1000us and pigpio stopped.")
