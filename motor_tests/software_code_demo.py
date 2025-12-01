"""
software_code_demo.py

Demo for capstone:
1. Arm ESCs
2. Run each motor individually (5 seconds each)
3. Pause
4. Run all motors at low throttle (5 seconds)
5. Run all motors at high throttle (5 seconds)
6. Stop all motors
"""

import time
import RPi.GPIO as GPIO

# === CONFIG ===============================================================

MOTOR_PINS = [17, 18, 27, 22]     # BCM numbering
PWM_FREQUENCY_HZ = 50

MIN_PULSE_MS = 1.0      # 1.0 ms = zero throttle
MAX_PULSE_MS = 2.0      # 2.0 ms = full throttle

ARM_TIME_SEC = 5
INDIVIDUAL_RUN_SEC = 5
GROUP_RUN_SEC = 5

LOW_THROTTLE = 0.2
HIGH_THROTTLE = 0.7

# ===========================================================================


def throttle_to_dc(throttle: float) -> float:
    """Convert 0.0–1.0 throttle → duty cycle (%) for 50 Hz PWM."""
    throttle = max(0.0, min(1.0, throttle))
    period_ms = 1000.0 / PWM_FREQUENCY_HZ

    min_dc = (MIN_PULSE_MS / period_ms) * 100
    max_dc = (MAX_PULSE_MS / period_ms) * 100

    return min_dc + throttle * (max_dc - min_dc)


def main():
    print("=== FlowX Motor Demo (Individual + Group) ===")
    print("Motor Pins:", MOTOR_PINS)
    print("REMOVE PROPS BEFORE RUNNING.")
    input("Press Enter to ARM...")

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Initialize PWM objects
    pwms = []
    for pin in MOTOR_PINS:
        GPIO.setup(pin, GPIO.OUT)
        p = GPIO.PWM(pin, PWM_FREQUENCY_HZ)
        p.start(0)
        pwms.append(p)

    def set_motor(index, throttle):
        dc = throttle_to_dc(throttle)
        pwms[index].ChangeDutyCycle(dc)

    def set_all(throttle):
        dc = throttle_to_dc(throttle)
        for p in pwms:
            p.ChangeDutyCycle(dc)

    try:
        # ARM ALL ESCs
        print(f"Arming ESCs at 0 throttle for {ARM_TIME_SEC} sec...")
        set_all(0.0)
        time.sleep(ARM_TIME_SEC)

        # INDIVIDUAL MOTOR TEST
        for i, _ in enumerate(MOTOR_PINS):
            print(f"\nRunning Motor {i+1} for {INDIVIDUAL_RUN_SEC} sec...")
            set_motor(i, LOW_THROTTLE)
            time.sleep(INDIVIDUAL_RUN_SEC)
            set_motor(i, 0.0)
            time.sleep(1)

        print("\nPausing 3 seconds before group test...")
        time.sleep(3)

        # GROUP TEST – LOW SPEED
        print(f"Running ALL motors at LOW throttle ({LOW_THROTTLE})...")
        set_all(LOW_THROTTLE)
        time.sleep(GROUP_RUN_SEC)

        # GROUP TEST – HIGH SPEED
        print(f"Running ALL motors at HIGH throttle ({HIGH_THROTTLE})...")
        set_all(HIGH_THROTTLE)
        time.sleep(GROUP_RUN_SEC)

        # SHUT ALL DOWN
        print("Stopping all motors...")
        set_all(0.0)
        time.sleep(2)

        print("Demo complete.")

    except KeyboardInterrupt:
        print("\n[CTRL+C] Stopping motors...")

    finally:
        print("Cleaning up GPIO...")
        set_all(0.0)
        for p in pwms:
            p.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
