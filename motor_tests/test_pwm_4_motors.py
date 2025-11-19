"""
test_pwm_4_motors.py

Simple 4-motor ESC test using Raspberry Pi PWM.
"""

import time
import RPi.GPIO as GPIO

# === USER CONFIG ==========================================================

MOTOR_PINS = [17, 18, 27, 22]

PWM_FREQUENCY_HZ = 50

MIN_PULSE_MS = 1.0
MAX_PULSE_MS = 2.0

ARM_TIME_SEC = 5
STEP_HOLD_SEC = 3

THROTTLE_STEPS = [0.1, 0.3, 0.6, 0.2, 0.1]

# ==========================================================================


def throttle_to_duty_cycle(throttle: float) -> float:
    throttle = max(0.0, min(1.0, throttle))
    period_ms = 1000.0 / PWM_FREQUENCY_HZ

    min_dc = (MIN_PULSE_MS / period_ms) * 100.0
    max_dc = (MAX_PULSE_MS / period_ms) * 100.0

    return min_dc + throttle * (max_dc - min_dc)


def main():
    print("=== FlowX 4-Motor ESC Test ===")
    print("MOTOR_PINS (BCM):", MOTOR_PINS)
    print("REMOVE ALL PROPS BEFORE RUNNING.")
    input("Press Enter to ARM ESCs at zero throttle...")

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    pwms = []

    try:
        for pin in MOTOR_PINS:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, PWM_FREQUENCY_HZ)
            pwm.start(0.0)
            pwms.append(pwm)

        def set_all(throttle: float):
            dc = throttle_to_duty_cycle(throttle)
            for pwm in pwms:
                pwm.ChangeDutyCycle(dc)

        print(f"Arming ESCs at 0.0 throttle for {ARM_TIME_SEC} seconds...")
        set_all(0.0)
        time.sleep(ARM_TIME_SEC)

        for t in THROTTLE_STEPS:
            print(f"Setting throttle = {t:.2f}")
            set_all(t)
            time.sleep(STEP_HOLD_SEC)

        print("Returning to zero throttle...")
        set_all(0.0)
        time.sleep(2.0)

        print("Test completed.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: Stopping motors...")

    finally:
        print("Shutting down PWM and cleaning up GPIO.")
        for pwm in pwms:
            pwm.ChangeDutyCycle(0.0)
            pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
