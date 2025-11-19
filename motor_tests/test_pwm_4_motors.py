"""
test_pwm_4_motors.py

Simple 4-motor ESC test using Raspberry Pi PWM.

- Drives 4 brushless motors via ESCs on 4 GPIO pins.
- Sends standard 50 Hz servo PWM (1–2 ms pulses).
- Arms ESCs at zero throttle, then ramps up in steps.

!!! SAFETY !!!
- REMOVE ALL PROPS before running this.
- Secure the motors so they cannot move.
"""

import time
import RPi.GPIO as GPIO

# === USER CONFIG ==========================================================

# BCM pin numbers for the 4 ESC signal wires
MOTOR_PINS = [17, 18, 27, 22]  # <-- CHANGE THESE to match your wiring

PWM_FREQUENCY_HZ = 50  # Standard for ESC/servo (20 ms period)

# Pulse limits (in milliseconds) for your ESCs
MIN_PULSE_MS = 1.0     # 1.0 ms  -> minimum throttle
MAX_PULSE_MS = 2.0     # 2.0 ms  -> maximum throttle

ARM_TIME_SEC = 5       # How long to sit at zero throttle to arm ESCs
STEP_HOLD_SEC = 3      # How long to hold each throttle step

# Throttle levels to test (0.0 = min, 1.0 = max)
THROTTLE_STEPS = [0.1, 0.3, 0.6, 0.2, 0.1]

# ==========================================================================


def throttle_to_duty_cycle(throttle: float) -> float:
    """
    Map a throttle value in [0.0, 1.0] to a duty cycle percentage
    for a 50 Hz PWM signal with 1–2 ms pulses.
    """
    throttle = max(0.0, min(1.0, throttle))  # clamp
    period_ms = 1000.0 / PWM_FREQUENCY_HZ    # 20 ms at 50 Hz

    min_dc = (MIN_PULSE_MS / period_ms) * 100.0
    max_dc = (MAX_PULSE_MS / period_ms) * 100.0

    return min_dc + throttle * (max_dc - min_dc)


def main():
    print("=== FlowX 4-Motor ESC Test ===")
    print("MOTOR_PINS (BCM):", MOTOR_PINS)
    print("!!! MAKE SURE ALL PROPS ARE REMOVED !!!")
    input("Press Enter to ARM ESCs at zero throttle...")

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    pwms = []

    try:
        # Set up all motor pins
        for pin in MOTOR_PINS:
            GPIO.setup(pin, GPIO.OUT)
            pwm = GPIO.PWM(pin, PWM_FREQUENCY_HZ)
            pwm.start(0.0)
            pwms.append(pwm)

        # Helper to set all motors to same throttle
        def set_all(throttle: float):
            dc = throttle_to_duty_cycle(throttle)
            for pwm in pwms:
                pwm.ChangeDutyCycle(dc)

        # Arm sequence: all motors at zero throttle
        print(f"Arming ESCs at 0.0 throttle for {ARM_TIME_SEC} seconds...")
        set_all(0.0)
        time.sleep(ARM_TIME_SEC)

        # Throttle ramp
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
        # Safe shutdown
        print("Shutting down PWM and cleaning up GPIO.")
        for pwm in pwms:
            pwm.ChangeDutyCycle(0.0)
            pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
