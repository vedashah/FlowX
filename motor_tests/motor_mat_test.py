import pigpio
import time

# ------------------ MOTOR PINS ------------------
M1 = 23
M2 = 16
M3 = 26
M4 = 27

MOTORS = {
    1: M1,
    2: M2,
    3: M3,
    4: M4
}

PWM_MIN = 1000      # ESC idle
PWM_SPIN = 1150     # Low spin speed (adjust if needed)
SPIN_TIME = 2       # seconds

# ------------------ PIGPIO SETUP ------------------
pi = pigpio.pi()
if not pi.connected:
    print("Run this first in terminal: sudo pigpiod")
    exit()

def set_motor(pin, pw):
    pi.set_servo_pulsewidth(pin, int(pw))

def stop_all():
    for pin in MOTORS.values():
        set_motor(pin, PWM_MIN)

# ------------------ MAIN LOOP ------------------
print("\nMotor Test Mode")
print("Remove props before testing!\n")

stop_all()

try:
    while True:
        choice = input("Pick motor (1-4) or q to quit: ")

        if choice.lower() == 'q':
            break

        if choice.isdigit():
            motor_num = int(choice)

            if motor_num in MOTORS:
                pin = MOTORS[motor_num]
                print(f"Spinning Motor {motor_num} for {SPIN_TIME} seconds...")

                set_motor(pin, PWM_SPIN)
                time.sleep(SPIN_TIME)

                set_motor(pin, PWM_MIN)
                print("Done.\n")
            else:
                print("Invalid motor number.\n")
        else:
            print("Invalid input.\n")

except KeyboardInterrupt:
    pass

finally:
    print("\nStopping all motors.")
    stop_all()
    pi.stop()