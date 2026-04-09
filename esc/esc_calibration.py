import pigpio
import time

ESC_PINS = [27, 26, 23, 16]

pi = pigpio.pi()
if not pi.connected:
    print("Run: sudo pigpiod")
    raise SystemExit(1)

def set_all(pw):
    for p in ESC_PINS:
        pi.set_servo_pulsewidth(p, pw)

print("ESC CALIBRATION MODE")
print("REMOVE PROPS NOW.")
input("Press ENTER when ready...")

# Step 1: Max throttle
print("Sending MAX throttle (2000us)")
set_all(2000)

input("Now power the ESCs. Wait for calibration beeps, then press ENTER...")

# Step 2: Min throttle
print("Sending MIN throttle (1000us)")
set_all(1000)

print("Waiting 5 seconds...")
time.sleep(5)

# Done
print("Calibration done. Motors OFF.")
set_all(1000)
pi.stop()
