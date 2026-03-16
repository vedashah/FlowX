#!/usr/bin/env python3
import serial
import time
import math

# ==============================
# SERIAL SETUP
# ==============================
ser = serial.Serial('/dev/serial0', 420000, timeout=0.02)

# ==============================
# CONFIGURATION
# ==============================
TICK_S = 0.2  # 200 ms valve actuation window
DEADBAND = 0.05
MAX_CMD = 1.0

# ==============================
# NORMALIZATION FUNCTIONS
# ==============================
def normalize_axis(value, center=1024, span=660):
    norm = (value - center) / span
    if abs(norm) < DEADBAND:
        return 0.0
    return max(-MAX_CMD, min(MAX_CMD, norm))

def normalize_throttle(value, min_val=172, max_val=1811):
    norm = (value - min_val) / (max_val - min_val)
    return max(0.0, min(1.0, norm))

# ==============================
# MIXING FUNCTION
# ==============================
def mix_controls(thr, roll, pitch, yaw):
    # Vertical mixing
    fl = thr + pitch + roll
    fr = thr + pitch - roll
    rr = thr - pitch - roll
    rl = thr - pitch + roll

    # Clamp
    fl = max(0.0, min(1.0, fl))
    fr = max(0.0, min(1.0, fr))
    rr = max(0.0, min(1.0, rr))
    rl = max(0.0, min(1.0, rl))

    # Yaw channels (separate CW/CCW thrusters)
    yaw_cw = max(0.0, yaw)
    yaw_ccw = max(0.0, -yaw)

    return fl, fr, rr, rl, yaw_cw, yaw_ccw

# ==============================
# VALVE CONTROL (PLACEHOLDER)
# ==============================
def apply_tick(fl, fr, rr, rl, yaw_cw, yaw_ccw):
    # Replace with actual GPIO / MOSFET control logic
    # Example: digitalWrite(pin, HIGH) etc.

    # For now just simulate timing window
    time.sleep(TICK_S)

# ==============================
# MAIN LOOP
# ==============================
print("Starting RC Thruster Logic...")

armed = False

while True:
    try:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue

        parts = line.split(',')
        if len(parts) < 5:
            continue

        roll_raw = int(parts[0])
        pitch_raw = int(parts[1])
        thr_raw = int(parts[2])
        yaw_raw = int(parts[3])
        arm_raw = int(parts[4])

        # Normalize
        roll = normalize_axis(roll_raw)
        pitch = normalize_axis(pitch_raw)
        yaw = normalize_axis(yaw_raw)
        thr = normalize_throttle(thr_raw)

        # Arm logic (example threshold)
        armed = arm_raw > 1500

        if armed:
            fl, fr, rr, rl, yaw_cw, yaw_ccw = mix_controls(thr, roll, pitch, yaw)
            apply_tick(fl, fr, rr, rl, yaw_cw, yaw_ccw)
        else:
            fl = fr = rr = rl = yaw_cw = yaw_ccw = 0.0

        # ==============================
        # DEBUG OUTPUT (ALWAYS ON)
        # ==============================
        print(
            f"RAW: R={roll_raw:4d} P={pitch_raw:4d} T={thr_raw:4d} "
            f"Y={yaw_raw:4d} ARM={arm_raw:4d} | "
            f"NORM thr={thr:.2f} r={roll:.2f} p={pitch:.2f} y={yaw:.2f} | "
            f"{'ARMED' if armed else 'DISARMED'}      ",
            end="\r"
        )

    except Exception as e:
        print("\nError:", e)
        time.sleep(0.1)