#!/usr/bin/env python3
import serial
import time
import smbus

# ============================
# SERIAL (CRSF RECEIVER)
# ============================
ser = serial.Serial('/dev/serial0', 420000, timeout=0.01)

def read_channels():
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

# ============================
# MCP23017 (Adafruit 12V Solenoid Driver)
# ============================
bus = smbus.SMBus(1)
MCP_ADDR = 0x20

IODIRA = 0x00  # Direction register for GPA0-7
OLATA  = 0x14  # Output latch register

# Set GPA0..7 as outputs
bus.write_byte_data(MCP_ADDR, IODIRA, 0x00)

# ============================
# VALVE MAPPING
# ============================
FL = 0
FR = 1
RR = 2
RL = 3
YAW_CW1  = 4
YAW_CW2  = 5
YAW_CCW1 = 6
YAW_CCW2 = 7
NUM_VALVES = 8
CHANNELS = [FL, FR, RR, RL, YAW_CW1, YAW_CW2, YAW_CCW1, YAW_CCW2]

# ============================
# CONTROL GAINS & LIMITS
# ============================
ROLL_GAIN  = 0.4
PITCH_GAIN = 0.4
TICK_S = 0.2   # 200 ms pulse density tick
DEADBAND = 20
MAX_VALVES_ON = 4   # Hardware limit

# ============================
# HELPER FUNCTIONS
# ============================
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def normalize_axis(val):
    centered = val - 1024
    if abs(centered) < DEADBAND:
        return 0.0
    return clamp(centered / 660.0, -1.0, 1.0)

def normalize_throttle(val):
    return clamp(val / 1811.0, 0.0, 1.0)

# ============================
# PULSE-DENSITY SCHEDULER
# ============================
class PulseDensityScheduler:
    def __init__(self, num_valves, window_ticks=5):
        self.n = num_valves
        self.W = window_ticks
        self.acc = [0.0] * num_valves

    def step(self, duties):
        fired = [False]*self.n
        for i in range(self.n):
            d = clamp(duties[i], 0.0, 1.0)
            self.acc[i] += d
            if self.acc[i] >= 1.0:
                fired[i] = True
                self.acc[i] -= 1.0
        return fired

    def refund(self, idx_list):
        for i in idx_list:
            self.acc[i] += 1.0

scheduler = PulseDensityScheduler(NUM_VALVES)

def apply_tick(duty):
    requested = scheduler.step(duty)

    # Enforce MAX_VALVES_ON
    on_indices = [i for i, r in enumerate(requested) if r]
    if len(on_indices) > MAX_VALVES_ON:
        # Choose top MAX_VALVES_ON by duty
        scored = [(duty[i], i) for i in on_indices]
        scored.sort(reverse=True)
        keep = set([i for _, i in scored[:MAX_VALVES_ON]])
        suppressed = [i for i in on_indices if i not in keep]
        scheduler.refund(suppressed)
        on_indices = list(keep)

    # Convert to bitmask
    bits = 0
    for i in on_indices:
        bits |= (1 << CHANNELS[i])
    bus.write_byte_data(MCP_ADDR, OLATA, bits)
    time.sleep(TICK_S)
    # Turn off all for next tick
    bus.write_byte_data(MCP_ADDR, OLATA, 0x00)

# ============================
# MAIN LOOP
# ============================
armed = False
print("MCP23017 8-Valve Thruster Controller Running...")

try:
    while True:
        ch = read_channels()
        if ch is None:
            continue

        roll_raw  = ch[0]
        pitch_raw = ch[1]
        thr_raw   = ch[2]
        yaw_raw   = ch[3]
        arm_raw   = ch[5]

        # ARM / DISARM LOGIC
        if arm_raw > 200 and armed:
            armed = False
            print("\nDISARMED")

        if not armed and thr_raw < 200 and yaw_raw > 1600 and arm_raw < 200:
            armed = True
            print("\nARMED")

        # Normalize
        roll  = normalize_axis(roll_raw) * ROLL_GAIN
        pitch = normalize_axis(pitch_raw) * PITCH_GAIN
        thr   = normalize_throttle(thr_raw)
        yaw   = normalize_axis(yaw_raw)

        # MIXING
        fl = thr + pitch + roll
        fr = thr + pitch - roll
        rr = thr - pitch - roll
        rl = thr - pitch + roll

        # YAW pairs: mutually exclusive
        if yaw > 0:
            yaw_cw  = clamp(yaw, 0.0, 1.0)
            yaw_ccw = 0.0
        elif yaw < 0:
            yaw_cw  = 0.0
            yaw_ccw = clamp(-yaw, 0.0, 1.0)
        else:
            yaw_cw  = 0.0
            yaw_ccw = 0.0

        # Build duty array
        duties = [
            fl, fr, rr, rl,
            yaw_cw, yaw_cw,   # CW pair
            yaw_ccw, yaw_ccw  # CCW pair
        ]

        # Apply
        if armed:
            apply_tick(duties)
        else:
            bus.write_byte_data(MCP_ADDR, OLATA, 0x00)

        # Print raw channels for debugging
        print(
            f"{'ARMED' if armed else 'DISARMED'} | "
            f"Raw CH: {roll_raw},{pitch_raw},{thr_raw},{yaw_raw},{arm_raw}",
            end="\r"
        )

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    bus.write_byte_data(MCP_ADDR, OLATA, 0x00)
    ser.close()