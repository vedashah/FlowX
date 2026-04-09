import board
import busio
from digitalio import Direction
from adafruit_mcp230xx.mcp23017 import MCP23017
import RPi.GPIO as GPIO
import serial
import time
import sys

# ------------------ I2C + MCP23017 Setup ------------------
i2c = busio.I2C(board.SCL, board.SDA)
mcp = MCP23017(i2c)

# ------------------ RGB LED Setup ------------------
LED_R = 24
LED_G = 25
LED_B = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_R, GPIO.OUT)
GPIO.setup(LED_G, GPIO.OUT)
GPIO.setup(LED_B, GPIO.OUT)

def set_led(r, g, b):
    GPIO.output(LED_R, r)
    GPIO.output(LED_G, g)
    GPIO.output(LED_B, b)

valves = []
for pin_num in range(16):
    valve = mcp.get_pin(pin_num)
    valve.direction = Direction.OUTPUT
    valve.value = False
    valves.append(valve)

# Valve layout — pairs per corner, odd=vertical, even=horizontal
# GPA0 = Valve 1 (Back Left  - Vertical)
# GPA1 = Valve 2 (Back Left  - Horizontal/Yaw)
# GPA2 = Valve 3 (Back Right - Vertical)
# GPA3 = Valve 4 (Back Right - Horizontal/Yaw)
# GPA4 = Valve 5 (Front Right - Vertical)
# GPA5 = Valve 6 (Front Right - Horizontal/Yaw)
# GPA6 = Valve 7 (Front Left  - Vertical)
# GPA7 = Valve 8 (Front Left  - Horizontal/Yaw)

BACK_LEFT_V    = valves[0]   # GPA0 - Vertical
BACK_LEFT_H    = valves[1]   # GPA1 - Horizontal (Yaw)
BACK_RIGHT_V   = valves[2]   # GPA2 - Vertical
BACK_RIGHT_H   = valves[3]   # GPA3 - Horizontal (Yaw)
FRONT_RIGHT_V  = valves[4]   # GPA4 - Vertical
FRONT_RIGHT_H  = valves[5]   # GPA5 - Horizontal (Yaw)
FRONT_LEFT_V   = valves[6]   # GPA6 - Vertical
FRONT_LEFT_H   = valves[7]   # GPA7 - Horizontal (Yaw)

# ------------------ SBUS Serial Setup ------------------
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

# ------------------ Thresholds ------------------
CENTER    = 1024
DEADBAND  = 250
THR_ON    = 400

DISARM_COUNT_NEEDED = 5

# ------------------ Helpers ------------------
def all_off():
    for v in valves:
        v.value = False

# ------------------ Main Loop ------------------
print("SBUS Thruster RC Control")
print("Left stick  UP    = All lift valves")
print("Left stick  LEFT  = Yaw CCW")
print("Left stick  RIGHT = Yaw CW")
print("Right stick LEFT  = Roll Left")
print("Right stick RIGHT = Roll Right")
print("Right stick UP    = Pitch Forward")
print("Right stick DOWN  = Pitch Back")
print("CH6 switch high   = DISARM")
print("Ctrl+C to exit\n")

armed = False
disarm_counter = 0
set_led(True, False, False)

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

        # ---------- Arming logic ----------
        if arm_raw > 200 and armed:
            disarm_counter += 1
            if disarm_counter >= DISARM_COUNT_NEEDED:
                armed = False
                disarm_counter = 0
                set_led(True, False, False)
                all_off()
                print(f"\nDISARMED (arm_raw={arm_raw}, held {DISARM_COUNT_NEEDED} frames)")
        else:
            disarm_counter = 0

        if not armed:
            if thr_raw < 200 and yaw_raw < 200 and arm_raw < 200:
                armed = True
                set_led(False, True, False)
                print(f"\nARMED (thr={thr_raw} yaw={yaw_raw} arm={arm_raw})")

        if not armed:
            all_off()
            print(f"DISARMED | Raw={roll_raw},{pitch_raw},{thr_raw},{yaw_raw},{arm_raw}   ", end="\r")
            time.sleep(0.02)
            continue

        # ---------- Determine commands from sticks ----------
        do_throttle   = thr_raw > THR_ON
        do_yaw_cw     = yaw_raw > (CENTER + DEADBAND)
        do_yaw_ccw    = yaw_raw < (CENTER - DEADBAND)
        do_roll_right = roll_raw > (CENTER + DEADBAND)
        do_roll_left  = roll_raw < (CENTER - DEADBAND)
        do_pitch_fwd  = pitch_raw > (CENTER + DEADBAND)
        do_pitch_back = pitch_raw < (CENTER - DEADBAND)

        # ---------- Set vertical valves (throttle / roll / pitch) ----------
        # Corner fires if throttle is commanded OR its corner is needed for roll/pitch.
        # Roll right: left side fires  | Roll left: right side fires
        # Pitch fwd:  back side fires  | Pitch back: front side fires
        BACK_LEFT_V.value   = do_throttle or do_pitch_fwd  or do_roll_right
        BACK_RIGHT_V.value  = do_throttle or do_pitch_fwd  or do_roll_left
        FRONT_RIGHT_V.value = do_throttle or do_pitch_back or do_roll_left
        FRONT_LEFT_V.value  = do_throttle or do_pitch_back or do_roll_right

        # ---------- Set horizontal valves (yaw) ----------
        # Yaw CW:  fire all horizontal valves in CW tangential direction
        # Yaw CCW: fire all horizontal valves in CCW tangential direction
        # Both A and B sides fire together for symmetric torque.
        BACK_LEFT_H.value   = do_yaw_cw
        BACK_RIGHT_H.value  = do_yaw_ccw
        FRONT_RIGHT_H.value = do_yaw_cw
        FRONT_LEFT_H.value  = do_yaw_ccw

        # ---------- Debug print ----------
        state = []
        if do_throttle:   state.append("THR")
        if do_yaw_cw:     state.append("YAW_CW")
        if do_yaw_ccw:    state.append("YAW_CCW")
        if do_roll_left:  state.append("ROLL_L")
        if do_roll_right: state.append("ROLL_R")
        if do_pitch_fwd:  state.append("PITCH_F")
        if do_pitch_back: state.append("PITCH_B")
        if not state:     state.append("IDLE")

        dc = f" dc={disarm_counter}" if disarm_counter > 0 else ""
        print(
            f"ARMED | {'+'.join(state):30s} | "
            f"Raw={roll_raw},{pitch_raw},{thr_raw},{yaw_raw} arm={arm_raw}{dc}   ",
            end="\r"
        )
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nShutting down...")

finally:
    all_off()
    ser.close()
    GPIO.cleanup()
    sys.exit(0)