import board
import busio
from digitalio import Direction
from adafruit_mcp230xx.mcp23017 import MCP23017
import time
import sys

# ------------------ I2C Setup ------------------
i2c = busio.I2C(board.SCL, board.SDA)
mcp = MCP23017(i2c)

# ------------------ Valve Mapping ------------------
valves = []

for pin_num in range(8):  # GPA0–GPA7
    valve = mcp.get_pin(pin_num)
    valve.direction = Direction.OUTPUT
    valve.value = False  # Start OFF
    valves.append(valve)

single_delay = 0.5  # seconds


def all_off():
    for valve in valves:
        valve.value = False


try:
    # ==============================
    # PASS 1: One at a time
    # ==============================
    previous_valve = None

    for valve in valves:
        if previous_valve is not None:
            previous_valve.value = False
        
        valve.value = True
        time.sleep(single_delay)
        
        previous_valve = valve

    # Turn last OFF before stacking
    valves[-1].value = False
    time.sleep(1)

    # ==============================
    # PASS 2: Stack ON
    # ==============================
    for valve in valves:
        valve.value = True
        time.sleep(single_delay)

    print("All valves ON. Press Ctrl+C to exit safely.")

    # ==============================
    # EXIT LOOP (wait here)
    # ==============================
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\nShutting down safely...")
    all_off()
    print("All valves OFF.")
    sys.exit(0)