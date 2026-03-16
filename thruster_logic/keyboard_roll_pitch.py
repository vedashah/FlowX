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
    valve.value = False
    valves.append(valve)


# Example valve groups (change these for your nozzle layout)
ROLL_VALVES = [valves[0], valves[3]]     # example
PITCH_VALVES = [valves[1], valves[2]]    # example


def all_off():
    for valve in valves:
        valve.value = False


def roll():
    all_off()
    for valve in ROLL_VALVES:
        valve.value = True
    print("ROLL command active")


def pitch():
    all_off()
    for valve in PITCH_VALVES:
        valve.value = True
    print("PITCH command active")


try:

    print("Control Mode")
    print("1 = Roll")
    print("2 = Pitch")
    print("0 = All Off")
    print("Ctrl+C = Exit")

    while True:

        cmd = input("Enter command: ")

        if cmd == "1":
            roll()

        elif cmd == "2":
            pitch()

        elif cmd == "0":
            all_off()
            print("All valves OFF")

        else:
            print("Invalid command")

except KeyboardInterrupt:
    print("\nShutting down safely...")
    all_off()
    sys.exit(0)