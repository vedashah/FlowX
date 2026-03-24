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

# ------------------ Valve Layout ------------------
# GPA0 = Valve 1 = Back Left
# GPA1 = Valve 2 = Front Left
# GPA2 = Valve 3 = Front Right
# GPA3 = Valve 4 = Back Right

BACK_LEFT   = valves[0]
FRONT_LEFT  = valves[1]
FRONT_RIGHT = valves[2]
BACK_RIGHT  = valves[3]


def all_off():
    for valve in valves:
        valve.value = False


def roll_left():
    all_off()
    FRONT_RIGHT.value = True
    BACK_RIGHT.value = True
    print("ROLL LEFT active (Front Right + Back Right)")


def roll_right():
    all_off()
    FRONT_LEFT.value = True
    BACK_LEFT.value = True
    print("ROLL RIGHT active (Front Left + Back Left)")


def pitch_forward():
    all_off()
    BACK_LEFT.value = True
    BACK_RIGHT.value = True
    print("PITCH FORWARD active (Back Left + Back Right)")


def pitch_back():
    all_off()
    FRONT_LEFT.value = True
    FRONT_RIGHT.value = True
    print("PITCH BACK active (Front Left + Front Right)")


try:
    print("Control Mode")
    print("1 = Roll Left")
    print("2 = Roll Right")
    print("3 = Pitch Forward")
    print("4 = Pitch Back")
    print("0 = All Off")
    print("Ctrl+C = Exit")

    while True:
        cmd = input("Enter command: ")

        if cmd == "1":
            roll_left()
        elif cmd == "2":
            roll_right()
        elif cmd == "3":
            pitch_forward()
        elif cmd == "4":
            pitch_back()
        elif cmd == "0":
            all_off()
            print("All valves OFF")
        else:
            print("Invalid command")

except KeyboardInterrupt:
    print("\nShutting down safely...")
    all_off()
    sys.exit(0)