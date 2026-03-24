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


def all_off():
    for valve in valves:
        valve.value = False


try:
    print("Valve Control Mode")
    print("1-8 = Turn on that valve")
    print("0   = All Off")
    print("Ctrl+C = Exit")

    while True:
        cmd = input("Enter valve number: ").strip()

        if cmd == "0":
            all_off()
            print("All valves OFF")

        elif cmd.isdigit() and 1 <= int(cmd) <= 8:
            all_off()
            idx = int(cmd) - 1
            valves[idx].value = True
            print(f"Valve {cmd} ON (GPA{idx})")

        else:
            print("Invalid command. Enter 0-8.")

except KeyboardInterrupt:
    print("\nShutting down safely...")
    all_off()
    sys.exit(0)