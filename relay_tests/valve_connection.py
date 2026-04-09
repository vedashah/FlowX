import board
import busio
from digitalio import Direction
from adafruit_mcp230xx.mcp23017 import MCP23017
import time

# I2C setup
i2c = busio.I2C(board.SCL, board.SDA)
mcp = MCP23017(i2c)

# Map pins to valves
valve1 = mcp.get_pin(0)   # GPA0
valve2 = mcp.get_pin(7)   # GPA7

# Set pins as outputs
valve1.direction = Direction.OUTPUT
valve2.direction = Direction.OUTPUT

# Test loop
while True:
    valve1.value = False  # OFF
    valve2.value = False
    time.sleep(2)
    
    valve1.value = True   # ON
    valve2.value = True
    time.sleep(2)
