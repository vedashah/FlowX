import serial

# Open UART (default Pi UART)
ser = serial.Serial(
    port="/dev/serial0",
    baudrate=420000,   # CRSF default baud rate
    timeout=1
)

print("Listening for ELRS / CRSF data...")

while True:
    data = ser.read(64)  # Read up to 64 bytes
    if data:
        print(data.hex())
