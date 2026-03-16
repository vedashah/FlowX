import serial
import struct
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

MSP_ATTITUDE = 108

def msp_request(cmd: int) -> bytes:
    length = 0
    checksum = length ^ cmd
    return b"$M<" + bytes([length, cmd, checksum])

def read_msp_response(ser: serial.Serial):
    # Find '$M>'
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b == b"$":
            hdr = ser.read(2)
            if hdr == b"M>":
                break

    size = ser.read(1)
    cmd  = ser.read(1)
    if not size or not cmd:
        return None

    size = size[0]
    cmd  = cmd[0]

    payload = ser.read(size)
    chk = ser.read(1)
    if len(payload) != size or len(chk) != 1:
        return None

    # checksum verify
    c = size ^ cmd
    for x in payload:
        c ^= x
    if c != chk[0]:
        return None

    return cmd, payload

ser = serial.Serial(PORT, BAUD, timeout=0.2)

while True:
    ser.write(msp_request(MSP_ATTITUDE))
    resp = read_msp_response(ser)
    if resp is None:
        print("No response")
        continue

    cmd, payload = resp
    if cmd == MSP_ATTITUDE and len(payload) == 6:
        roll_tenth, pitch_tenth, heading = struct.unpack("<hhh", payload)
        roll  = roll_tenth / 10.0
        pitch = pitch_tenth / 10.0
        yaw   = heading * 1.0
        print(f"Roll={roll:6.1f}°  Pitch={pitch:6.1f}°  Yaw={yaw:6.0f}°", end="\r")

    time.sleep(0.02)  # 50 Hz
