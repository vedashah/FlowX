import serial
import time

ser = serial.Serial('/dev/serial0', 420000, timeout=0.01)

print("CHANNEL SCANNER - Flip your SA switch and see which CH changes!")

try:
    while True:
        ser.reset_input_buffer()
        data = ser.read(26)
        if len(data) >= 24 and data[0] == 0xc8:
            # Decoding all 8 possible channels
            ch1 = ((data[3] | data[4] << 8) & 0x07FF)
            ch2 = ((data[4] >> 3 | data[5] << 5) & 0x07FF)
            ch3 = ((data[5] >> 6 | data[6] << 2 | data[7] << 10) & 0x07FF)
            ch4 = ((data[7] >> 1 | data[8] << 7) & 0x07FF)
            ch5 = ((data[8] >> 4 | data[9] << 4) & 0x07FF)
            ch6 = ((data[9] >> 7 | data[10] << 1 | data[11] << 9) & 0x07FF)
            ch7 = ((data[11] >> 2 | data[12] << 6) & 0x07FF)
            ch8 = ((data[12] >> 5 | data[13] << 3) & 0x07FF)

            print(f"CH1:{ch1:4} CH2:{ch2:4} CH3:{ch3:4} CH4:{ch4:4} | CH5:{ch5:4} CH6:{ch6:4} CH7:{ch7:4} CH8:{ch8:4}", end='\r')
        time.sleep(0.01)
except KeyboardInterrupt:
    ser.close()