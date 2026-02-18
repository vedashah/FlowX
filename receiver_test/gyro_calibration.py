from smbus2 import SMBus
import time

# ========= MPU SETUP =========
MPU_ADDR = 0x68
PWR_MGMT_1  = 0x6B
GYRO_XOUT_H = 0x43
GYRO_SCALE  = 131.0  # ±250°/s

bus = SMBus(1)
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

# ========= FUNCTIONS =========
def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low  = bus.read_byte_data(MPU_ADDR, reg + 1)
    value = (high << 8) | low
    if value >= 0x8000:
        value -= 65536
    return value

def read_gyro():
    gx = read_word(GYRO_XOUT_H)     / GYRO_SCALE
    gy = read_word(GYRO_XOUT_H + 2) / GYRO_SCALE
    gz = read_word(GYRO_XOUT_H + 4) / GYRO_SCALE
    return gx, gy, gz

# ========= CALIBRATION =========
print("Calibrating gyro... Keep drone still.")

bias_x = 0
bias_y = 0
bias_z = 0

samples = 500

for _ in range(samples):
    gx, gy, gz = read_gyro()
    bias_x += gx
    bias_y += gy
    bias_z += gz
    time.sleep(0.002)

bias_x /= samples
bias_y /= samples
bias_z /= samples

print("Calibration complete.")
print("Bias values:")
print("X:", bias_x)
print("Y:", bias_y)
print("Z:", bias_z)

bus.close()
