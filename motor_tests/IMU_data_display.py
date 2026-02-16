from smbus2 import SMBus
import time

# Device address (usually 0x68)
MPU_ADDR = 0x68

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

# Scale modifiers (default settings)
ACCEL_SCALE = 16384.0   # LSB/g (±2g)
GYRO_SCALE  = 131.0     # LSB/(°/s) (±250°/s)


def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low

    # Convert to signed 16-bit
    if value >= 0x8000:
        value -= 65536
    return value


with SMBus(1) as bus:

    # Wake up MPU6050 (it starts in sleep mode)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

    print("MPU6050 initialized...")
    time.sleep(1)

    while True:
        # Accelerometer raw
        ax_raw = read_word(bus, MPU_ADDR, ACCEL_XOUT_H)
        ay_raw = read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 2)
        az_raw = read_word(bus, MPU_ADDR, ACCEL_XOUT_H + 4)

        # Gyroscope raw
        gx_raw = read_word(bus, MPU_ADDR, GYRO_XOUT_H)
        gy_raw = read_word(bus, MPU_ADDR, GYRO_XOUT_H + 2)
        gz_raw = read_word(bus, MPU_ADDR, GYRO_XOUT_H + 4)

        # Convert to physical units
        ax = ax_raw / ACCEL_SCALE
        ay = ay_raw / ACCEL_SCALE
        az = az_raw / ACCEL_SCALE

        gx = gx_raw / GYRO_SCALE
        gy = gy_raw / GYRO_SCALE
        gz = gz_raw / GYRO_SCALE

        print(f"Accel (g):  X={ax:.2f}  Y={ay:.2f}  Z={az:.2f}")
        print(f"Gyro (°/s): X={gx:.2f}  Y={gy:.2f}  Z={gz:.2f}")
        print("-" * 40)

        time.sleep(0.5)
