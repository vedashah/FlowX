# Receiver Test
### *FlowX – RC Receiver & Gyro Input Validation*

The `receiver_test/` directory contains scripts for validating RC receiver input and gyroscope sensor data independently before integrating either into the main flight controller.

---

## Scripts

| File | Description |
|------|-------------|
| `receiver_input_test.py` | Decodes PPM frames from the RC receiver via a `pigpio` callback and prints all 8 channel values live to the terminal |
| `gyro_calibration.py` | Collects 500 static MPU6050 gyro samples over I²C and computes X/Y/Z bias offsets for zeroing |
| `gyro_tests.py` | ⚠️ **Empty — not yet implemented.** Placeholder for gyro streaming and noise floor analysis. |

---

## `receiver_input_test.py` — Details

Reads PPM (Pulse Position Modulation) input on **GPIO 17** using a `pigpio` rising-edge callback.

- **Sync gap:** frames separated by pulses > 3 000 µs
- **Channels decoded:** 8
- **Output:** Live terminal readout of all 8 channel pulse widths (µs)

**Run:**
```bash
sudo pigpiod
python3 receiver_input_test.py
```

---

## `gyro_calibration.py` — Details

Reads the **MPU6050** directly over **I²C** (bus 1, address `0x68`) via `smbus2`. Collects 500 samples at 2 ms intervals while the drone is stationary, then reports mean bias per axis.

**Example output:**
```
Calibrating gyro... Keep drone still.
Calibration complete.
Bias values:
X: 0.0381
Y: -0.1145
Z: 0.0229
```

**Scale:** ±250°/s → `GYRO_SCALE = 131.0 LSB/(°/s)`

Apply these offsets in `hybrid_rate_controller.py` if using the MPU6050 as the primary IMU. The Betaflight FC performs its own internal calibration when reading via MSP.

**Run:**
```bash
python3 gyro_calibration.py
```

---

## IMU Source Reference

Two gyro paths exist in the FlowX system — know which one you're using:

| Source | Interface | Used in |
|--------|-----------|---------|
| MPU6050 (standalone) | I²C via `smbus2` | `gyro_calibration.py`, `motor_tests/IMU_data_display.py` |
| Betaflight F722 FC | MSP over USB (`/dev/ttyACM0`) | `hybrid_rate_controller.py`, `pi_fc_connection_test.py` |

The **MSP path (Betaflight)** is preferred for closed-loop flight control — it benefits from the FC's onboard filtering pipeline.

---

## Dependencies

```bash
sudo pigpiod
pip install -r ../requirements.txt
```

- PPM receiver must be wired to **GPIO 17**
- MPU6050 must be on **I²C bus 1** at address `0x68`
