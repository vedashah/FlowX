# Motor Tests
### *FlowX – Motor, ESC, PWM & IMU Validation Suite*

The `motor_tests/` directory contains scripts for validating electric motor and ESC subsystems on the FlowX test bench. Scripts are organized by input type (PWM direct, SBUS, PPM) and by purpose (mixer, passthrough, scanner, IMU).

---

## Scripts

### PWM / ESC Direct Tests

| File | Description |
|------|-------------|
| `test_pwm_4_motors.py` | Arms all four ESCs then steps through a non-linear throttle sequence (`10% → 30% → 60% → 20% → 10%`). Uses `RPi.GPIO` software PWM. |
| `esc_full_ramp_demo.py` | Same structure as above but ramps linearly from 20% to 100%. Used for team demos and code reviews — not a characterization test. |
| `single_motor_test.py` | Spins a single ESC (GPIO 18) from idle to mid-throttle using `gpiozero`. Good for isolated motor bench testing. |
| `motor_matrix_test.py` | Interactive test — prompts user to pick a motor (1–4) and spins it for 2 seconds via `pigpio`. Verifies wiring and direction one motor at a time. |

### SBUS RC Input → Motor Control

| File | Description |
|------|-------------|
| `sbus_quad_mixer.py` | Full SBUS → Quad-X mixer with roll, pitch, and yaw. Correct motor layout (M1 Rear-Right, M2 Front-Right, M3 Rear-Left, M4 Front-Left) with explicit CW/CCW direction handling. |
| `sbus_roll_pitch_mixer.py` | Earlier iteration of the SBUS mixer — roll and pitch only, no yaw. Simpler baseline for testing stick response before adding yaw. |
| `sbus_channel_scanner.py` | Decodes and prints all 8 raw SBUS channel values in real time. Use this to identify which channel corresponds to which switch or stick on your transmitter. |

### PPM RC Input → Motor Control

| File | Description |
|------|-------------|
| `ppm_roll_pitch_mixer.py` | PPM input version of the roll/pitch mixer. Uses a `pigpio` callback decoder with 500 ms failsafe timeout. Arm/disarm via CH5. |
| `ppm_throttle_passthrough.py` | Passes RC throttle channel directly to all four motors. Validates the full PPM → Raspberry Pi → ESC signal path end-to-end. |

### IMU

| File | Description |
|------|-------------|
| `IMU_data_display.py` | Reads the MPU6050 over I²C (`smbus2`) and prints live accelerometer (g) and gyroscope (°/s) values at 2 Hz. Default config: ±2g / ±250°/s. |
| `smbus2_import_check.py` | Two-line sanity check — imports `smbus2` and prints a success message. Run first if the IMU scripts fail to confirm the library is installed. |

> **Note:** `esc_calibration.py` lives in [`../esc/`](../esc/). Run calibration from there.

> **Note:** For gyro calibration and MSP-based IMU reads, see [`../receiver_test/`](../receiver_test/).

---

## Motor GPIO Pin Mapping

| Motor | Position | GPIO (BCM) |
|-------|----------|------------|
| M1 | Rear Right | 23 |
| M2 | Front Right | 16 |
| M3 | Rear Left | 26 |
| M4 | Front Left | 27 |

> **Note:** `test_pwm_4_motors.py` and `esc_full_ramp_demo.py` use legacy demo pins (17, 18, 27, 22) — update `MOTOR_PINS` to match your wiring before running.

---

## Setup

```bash
sudo pigpiod
pip install -r ../requirements.txt
```

**Safety:** Always remove propellers before running any motor test. Motors will spin.
