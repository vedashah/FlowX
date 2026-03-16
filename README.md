# FlowX – Cold-Gas Thruster Reconnaissance Drone
### TAMU Senior Capstone Project

FlowX is a proof-of-concept **cold-gas propelled reconnaissance drone** designed for low-gravity lunar environments. The system targets future **Permanently Shadowed Region (PSR)** exploration missions by demonstrating high-mobility, propellant-efficient flight using **CO₂/N₂ cold-gas thrusters** controlled by a Raspberry Pi embedded system.

---

## Project Overview

FlowX is developed across two semesters as a senior capstone with the following goals:

- Build a functional **cold-gas thruster test platform** with real-time valve actuation
- Measure **impulse, thrust curves, and valve response times** across nozzle geometries
- Develop an onboard **embedded control system** using Raspberry Pi + custom MOSFET driver boards
- Implement a **hybrid rate controller** reading gyro data over MSP from a Betaflight FC
- Demonstrate **controlled 6-DOF maneuvering** for lunar-like low-gravity conditions
- Provide a modular foundation for future **PSR mapping missions**

---

## Repository Structure

```
FlowX/
├── requirements.txt                      # All Python dependencies
│
├── flight_control/                       # Closed-loop flight control system
│   └── hybrid_rate_controller.py         # 200 Hz PID rate controller (SBUS + MSP → ESC)
│
├── fc_tests/                             # Flight controller interface validation
│   └── pi_fc_connection_test.py          # MSP connection test — prints live FC attitude
│
├── esc/                                  # ESC calibration utilities
│   └── esc_calibration.py
│
├── logs/                                 # Auto-generated test rig CSV logs
│
├── motor_tests/                          # Motor, ESC, PWM, and IMU validation scripts
│   ├── test_pwm_4_motors.py              # 4-motor PWM ramp test
│   ├── esc_full_ramp_demo.py             # 0→100% ramp for demos/reviews
│   ├── single_motor_test.py              # Single motor bench test (gpiozero)
│   ├── motor_matrix_test.py              # Interactive motor-by-motor spin test
│   ├── sbus_quad_mixer.py                # SBUS → Quad-X roll/pitch/yaw mixer
│   ├── sbus_roll_pitch_mixer.py          # SBUS → roll/pitch only (baseline)
│   ├── sbus_channel_scanner.py           # Prints all 8 raw SBUS channel values
│   ├── ppm_roll_pitch_mixer.py           # PPM → roll/pitch mixer with failsafe
│   ├── ppm_throttle_passthrough.py       # PPM throttle → all 4 motors passthrough
│   ├── IMU_data_display.py               # Live MPU6050 accel + gyro display (I²C)
│   └── smbus2_import_check.py            # smbus2 library sanity check
│
├── nozzle_test_results/                  # Thrust characterization data (CSV + plots)
│
├── receiver_test/                        # RC receiver and gyro input validation
│   ├── receiver_input_test.py            # PPM decoder — prints live channel values
│   ├── gyro_calibration.py               # MPU6050 bias calibration (500 samples)
│   └── gyro_tests.py                     # ⚠️ Empty placeholder
│
├── relay_tests/                          # Valve relay and thrust stand hardware tests
│   ├── thrust_characterization.py        # Timed valve pulse + HX711 data capture
│   └── valve_connection.py               # Cycles two valves ON/OFF to verify wiring
│
└── thruster_logic/                       # Cold-gas thruster mixing and RC actuation
    ├── rc_thruster_controller.py         # Primary 8-valve SBUS controller (MCP23017)
    ├── rc_valve_mixer_stub.py            # Simplified mixer — apply_tick() is placeholder
    ├── individual_thruster_fire.py       # Fires each valve in sequence for flow testing
    └── keyboard_roll_pitch.py            # Keyboard valve control for bench testing
```

---

## Recommended Test Sequence

Before running the full flight controller, work through this validation sequence:

```
1. esc/esc_calibration.py               — Calibrate ESCs once on new hardware
2. receiver_test/receiver_input_test.py — Confirm RC receiver is decoded correctly
3. motor_tests/sbus_channel_scanner.py  — Identify channel assignments on your TX
4. motor_tests/motor_matrix_test.py     — Verify each motor spins in the right direction
5. fc_tests/pi_fc_connection_test.py    — Confirm MSP link to Betaflight FC is alive
6. flight_control/hybrid_rate_controller.py — Run the closed-loop controller
```

---

## Hardware Stack

| Component | Part |
|-----------|------|
| Compute | Raspberry Pi 4 |
| Flight Controller | Betaflight F722 (IMU source via MSP) |
| RC Receiver | SBUS-output receiver (`/dev/serial0`, 420 000 baud) |
| ESCs | Standard PWM ESCs (1000–2000 µs) on GPIO 27, 26, 23, 16 |
| Valve Driver | MCP23017 I²C GPIO expander + MOSFET board (addr `0x20`) |
| IMU (standalone) | MPU6050 on I²C bus 1, addr `0x68` |
| Thrust Sensing | HX711 load cell amplifier (DT → GPIO 5, SCK → GPIO 6) |
| Propellant | CO₂ / N₂ cold gas |

---

## Setup & Dependencies

### System Requirements
- Raspberry Pi OS (64-bit recommended)
- Python 3.9+
- `pigpiod` daemon running (`sudo pigpiod`)

### Enable UART
Add to `/boot/config.txt`:
```
enable_uart=1
dtoverlay=disable-bt
```

### Install Python Dependencies
```bash
pip install -r requirements.txt
```

---

## Subdirectory READMEs

- [`flight_control/README.md`](flight_control/README.md) – PID controller details, gains, motor map
- [`fc_tests/README.md`](fc_tests/README.md) – MSP connection test and protocol reference
- [`esc/README.md`](esc/README.md) – ESC calibration procedure and GPIO pin map
- [`logs/README.md`](logs/README.md) – Log file format and full column reference
- [`motor_tests/README.md`](motor_tests/README.md) – Motor, ESC, PWM, and IMU test suite
- [`nozzle_test_results/README.md`](nozzle_test_results/README.md) – Nozzle geometry test data
- [`receiver_test/README.md`](receiver_test/README.md) – RC receiver and gyro validation
- [`relay_tests/README.md`](relay_tests/README.md) – Valve and thrust stand hardware tests
- [`thruster_logic/README.md`](thruster_logic/README.md) – Thruster mixing and RC control logic

---

## Data & Logs

Runtime telemetry logs are written to `logs/` as `rig_log_YYYYMMDD_HHMMSS.csv`.
Nozzle thrust data (CSV + PNG plots) lives in `nozzle_test_results/`.

> **Note:** `logs/*.csv` is excluded from Git via `.gitignore`. Only commit intentional test result snapshots.

---

## Team

TAMU FlowX
