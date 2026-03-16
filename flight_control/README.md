# Flight Control
### *FlowX – Closed-Loop Rate Controller*

The `flight_control/` directory contains the primary flight control system for the FlowX test bench. This is the main script run during active testing sessions.

---

## Scripts

| File | Description |
|------|-------------|
| `hybrid_rate_controller.py` | 200 Hz closed-loop PID rate controller — reads RC input via SBUS and gyro data via MSP, outputs PWM to four ESCs |

---

## `hybrid_rate_controller.py` — Details

A **hybrid** controller in the sense that it splits responsibilities across two devices:
- **Raspberry Pi** handles RC decoding, PID computation, and motor output
- **Betaflight F722 FC** provides IMU data (gyroscope) over MSP — acting purely as a sensor, not a flight controller

**Control loop (200 Hz):**
1. Read SBUS frame from RC receiver (`/dev/serial0`, 420 000 baud)
2. Request raw IMU data from Betaflight FC (`/dev/ttyACM0`, MSP `RAW_IMU` command)
3. Compute roll, pitch, yaw rate errors (setpoint from sticks − measured gyro rate)
4. Run three independent PID loops
5. Mix PID outputs into per-motor PWM commands (Quad-X mixer)
6. Write PWM to four ESCs via `pigpio`
7. Log all telemetry to a timestamped CSV in `../logs/`

**Arm sequence:** throttle low + arm switch low → `ARMED`
**Disarm:** arm switch high, or IMU timeout (> 100 ms without valid MSP response)

---

## PID Configuration

Gains are set in the `PID` dict near the top of the file:

```python
PID = {
    "roll":  {"kp": 1.0, "ki": 0.0, "kd": 0.0, "i_lim": 200.0},
    "pitch": {"kp": 1.0, "ki": 0.0, "kd": 0.0, "i_lim": 200.0},
    "yaw":   {"kp": 0.8, "ki": 0.0, "kd": 0.0, "i_lim": 200.0},
}
```

Currently P-only. Tune `ki` and `kd` after verifying basic P response on the rig.

**Rate limits (deg/s):**
```python
MAX_RATE_DPS_ROLL  = 120.0
MAX_RATE_DPS_PITCH = 120.0
MAX_RATE_DPS_YAW   = 90.0
```

**Stick deadband:** `STICK_DEADBAND_US = 70` µs either side of center (1500 µs)

---

## Motor GPIO Pin Mapping

| Motor | Position | GPIO (BCM) |
|-------|----------|------------|
| M1 | Front-Left | 27 |
| M2 | Front-Right | 26 |
| M3 | Rear-Right | 23 |
| M4 | Rear-Left | 16 |

---

## Run

```bash
sudo pigpiod
python3 hybrid_rate_controller.py
```

Logs are written to `../logs/rig_log_YYYYMMDD_HHMMSS.csv`. See [`../logs/README.md`](../logs/README.md) for the full column reference.

Before running, verify the FC MSP link is working:
```bash
python3 ../fc_tests/pi_fc_connection_test.py
```

---

## Dependencies

```bash
pip install -r ../requirements.txt
sudo pigpiod
```
