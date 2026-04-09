# Relay Tests
### *FlowX – Valve Actuation & Thrust Stand Hardware Tests*

The `relay_tests/` directory contains scripts for testing the solenoid valve relay board and the thrust measurement stand. These scripts validate the full hardware path from Raspberry Pi I²C commands through to physical valve actuation and force measurement.

---

## Scripts

| File | Description |
|------|-------------|
| `thrust_characterization.py` | Opens a valve for a timed pulse, records HX711 load cell readings, then saves a thrust curve CSV + PNG to `../nozzle_test_results/` |
| `valve_connection.py` | Cycles two valve solenoids ON/OFF every 2 seconds via MCP23017. Confirms I²C wiring, address, and MOSFET driver response. |

---

## `thrust_characterization.py` — Details

Full thrust characterization routine:
1. Opens valve 1 for a configurable pulse duration
2. Samples the HX711 load cell at high rate during the pulse
3. Computes tare offset, net force (ADC counts → physical units with calibration factor), and EMA-smoothed curve
4. Saves results as a timestamped CSV and PNG thrust curve to `../nozzle_test_results/`

**Run:**
```bash
python3 thrust_characterization.py
```

---

## `valve_connection.py` — Details

A simple looping test that toggles two valves every 2 seconds.

- **Valve 1:** MCP23017 pin GPA0
- **Valve 2:** MCP23017 pin GPA7

Use this first when setting up new hardware to confirm the I²C address (`0x20`), wiring continuity, and that MOSFET drivers are switching correctly.

**Run:**
```bash
python3 valve_connection.py
```
Terminate with `Ctrl+C`.

---

## Hardware Connections

| Component | Interface |
|-----------|-----------|
| MCP23017 GPIO expander | I²C (SDA/SCL, address `0x20`) |
| Solenoid valves | MCP23017 output pins → MOSFET driver board |
| HX711 load cell amplifier | GPIO 5 (DT), GPIO 6 (SCK) |

---

## Dependencies

```bash
pip install -r ../requirements.txt
```
