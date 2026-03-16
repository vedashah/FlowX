# Thruster Logic
### *FlowX – Cold-Gas Thruster Mixing & RC Actuation Control*

The `thruster_logic/` directory contains the core control scripts for mapping RC receiver inputs to cold-gas thruster valve commands via the MCP23017 MOSFET driver board.

---

## Scripts

| File | Description |
|------|-------------|
| `rc_thruster_controller.py` | **Primary thruster controller.** Full SBUS → 8-valve MCP23017 pipeline with pulse-density scheduling and hardware safety limits. |
| `rc_valve_mixer_stub.py` | Simplified 6-channel valve mixer. Mixing math is complete; `apply_tick()` is a **placeholder** — replace with MCP23017 calls before hardware use. |
| `individual_thruster_fire.py` | Fires each of the 8 valves in sequence (one at a time, then stacked). Used to verify valve wiring and flow continuity. |
| `keyboard_roll_pitch.py` | Keyboard-controlled valve actuation (`1` = roll, `2` = pitch, `0` = all off). Bench test without an RC transmitter. |

---

## `rc_thruster_controller.py` — Primary Controller

This is the production-ready thruster control script. It implements the full RC → valve pipeline:

**Input:** SBUS serial stream on `/dev/serial0` at 420 000 baud

**Processing:**
1. Decodes SBUS frame and normalizes axes (deadband ±20 raw counts)
2. Runs a Quad-X mixing law to compute per-valve duty cycles
3. **Pulse Density Scheduler** — accumulator-based scheduler over a 5-tick window converts continuous duty commands into discrete open/close decisions each tick
4. **Hardware safety cap** — maximum 4 valves open simultaneously (`MAX_VALVES_ON = 4`)

**Output:** MCP23017 I²C GPIO expander at address `0x20` — 8 solenoid valves via MOSFET board

**Arm sequence:** throttle low + yaw high + arm switch low

**Key config constants:**
```python
ROLL_GAIN     = 0.4    # Roll authority (0.0–1.0)
PITCH_GAIN    = 0.4    # Pitch authority (0.0–1.0)
TICK_S        = 0.2    # Valve pulse window (seconds)
DEADBAND      = 20     # Stick deadband (raw SBUS counts)
MAX_VALVES_ON = 4      # Hardware safety limit
MCP_ADDR      = 0x20   # I²C address of MCP23017
```

**Run:**
```bash
python3 rc_thruster_controller.py
```

---

## Valve Layout (MCP23017 Pin Mapping)

| MCP23017 Pin | Valve | Function |
|---|---|---|
| GPA0 | FL | Front-Left thruster |
| GPA1 | FR | Front-Right thruster |
| GPA2 | RR | Rear-Right thruster |
| GPA3 | RL | Rear-Left thruster |
| GPA4 | YAW_CW1 | CW yaw thruster A |
| GPA5 | YAW_CW2 | CW yaw thruster B |
| GPA6 | YAW_CCW1 | CCW yaw thruster A |
| GPA7 | YAW_CCW2 | CCW yaw thruster B |

---

## Thruster Mixer (Quad-X Convention)

```
       Front
  FL ──────── FR
   \          /
    \  [Body] /
   RL ──────── RR
       Rear
```

| Thruster | Throttle | Pitch | Roll | Yaw |
|----------|----------|-------|------|-----|
| FL | + | + | + | — |
| FR | + | + | − | — |
| RR | + | − | − | — |
| RL | + | − | + | — |
| YAW_CW (×2) | — | — | — | + |
| YAW_CCW (×2) | — | — | — | − |

CW and CCW yaw channels are mutually exclusive — only one direction fires per tick.

---

## Dependencies

```bash
pip install -r ../requirements.txt
```

Ensure SBUS receiver is on `/dev/serial0` and UART is enabled in `/boot/config.txt`.
