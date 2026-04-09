# FC Tests
### *FlowX – Flight Controller Interface Validation*

The `fc_tests/` directory contains scripts for validating the MSP (Multiwii Serial Protocol) interface between the Raspberry Pi and the Betaflight flight controller. Run these before starting `flight_control/hybrid_rate_controller.py` to confirm the FC link is healthy.

---

## Scripts

| File | Description |
|------|-------------|
| `pi_fc_connection_test.py` | Sends MSP `ATTITUDE` requests to the Betaflight FC and prints live roll/pitch/yaw values to the terminal |

---

## `pi_fc_connection_test.py` — Details

Connects to the Betaflight FC over USB serial (`/dev/ttyACM0`, 115 200 baud) and polls `MSP_ATTITUDE` (command 108) in a loop.

**What it validates:**
- USB serial connection to the FC is alive
- Betaflight is responding to MSP requests
- MSP frame parsing (header `$M>`, checksum, payload) is working correctly
- Attitude values (roll/pitch in 0.1° units, yaw in 1° units) are updating and look sane

**Expected output:**
```
Roll:  -0.3°   Pitch:  1.2°   Yaw:  247.0°
Roll:  -0.3°   Pitch:  1.2°   Yaw:  247.1°
...
```

If the script hangs or prints nothing, check:
1. FC is powered and connected via USB
2. Correct port (`/dev/ttyACM0`) — use `ls /dev/ttyACM*` to confirm
3. Betaflight has MSP enabled on the USB port (Ports tab in Betaflight Configurator)

---

## Run

```bash
python3 pi_fc_connection_test.py
```

No `pigpiod` required — this script only uses serial, not GPIO.

---

## MSP Protocol Reference

| Field | Value |
|-------|-------|
| Header | `$M<` (request) / `$M>` (response) |
| Command | `108` (`MSP_ATTITUDE`) |
| Payload | 6 bytes: `int16` roll (×10), `int16` pitch (×10), `int16` yaw |
| Checksum | XOR of length, command, and all payload bytes |

The same MSP parsing logic used here (`msp_request`, `read_msp_response`) is replicated in `flight_control/hybrid_rate_controller.py` for the `RAW_IMU` command.

---

## Dependencies

```bash
pip install pyserial
```
