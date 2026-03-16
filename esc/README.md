# ESC Calibration
### *FlowX – Electronic Speed Controller Setup*

The `esc/` directory contains the ESC calibration utility for the FlowX test bench. Calibration must be performed once when setting up new ESCs, or any time the PWM range needs to be re-established.

---

## Scripts

| File | Description |
|------|-------------|
| `esc_calibration.py` | Walks through the full ESC calibration sequence (max → min throttle) |

---

## Calibration Procedure

**Before you begin:** Remove all propellers. Motors will arm at the end of calibration.

1. Run the script:
   ```bash
   sudo pigpiod
   python3 esc_calibration.py
   ```
2. When prompted, the script sends **2000 µs (max throttle)** to all ESCs.
3. Power the ESCs **while the script is holding max throttle**.
4. Wait for the ESC calibration beep sequence, then press `Enter`.
5. The script sends **1000 µs (min throttle)** and waits 5 seconds for ESCs to confirm.
6. ESCs are now calibrated for the 1000–2000 µs range.

---

## GPIO Pin Mapping

| Motor | GPIO Pin |
|-------|----------|
| M1 (Front-Left) | 27 |
| M2 (Front-Right) | 26 |
| M3 (Rear-Right) | 23 |
| M4 (Rear-Left) | 16 |

These pins match the configuration in `hybrid_rate_controller.py`.

---

## Dependencies

```bash
pip install pigpio
sudo pigpiod
```
