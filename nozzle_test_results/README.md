# Nozzle Test Results
### *FlowX – Cold-Gas Nozzle Thrust Characterization Data*

This directory stores all thrust characterization data collected on the FlowX test stand. Each test run produces a paired CSV (raw data) and PNG (thrust curve plot). Tests compare different nozzle geometries to identify optimal designs for the final propulsion system.

---

## Nozzle Configurations Tested

| File Prefix | Nozzle | Geometry |
|-------------|--------|----------|
| `Nozzle_A_Straight` | Nozzle A | Straight bore |
| `Nozzle_B_Straight` | Nozzle B | Straight bore |
| `Nozzle_B_Curved_Cone` | Nozzle B | Curved convergent cone |
| `Nozzle_C_Straight` | Nozzle C | Straight bore |
| `Nozzle_D_Straight` | Nozzle D | Straight bore |
| `c_*` | Configuration C | Re-test / variant run |

The trailing number in each filename is a Unix timestamp (seconds) marking when the test was run.

---

## CSV Column Reference

| Column | Description |
|--------|-------------|
| `t_s` | Elapsed time since test start (seconds) |
| `net_counts` | Raw HX711 load cell reading minus tare offset (ADC counts) |
| `net_counts_ema` | Exponential moving average of `net_counts` (smoothed) |
| `valve_on` | Valve state during sample (`1` = open, `0` = closed) |

To convert `net_counts` to force (Newtons), apply your load cell calibration factor:
```
Force (N) = net_counts / calibration_factor
```

---

## PNG Plots

Each `.png` file is a thrust curve plot generated automatically by `relay_tests/Thrust_test.py` at the end of each test run. Plots show:
- Raw and smoothed (EMA) thrust vs. time
- Valve-open period highlighted
- Peak thrust and estimated total impulse annotated

---

## Notes

- All tests were conducted with CO₂ propellant at bench supply pressure.
- Nozzle geometries were 3D printed. Dimensions are tracked in the project CAD files.
- The `c_1770938351` run is an unlabeled configuration C variant — update the filename when the geometry is confirmed.
- For the test script that generates these files, see [`../relay_tests/Thrust_test.py`](../relay_tests/Thrust_test.py).
