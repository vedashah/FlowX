import time, csv
import board, busio
from digitalio import Direction
from adafruit_mcp230xx.mcp23017 import MCP23017
from hx711 import HX711

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ------------------ Valve ------------------
i2c = busio.I2C(board.SCL, board.SDA)
mcp = MCP23017(i2c)
valve1 = mcp.get_pin(0)
valve1.direction = Direction.OUTPUT

# Your logic: False = OFF, True = ON
def valve_on():  valve1.value = True
def valve_off(): valve1.value = False
valve_off()

# ------------------ HX711 ------------------
DT, SCK = 5, 6
hx = HX711(DT, SCK)

def read_avg(n=10, delay=0.002):
    samples = []
    for _ in range(n):
        v = hx.get_raw_data()
        if v is None:
            pass
        elif isinstance(v, (list, tuple)):
            for x in v:
                if isinstance(x, (int, float)):
                    samples.append(x)
        elif isinstance(v, (int, float)):
            samples.append(v)
        time.sleep(delay)
    if not samples:
        raise RuntimeError("No HX711 samples. Check wiring/power/pins.")
    return sum(samples) / len(samples)

def ema(prev, x, alpha=0.2):
    return x if prev is None else (alpha * x + (1 - alpha) * prev)

# ------------------ Tare once ------------------
input("Remove all weight / no thrust. Press Enter to tare...")
hx.reset()
time.sleep(0.3)
time.sleep(0.5)
offset = read_avg(80)
print("Offset:", offset)

# ------------------ Test config ------------------
test_name = input("Enter test name: ").strip() or "test"
ts = int(time.time())
csv_path = f"{test_name}_{ts}.csv"
png_path = f"{test_name}_{ts}.png"
print("Logging to:", csv_path)

ON_TIME  = 5.0
OFF_TIME = 5.0

# store for plot
t_series, net_series, net_ema_series, valve_series = [], [], [], []
f_ema = None

start = time.perf_counter()

def now():
    return time.perf_counter() - start

print("Running... Ctrl+C to stop.")
print("Cycle: ON 5s -> OFF 5s (tare only once at start)")

cycle = 0

try:
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "net_counts", "net_counts_ema", "valve_on"])

        while True:
            cycle += 1

            # ----- ON window metrics -----
            on_peak = float("-inf")
            on_sum = 0.0
            on_impulse = 0.0
            on_n = 0
            prev_t = None
            prev_net = None

            valve_on()
            print(f"[Cycle {cycle}] VALVE: ON")
            t_on_start = time.perf_counter()

            while (time.perf_counter() - t_on_start) < ON_TIME:
                t = now()
                raw = read_avg(5)
                net = raw - offset
                f_ema = ema(f_ema, net, alpha=0.2)

                w.writerow([t, net, f_ema, 1])

                t_series.append(t)
                net_series.append(net)
                net_ema_series.append(f_ema)
                valve_series.append(1)

                # metrics
                on_peak = max(on_peak, net)
                on_sum += net
                on_n += 1

                # impulse (trapezoid)
                if prev_t is not None:
                    dt = t - prev_t
                    on_impulse += 0.5 * (prev_net + net) * dt
                prev_t, prev_net = t, net

                time.sleep(0.02)

            valve_off()
            on_avg = on_sum / max(on_n, 1)
            print(f"[Cycle {cycle}] Peak={on_peak:.1f} counts | Avg={on_avg:.1f} counts | Impulse={on_impulse:.1f} count*s")

            # ----- OFF -----
            print(f"[Cycle {cycle}] VALVE: OFF")
            t_off_start = time.perf_counter()
            while (time.perf_counter() - t_off_start) < OFF_TIME:
                t = now()
                raw = read_avg(5)
                net = raw - offset
                f_ema = ema(f_ema, net, alpha=0.2)

                w.writerow([t, net, f_ema, 0])

                t_series.append(t)
                net_series.append(net)
                net_ema_series.append(f_ema)
                valve_series.append(0)

                time.sleep(0.02)

except KeyboardInterrupt:
    pass
finally:
    valve_off()
    try:
        hx.power_down()
    except Exception:
        pass

# ------------------ Plot ------------------
if len(t_series) >= 2:
    plt.figure()
    plt.plot(t_series, net_ema_series, label="net_counts_ema (smoothed)")
    plt.plot(t_series, net_series, alpha=0.3, label="net_counts")
    plt.xlabel("Time (s)")
    plt.ylabel("Relative thrust (counts)")
    plt.title(test_name)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(png_path, dpi=200)
    print("Saved plot:", png_path)

print("Saved CSV:", csv_path)
