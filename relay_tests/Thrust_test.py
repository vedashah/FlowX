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

def _collect_samples(n=10, delay=0.002):
    """Collect numeric samples from hx.get_raw_data() across different hx711 libs."""
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
    return samples

def read_avg(n=10, delay=0.002):
    s = _collect_samples(n=n, delay=delay)
    if not s:
        raise RuntimeError("No HX711 samples. Check wiring/power/pins.")
    return sum(s) / len(s)

def read_med(n=9, delay=0.002):
    """Median read: very good at killing one-off spikes."""
    s = _collect_samples(n=n, delay=delay)
    if not s:
        raise RuntimeError("No HX711 samples. Check wiring/power/pins.")
    s.sort()
    return s[len(s)//2]

def ema(prev, x, alpha=0.2):
    return x if prev is None else (alpha * x + (1 - alpha) * prev)

# ------------------ Tare once ------------------
input("Remove all weight / no thrust. Press Enter to tare...")
hx.reset()
time.sleep(0.3)
time.sleep(0.5)
offset = read_med(21, delay=0.001)
print("Offset:", offset)

# ------------------ Test config ------------------
test_name = input("Enter test name: ").strip() or "test"
ts = int(time.time())
csv_path = f"{test_name}_{ts}.csv"
png_path = f"{test_name}_{ts}.png"
print("Logging to:", csv_path)

ON_TIME  = 5
OFF_TIME = 15

# Spike rejection threshold (tune if needed)
# Your real values are ~0-15k, so 40k is a safe cutoff.
SPIKE_ABS_LIMIT = 40000

# store for plot
t_series, net_series, net_ema_series, valve_series = [], [], [], []
f_ema = None

start = time.perf_counter()
def now():
    return time.perf_counter() - start

print("Running... Ctrl+C to stop.")
print("Cycle: ON 5s -> OFF 15s (tare only once at start)")

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

                # Median read to kill outliers
                raw = read_med(9)
                net = raw - offset

                # Extra spike reject
                if abs(net) > SPIKE_ABS_LIMIT:
                    continue

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

                raw = read_med(9)
                net = raw - offset

                if abs(net) > SPIKE_ABS_LIMIT:
                    continue

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
    plt.figure(figsize=(10, 6))

    plt.plot(t_series, net_ema_series, label="net_counts_ema (smoothed)")
    plt.plot(t_series, net_series, alpha=0.3, label="net_counts")

    # ---- Stats ONLY when valve is ON ----
    y_on = [y for y, v in zip(net_series, valve_series) if v == 1]

    if y_on:
        y_min = min(y_on)
        y_max = max(y_on)
        y_avg = sum(y_on) / len(y_on)

        # Horizontal lines for ON-only min/max/avg
        plt.axhline(y_avg, linestyle="--", linewidth=1, label=f"ON avg = {y_avg:.1f}")
        plt.axhline(y_max, linestyle="--", linewidth=1, label=f"ON max = {y_max:.1f}")
        plt.axhline(y_min, linestyle="--", linewidth=1, label=f"ON min = {y_min:.1f}")

        # Stats box
        stats_text = f"ON min: {y_min:.1f}\nON avg: {y_avg:.1f}\nON max: {y_max:.1f}"
        plt.gca().text(
            0.02, 0.98, stats_text,
            transform=plt.gca().transAxes,
            va="top", ha="left",
            bbox=dict(boxstyle="round", alpha=0.8)
        )
    else:
        plt.gca().text(
            0.02, 0.98, "No valve-ON samples found",
            transform=plt.gca().transAxes,
            va="top", ha="left",
            bbox=dict(boxstyle="round", alpha=0.8)
        )

    plt.xlabel("Time (s)")
    plt.ylabel("Relative thrust (counts)")
    plt.title(test_name)
    plt.grid(True)

    plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))
    plt.tight_layout()
    plt.savefig(png_path, dpi=200, bbox_inches="tight")
    print("Saved plot:", png_path)
else:
    print("Not enough data to plot.")



print("Saved CSV:", csv_path)
