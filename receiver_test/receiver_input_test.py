import pigpio
import time

PPM_PIN = 17
CHANNELS = 8
SYNC_GAP_US = 3000  # frame sync gap

pi = pigpio.pi()
if not pi.connected:
    print("❌ pigpio not running")
    exit(1)

channels = [0] * CHANNELS
last_tick = 0
chan_index = 0

def ppm_callback(gpio, level, tick):
    global last_tick, chan_index

    if level != 1:
        return

    pulse = pigpio.tickDiff(last_tick, tick)
    last_tick = tick

    if pulse > SYNC_GAP_US:
        chan_index = 0
    elif chan_index < CHANNELS:
        channels[chan_index] = pulse
        chan_index += 1

cb = pi.callback(PPM_PIN, pigpio.RISING_EDGE, ppm_callback)

print("✅ Listening for PPM on GPIO 17...")
print("Move your sticks. Press CTRL+C to exit.\n")

try:
    while True:
        print("Channels:", channels)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    cb.cancel()
    pi.stop()