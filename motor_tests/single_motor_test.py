from gpiozero import PWMOutputDevice
from time import sleep

esc = PWMOutputDevice(18, frequency=50)  # GPIO 18, 50 Hz

# Arm ESC
esc.value = 0.05   # ~1ms pulse (minimum throttle)
sleep(3)

# Turn motor ON
esc.value = 0.10   # ~1.5ms pulse (mid throttle)
sleep(3)

# Turn motor OFF
esc.value = 0.05
sleep(1)
esc.close()
