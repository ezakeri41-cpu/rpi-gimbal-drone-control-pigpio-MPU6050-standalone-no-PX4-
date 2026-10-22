import pigpio
import time

# GPIO pin assignments -- CONFIRM these against your actual wiring.
# ESC1/ESC2 match your existing pwm_test.py (GPIO18, GPIO19).
# ESC3/ESC4 are placeholders -- update to your real GPIO pins.
ESC1 = 18
ESC2 = 19
ESC3 = 12
ESC4 = 13

ESCS = [ESC1, ESC2, ESC3, ESC4]

pi = pigpio.pi()

if not pi.connected:
    print("pigpio connection failed")
    exit()

def set_all(pulsewidth):
    for esc in ESCS:
        pi.set_servo_pulsewidth(esc, pulsewidth)

print("sending 900us")
set_all(900)
time.sleep(3)

print("sending 1000us")
set_all(1000)
time.sleep(10)

print("stopping")
set_all(0)
pi.stop()
