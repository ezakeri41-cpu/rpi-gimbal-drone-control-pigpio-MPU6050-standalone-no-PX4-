#!/usr/bin/env python3
"""
Standalone roll-axis system identification -- no PX4, no MAVLink.

This script owns the whole chain itself:

    [tau_x, tau_y, tau_z, thrust]   (normalized: torques [-1,1], thrust [0,1])
              |  combine  (quad-X mixer geometry)
        motor_i                      (normalized fraction, [0,1])
              |  clamp, then scale
        pwm_i                        (microseconds, [1000, 2000])
              |  pigpio
        ESCs

Excitation: linear-sweep chirp on tau_x only (roll). tau_y and tau_z are
held at zero; thrust is held constant at BASE_THRUST_NORM.

Logs time, commanded tau_x, roll/pitch/yaw angles, roll rate, and the four
per-motor PWM values -- the PWM values are what THIS script computed and
sent, so there is no ambiguity about what the ESCs actually received.

SAFETY
------
Nothing else is in the loop. There is no PX4 failsafe, no rate controller,
no stabilization of any kind behind this script -- the angle/rate aborts
below are the ONLY automatic protection. Start with a small AMPLITUDE and
step up across runs.

The first thing this script does is arm the ESCs (900us for 5s). That
phase is deliberately excluded from the CSV.
"""

import time
import csv
import math
import atexit
import signal

import pigpio
from mpu6050 import mpu6050

# ============================== CONFIG ====================================

# --- excitation -----------------------------------------------------------
BASE_THRUST_NORM = 0.5   # constant normalized thrust, [0,1]
AMPLITUDE = 0.01         # chirp amplitude on tau_x, normalized [-1,1]
CHIRP_F0 = 0.3           # Hz, start frequency
CHIRP_F1 = 2.0           # Hz, end frequency
CHIRP_DURATION = 30.0    # seconds

LOOP_RATE_HZ = 50        # control/logging loop rate

# --- safety ---------------------------------------------------------------
ANGLE_LIMIT_DEG = 30.0     # abort if |roll| exceeds this
RATE_LIMIT_DEG_S = 150.0   # abort if |roll rate| exceeds this
ANGLE_LIMIT_RAD = math.radians(ANGLE_LIMIT_DEG)
RATE_LIMIT_RAD_S = math.radians(RATE_LIMIT_DEG_S)

# --- ESC arming -----------------------------------------------------------
ARM_PWM = 900
ARM_DURATION_S = 5.0

# --- GPIO (from your four_motor_bridge.py wiring) -------------------------
GPIO_M1 = 18
GPIO_M2 = 19
GPIO_M3 = 12
GPIO_M4 = 13
GPIO_MOTORS = [GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4]

PWM_MIN = 1000
PWM_MAX = 2000

# --- MPU6050 calibration (from your calibration + tilt runs) --------------
GYRO_BIAS = {'x': 5.148221, 'y': -3.911198, 'z': 1.039099}   # deg/s
ROLL_OFFSET_DEG = -2.555     # from mpu6050_tilt.py "y_tilt" (atan2(ay, az))
PITCH_OFFSET_DEG = 2.806     # from mpu6050_tilt.py "x_tilt" (atan2(ax, az))

ALPHA = 0.98   # complementary filter weight (gyro vs accel)

OUTPUT_CSV = 'standalone_chirp_roll_log.csv'

# --- mixer geometry -------------------------------------------------------
# PX4 v1.9.2 quad_x normalized allocation matrix, rows = motors 1..4,
# columns = [roll_scale, pitch_scale, yaw_scale, thrust_scale].
# Confirmed against mixer_multirotor.py / quad_x geometry in your source.
ROTORS = [
    {'roll': -0.707107, 'pitch':  0.707107, 'yaw':  1.0, 'thrust': 1.0},  # M1
    {'roll':  0.707107, 'pitch': -0.707107, 'yaw':  1.0, 'thrust': 1.0},  # M2
    {'roll':  0.707107, 'pitch':  0.707107, 'yaw': -1.0, 'thrust': 1.0},  # M3
    {'roll': -0.707107, 'pitch': -0.707107, 'yaw': -1.0, 'thrust': 1.0},  # M4
]

# ==========================================================================


def chirp_tau_x(t):
    """Linear-sweep chirp, CHIRP_F0 -> CHIRP_F1 over CHIRP_DURATION."""
    if t > CHIRP_DURATION:
        return 0.0
    k = (CHIRP_F1 - CHIRP_F0) / CHIRP_DURATION
    phase = 2.0 * math.pi * (CHIRP_F0 * t + 0.5 * k * t * t)
    return AMPLITUDE * math.sin(phase)


def mix(tau_x, tau_y, tau_z, thrust, t=None):
    """Combine the four normalized commands into four normalized per-motor
    values, then clamp each to [0,1].

    NOTE: this is a plain combine-and-clamp. It does NOT replicate PX4's
    airmode desaturation (which reduces thrust/roll/pitch/yaw in a specific
    order to preserve the shape of the demand before anything is cut). The
    two are equivalent ONLY while no motor value leaves [0,1] -- which is
    why any clamp event is reported loudly below rather than silently
    absorbed.
    """
    raw = []
    for r in ROTORS:
        raw.append(tau_x * r['roll'] +
                   tau_y * r['pitch'] +
                   tau_z * r['yaw'] +
                   thrust * r['thrust'])

    clamped = []
    for i, v in enumerate(raw):
        if v < 0.0 or v > 1.0:
            where = f" at t={t:.2f}s" if t is not None else ""
            print(f"  !! SATURATION{where}: motor {i + 1} raw={v:.4f} "
                  f"outside [0,1] -- clamped. Mixer output no longer matches "
                  f"the commanded torque/thrust; treat this sample as suspect.")
        clamped.append(max(0.0, min(1.0, v)))

    return clamped


def to_pwm(motor_norm):
    """Normalized motor fraction [0,1] -> PWM microseconds [1000,2000]."""
    return [int(round(PWM_MIN + m * (PWM_MAX - PWM_MIN))) for m in motor_norm]


# --------------------------- pigpio setup ---------------------------------
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Cannot connect to pigpiod (is the daemon running?)")


def cut_motors():
    """Stop pulsing the GPIO entirely. pigpio holds the last value forever
    otherwise -- including after this process exits."""
    for gpio in GPIO_MOTORS:
        pi.set_servo_pulsewidth(gpio, 0)
    print("Motor PWM output stopped (0us to all ESCs).")


atexit.register(cut_motors)


def _signal_handler(signum, frame):
    cut_motors()
    raise SystemExit(f"Terminated by signal {signum}")


signal.signal(signal.SIGTERM, _signal_handler)
signal.signal(signal.SIGHUP, _signal_handler)    # SSH session drop


# ------------------------- 1) ESC ARMING ----------------------------------
# First thing the script does. Nothing here is logged.
print(f"Arming ESCs: {ARM_PWM}us to all four motors for {ARM_DURATION_S}s...")
_arm_t0 = time.time()
while time.time() - _arm_t0 < ARM_DURATION_S:
    for gpio in GPIO_MOTORS:
        pi.set_servo_pulsewidth(gpio, ARM_PWM)
    time.sleep(0.02)
print("Arming complete.")


# ------------------------- MPU6050 ----------------------------------------
mpu = mpu6050(0x68)
angle_est = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}   # radians


def update_attitude(dt):
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()

    gx = gyro['x'] - GYRO_BIAS['x']
    gy = gyro['y'] - GYRO_BIAS['y']
    gz = gyro['z'] - GYRO_BIAS['z']

    # note the -accel['z']: this board reads az ~= -9.6 when level, so the
    # sign is flipped here to keep atan2 away from the +/-180 branch cut
    accel_roll = math.degrees(math.atan2(accel['y'], -accel['z'])) - ROLL_OFFSET_DEG
    accel_pitch = math.degrees(math.atan2(
        -accel['x'], math.sqrt(accel['y'] ** 2 + accel['z'] ** 2))) - PITCH_OFFSET_DEG

    roll_deg = ALPHA * (math.degrees(angle_est['roll']) + gx * dt) + (1 - ALPHA) * accel_roll
    pitch_deg = ALPHA * (math.degrees(angle_est['pitch']) + gy * dt) + (1 - ALPHA) * accel_pitch
    yaw_deg = math.degrees(angle_est['yaw']) + gz * dt   # gyro-only, drifts

    angle_est['roll'] = math.radians(roll_deg)
    angle_est['pitch'] = math.radians(pitch_deg)
    angle_est['yaw'] = math.radians(yaw_deg)

    rates = {'roll': math.radians(gx),
             'pitch': math.radians(gy),
             'yaw': math.radians(gz)}
    return angle_est, rates


# --------------------------- 2) MAIN LOOP ---------------------------------
log_rows = []
period = 1.0 / LOOP_RATE_HZ

print(f"Running chirp on tau_x: amplitude={AMPLITUDE}, "
      f"{CHIRP_F0}-{CHIRP_F1} Hz over {CHIRP_DURATION}s, "
      f"base thrust={BASE_THRUST_NORM}.")
print(f"Aborts: |roll| > {ANGLE_LIMIT_DEG} deg, "
      f"|roll rate| > {RATE_LIMIT_DEG_S} deg/s.")

try:
    # settle at symmetric base thrust briefly before exciting anything
    settle_pwms = to_pwm(mix(0.0, 0.0, 0.0, BASE_THRUST_NORM))
    print(f"Settling at base thrust, PWM={settle_pwms} for 2s...")
    _s0 = time.time()
    while time.time() - _s0 < 2.0:
        for gpio, pwm in zip(GPIO_MOTORS, settle_pwms):
            pi.set_servo_pulsewidth(gpio, pwm)
        time.sleep(period)

    t_start = time.time()
    last_t = t_start

    while True:
        now = time.time()
        dt = now - last_t
        last_t = now
        t = now - t_start

        angles, rates = update_attitude(dt)

        tau_x = chirp_tau_x(t)
        motor_norm = mix(tau_x, 0.0, 0.0, BASE_THRUST_NORM, t=t)
        pwms = to_pwm(motor_norm)

        for gpio, pwm in zip(GPIO_MOTORS, pwms):
            pi.set_servo_pulsewidth(gpio, pwm)

        log_rows.append((t, tau_x,
                         angles['roll'], angles['pitch'], angles['yaw'],
                         rates['roll'],
                         pwms[0], pwms[1], pwms[2], pwms[3]))

        if int(t / period) % 10 == 0:
            print(f"  t={t:6.2f}s  tau_x={tau_x:+.4f}  "
                  f"roll={math.degrees(angles['roll']):7.2f}deg  "
                  f"roll_rate={math.degrees(rates['roll']):7.2f}deg/s  "
                  f"pwm={pwms}")

        if abs(angles['roll']) > ANGLE_LIMIT_RAD:
            print(f"ABORT: roll exceeded {ANGLE_LIMIT_DEG} deg.")
            break

        if abs(rates['roll']) > RATE_LIMIT_RAD_S:
            print(f"ABORT: roll rate exceeded {RATE_LIMIT_DEG_S} deg/s.")
            break

        if t > CHIRP_DURATION:
            print("Chirp complete.")
            break

        time.sleep(max(0.0, period - (time.time() - now)))

except KeyboardInterrupt:
    print("\nAborted by user.")

finally:
    # ramp down to zero thrust rather than cutting abruptly
    print("Ramping thrust down...")
    ramp_steps = int(1.0 * LOOP_RATE_HZ)
    for i in range(ramp_steps):
        thr = BASE_THRUST_NORM * (1.0 - (i + 1) / ramp_steps)
        ramp_pwms = to_pwm(mix(0.0, 0.0, 0.0, thr))
        for gpio, pwm in zip(GPIO_MOTORS, ramp_pwms):
            pi.set_servo_pulsewidth(gpio, pwm)
        time.sleep(period)

    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'tau_x_cmd',
                         'roll_rad', 'pitch_rad', 'yaw_rad',
                         'rollspeed_rad_s',
                         'pwm1', 'pwm2', 'pwm3', 'pwm4'])
        writer.writerows(log_rows)
    print(f"Saved {len(log_rows)} rows to {OUTPUT_CSV}")
    # cut_motors() runs automatically on exit via atexit
