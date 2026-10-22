#!/usr/bin/env python3
"""
OPEN-LOOP constant torque test. No controller, no feedback -- just applies
a fixed tau_x plus base thrust and watches what happens.

Purpose: your identified roll model predicts a specific steady-state angle
for a given constant torque (DC gain = b/a0 = 24.8/25.89 = 0.958 rad/unit
= 54.9 deg/unit). This script lets you check that prediction directly on
hardware, with no PID/outer-loop logic in the way to obscure the answer.

    tau_x = TAU_X_CONST -> mixer -> motor_i (clamped [0,1]) -> PWM -> ESCs

For tau_x = 0.1 at thrust = 0.2, the model predicts a steady roll of
about 0.1 * 54.9 = 5.49 deg, with motor commands well inside [0,1]
(M1/M4 ~0.129, M2/M3 ~0.271 -- see the printed values at startup).

SAFETY
------
This is still fully open-loop and unopposed by anything but the angle/rate
aborts below. Nothing stops the vehicle from drifting if the real DC gain
or the rig's resting trim differs from the model's prediction -- watch the
live printout and be ready to Ctrl+C.

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

BASE_THRUST_NORM = 0.2     # same operating point the model was identified at
TAU_X_CONST = 0.1          # constant commanded roll torque, normalized [-1,1]
RUN_DURATION = 15.0        # seconds

LOOP_RATE_HZ = 100

# --- safety ---------------------------------------------------------------
ANGLE_LIMIT_DEG = 35.0
RATE_LIMIT_DEG_S = 250.0
ANGLE_LIMIT_RAD = math.radians(ANGLE_LIMIT_DEG)
RATE_LIMIT_ABORT_RAD_S = math.radians(RATE_LIMIT_DEG_S)

# --- ESC arming -----------------------------------------------------------
ARM_PWM = 900
ARM_DURATION_S = 5.0

# --- hardware ---------------------------------------------------------------
GPIO_MOTORS = [18, 19, 12, 13]         # M1, M2, M3, M4
PWM_MIN, PWM_MAX = 1000, 2000

# --- MPU6050 calibration ----------------------------------------------------
GYRO_BIAS = {'x': 5.200038, 'y': -3.915649, 'z': 1.139756}   # deg/s
ROLL_OFFSET_DEG = 4.841
PITCH_OFFSET_DEG = -0.816   # see the sign note in roll_cascade_control.py --
                             # this is the negative of the tilt script's
                             # x_tilt, because the formula below negates ax

ALPHA = 0.98

OUTPUT_CSV = 'open_loop_tau_x_log.csv'

# --- mixer ------------------------------------------------------------------
ROTORS = [
    {'roll': -0.707107, 'pitch':  0.707107, 'yaw':  1.0, 'thrust': 1.0},  # M1
    {'roll':  0.707107, 'pitch': -0.707107, 'yaw':  1.0, 'thrust': 1.0},  # M2
    {'roll':  0.707107, 'pitch':  0.707107, 'yaw': -1.0, 'thrust': 1.0},  # M3
    {'roll': -0.707107, 'pitch': -0.707107, 'yaw': -1.0, 'thrust': 1.0},  # M4
]

# ==========================================================================

DT = 1.0 / LOOP_RATE_HZ


def mix(tau_x, tau_y, tau_z, thrust):
    raw = [tau_x * r['roll'] + tau_y * r['pitch'] + tau_z * r['yaw']
           + thrust * r['thrust'] for r in ROTORS]
    out = [max(0.0, min(1.0, v)) for v in raw]
    return out, any(abs(a - b) > 1e-9 for a, b in zip(raw, out))


def to_pwm(motors):
    return [int(round(PWM_MIN + m * (PWM_MAX - PWM_MIN))) for m in motors]


# --------------------------- pigpio ---------------------------------------
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpiod is not running")


def cut_motors():
    for g in GPIO_MOTORS:
        pi.set_servo_pulsewidth(g, 0)
    print("Motor PWM stopped (0 us).")


atexit.register(cut_motors)


def _sig(signum, frame):
    cut_motors()
    raise SystemExit(f"terminated by signal {signum}")


signal.signal(signal.SIGTERM, _sig)
signal.signal(signal.SIGHUP, _sig)


# ------------------------- 1) ARM THE ESCs --------------------------------
print(f"Arming: {ARM_PWM} us to all four ESCs for {ARM_DURATION_S:.0f}s...")
_t0 = time.time()
while time.time() - _t0 < ARM_DURATION_S:
    for g in GPIO_MOTORS:
        pi.set_servo_pulsewidth(g, ARM_PWM)
    time.sleep(0.02)
print("Armed.")


# ------------------------- MPU6050 ------------------------------------------
mpu = mpu6050(0x68)
ang = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}


def read_accel_angles():
    a = mpu.get_accel_data()
    r = math.degrees(math.atan2(a['y'], -a['z'])) - ROLL_OFFSET_DEG
    p = math.degrees(math.atan2(-a['x'],
                                math.sqrt(a['y'] ** 2 + a['z'] ** 2))) \
        - PITCH_OFFSET_DEG
    return r, p


def init_attitude():
    rs = ps = 0.0
    n = 50
    for _ in range(n):
        r, p = read_accel_angles()
        rs += r
        ps += p
        time.sleep(0.005)
    ang['roll'] = math.radians(rs / n)
    ang['pitch'] = math.radians(ps / n)
    ang['yaw'] = 0.0
    print(f"Attitude initialised: roll={rs/n:+.2f} deg, pitch={ps/n:+.2f} deg")


def update_attitude(dt):
    a = mpu.get_accel_data()
    g = mpu.get_gyro_data()

    gx = g['x'] - GYRO_BIAS['x']
    gy = g['y'] - GYRO_BIAS['y']
    gz = g['z'] - GYRO_BIAS['z']

    acc_r = math.degrees(math.atan2(a['y'], -a['z'])) - ROLL_OFFSET_DEG
    acc_p = math.degrees(math.atan2(-a['x'],
                                    math.sqrt(a['y'] ** 2 + a['z'] ** 2))) \
        - PITCH_OFFSET_DEG

    r = ALPHA * (math.degrees(ang['roll']) + gx * dt) + (1 - ALPHA) * acc_r
    p = ALPHA * (math.degrees(ang['pitch']) + gy * dt) + (1 - ALPHA) * acc_p
    y = math.degrees(ang['yaw']) + gz * dt

    ang['roll'] = math.radians(r)
    ang['pitch'] = math.radians(p)
    ang['yaw'] = math.radians(y)

    rates = {'roll': math.radians(gx),
             'pitch': math.radians(gy),
             'yaw': math.radians(gz)}
    return ang, rates


# ------------------------------ run -----------------------------------------
rows = []

predicted_deg = TAU_X_CONST * 54.9   # DC gain from the identified roll model
m0, _ = mix(TAU_X_CONST, 0.0, 0.0, BASE_THRUST_NORM)
print(f"\ntau_x = {TAU_X_CONST}, thrust = {BASE_THRUST_NORM}")
print(f"motor commands (steady): {[round(v, 4) for v in m0]}  "
      f"-> PWM {to_pwm(m0)}")
print(f"model predicts steady roll ~ {predicted_deg:.2f} deg\n")

try:
    settle, _ = mix(0.0, 0.0, 0.0, BASE_THRUST_NORM)
    settle_pwm = to_pwm(settle)
    print(f"Spooling up to thrust {BASE_THRUST_NORM} for 2s...")
    _t = time.time()
    while time.time() - _t < 2.0:
        for g, w in zip(GPIO_MOTORS, settle_pwm):
            pi.set_servo_pulsewidth(g, w)
        time.sleep(DT)

    init_attitude()
    print(f"Applying constant tau_x = {TAU_X_CONST} for {RUN_DURATION}s.\n")

    t_start = time.time()
    t_prev = t_start
    n_clip = 0

    while True:
        now = time.time()
        dt = now - t_prev
        t_prev = now
        t = now - t_start

        a, w = update_attitude(dt)

        motors, clipped = mix(TAU_X_CONST, 0.0, 0.0, BASE_THRUST_NORM)
        pwms = to_pwm(motors)
        for g, p_ in zip(GPIO_MOTORS, pwms):
            pi.set_servo_pulsewidth(g, p_)
        if clipped:
            n_clip += 1

        rows.append((t, TAU_X_CONST,
                     a['roll'], a['pitch'], a['yaw'],
                     w['roll'], w['pitch'], w['yaw'],
                     pwms[0], pwms[1], pwms[2], pwms[3]))

        if int(t / DT) % int(LOOP_RATE_HZ / 5) == 0:
            print(f"  t={t:5.2f}  roll={math.degrees(a['roll']):+6.2f}deg "
                  f"(target ~{predicted_deg:.2f})  "
                  f"rate={math.degrees(w['roll']):+7.2f}deg/s  "
                  f"pitch={math.degrees(a['pitch']):+6.2f}  "
                  f"yaw={math.degrees(a['yaw']):+6.2f}")

        if abs(a['roll']) > ANGLE_LIMIT_RAD:
            raise RuntimeError(f"ABORT: roll {math.degrees(a['roll']):.1f} deg")
        if abs(w['roll']) > RATE_LIMIT_ABORT_RAD_S:
            raise RuntimeError(f"ABORT: roll rate {math.degrees(w['roll']):.1f} deg/s")

        if t > RUN_DURATION:
            print("\nRun complete.")
            break

        slack = DT - (time.time() - now)
        if slack > 0:
            time.sleep(slack)

except KeyboardInterrupt:
    print("\nStopped by user.")
except RuntimeError as e:
    print(f"\n{e}")

finally:
    print("Spooling down...")
    steps = int(1.0 * LOOP_RATE_HZ)
    for i in range(steps):
        thr = BASE_THRUST_NORM * (1.0 - (i + 1) / steps)
        tau = TAU_X_CONST * (1.0 - (i + 1) / steps)
        m, _ = mix(tau, 0.0, 0.0, thr)
        for g, p_ in zip(GPIO_MOTORS, to_pwm(m)):
            pi.set_servo_pulsewidth(g, p_)
        time.sleep(DT)

    with open(OUTPUT_CSV, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['time', 'tau_x_cmd',
                     'roll_rad', 'pitch_rad', 'yaw_rad',
                     'roll_rate_rad_s', 'pitch_rate_rad_s', 'yaw_rate_rad_s',
                     'pwm1', 'pwm2', 'pwm3', 'pwm4'])
        wr.writerows(rows)
    print(f"Saved {len(rows)} rows to {OUTPUT_CSV}")

    if rows:
        n = len(rows)
        tail = [math.degrees(r[2]) for r in rows[int(0.7 * n):]]
        print(f"\nlast 30% mean roll = {sum(tail)/len(tail):+.2f} deg "
              f"(model predicted {predicted_deg:.2f} deg)")
