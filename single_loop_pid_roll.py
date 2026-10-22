#!/usr/bin/env python3
"""
SINGLE-LOOP PID roll controller. No cascade -- one PID acting directly on
roll ANGLE error, output goes straight to tau_x. No separate rate loop.

    roll_ref (constant, deg)
          |
         (+)--> PID --> sat[-1,1] --> tau_x --> mixer --> PWM --> ESCs
          |
    measured roll angle (complementary filter)

This is the simplest possible closed loop: one error signal (angle, not
rate), one PID, no outer/inner split. Useful as a baseline before adding
cascade structure, or for a straightforward P/PID vs. cascade comparison
in your thesis.

Note the PID here acts on ANGLE error directly, so its gains are NOT the
same as the inner (rate) PID used in the cascade scripts -- start
conservative and increase from there, since a single loop on angle alone
has no rate feedback to damp overshoot.

SAFETY
------
The angle/rate aborts below are the only automatic protection.

The first thing this script does is arm the ESCs (900us for 5s).
"""

import time
import csv
import math
import atexit
import signal

import pigpio
from mpu6050 import mpu6050

# ============================== CONFIG ====================================

BASE_THRUST_NORM = 0.2
ROLL_REF_DEG = 5.0
REF_RAMP_TIME = 2.0        # s, linear ramp 0 -> target, then hold

RUN_DURATION = 20.0
LOOP_RATE_HZ = 100

# --- single PID (angle error -> normalized torque) ------------------------
# NOT the cascade's rate-loop gains -- this PID sees angle error directly
# (radians), so it needs its own, generally much smaller, gains. These are a
# conservative starting point; tune from here rather than reusing the
# cascade values.
PID_ROLL = {'P': 0.3, 'I': 0.05, 'D': 0.15, 'N': 25.0}
TORQUE_LIMIT = 1.0

# --- safety ---------------------------------------------------------------
ANGLE_LIMIT_DEG = 30.0
RATE_LIMIT_DEG_S = 200.0
ANGLE_LIMIT_RAD = math.radians(ANGLE_LIMIT_DEG)
RATE_LIMIT_ABORT_RAD_S = math.radians(RATE_LIMIT_DEG_S)

# --- ESC arming -----------------------------------------------------------
ARM_PWM = 900
ARM_DURATION_S = 5.0

# --- hardware -------------------------------------------------------------
GPIO_MOTORS = [18, 19, 12, 13]
PWM_MIN, PWM_MAX = 1000, 2000

# --- MPU6050 calibration --------------------------------------------------
GYRO_BIAS = {'x': 5.200038, 'y': -3.915649, 'z': 1.139756}   # deg/s
ROLL_OFFSET_DEG = 4.841
PITCH_OFFSET_DEG = -0.816

ALPHA = 0.98

OUTPUT_CSV = 'single_loop_pid_roll_log.csv'

# --- mixer ----------------------------------------------------------------
ROTORS = [
    {'roll': -0.707107, 'pitch':  0.707107, 'yaw':  1.0, 'thrust': 1.0},
    {'roll':  0.707107, 'pitch': -0.707107, 'yaw':  1.0, 'thrust': 1.0},
    {'roll':  0.707107, 'pitch':  0.707107, 'yaw': -1.0, 'thrust': 1.0},
    {'roll': -0.707107, 'pitch': -0.707107, 'yaw': -1.0, 'thrust': 1.0},
]

# ==========================================================================

DT = 1.0 / LOOP_RATE_HZ


class PID:
    def __init__(self, P, I, D, N, lo, hi):
        self.P, self.I, self.D, self.N = P, I, D, N
        self.lo, self.hi = lo, hi
        self.integ = 0.0
        self.dstate = 0.0
        self.prev_e = 0.0
        self.saturated = False

    def reset(self):
        self.integ = 0.0
        self.dstate = 0.0
        self.prev_e = 0.0
        self.saturated = False

    def update(self, e, dt):
        if dt <= 0.0:
            dt = 1e-3
        p = self.P * e
        if self.D:
            self.dstate = ((self.dstate + self.D * self.N * (e - self.prev_e))
                           / (1.0 + self.N * dt))
        d = self.dstate
        i_try = self.integ + self.I * e * dt
        u_raw = p + i_try + d
        u = max(self.lo, min(self.hi, u_raw))
        # conditional anti-windup
        if u == u_raw:
            self.integ = i_try
        else:
            if not ((u_raw > self.hi and e > 0) or (u_raw < self.lo and e < 0)):
                self.integ = i_try
        self.prev_e = e
        self.saturated = (u != u_raw)
        return u


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
    print("Motor PWM stopped.")


atexit.register(cut_motors)


def _sig(signum, frame):
    cut_motors()
    raise SystemExit(f"terminated by signal {signum}")


signal.signal(signal.SIGTERM, _sig)
signal.signal(signal.SIGHUP, _sig)


# ----------------------------- ARM ----------------------------------------
print(f"Arming: {ARM_PWM} us for {ARM_DURATION_S:.0f}s...")
_t0 = time.time()
while time.time() - _t0 < ARM_DURATION_S:
    for g in GPIO_MOTORS:
        pi.set_servo_pulsewidth(g, ARM_PWM)
    time.sleep(0.02)
print("Armed.")


# ----------------------------- MPU6050 ------------------------------------
mpu = mpu6050(0x68)
ang = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}


def init_attitude():
    rs = ps = 0.0
    n = 50
    for _ in range(n):
        a = mpu.get_accel_data()
        rs += math.degrees(math.atan2(a['y'], -a['z'])) - ROLL_OFFSET_DEG
        ps += math.degrees(math.atan2(-a['x'],
              math.sqrt(a['y']**2 + a['z']**2))) - PITCH_OFFSET_DEG
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
                         math.sqrt(a['y']**2 + a['z']**2))) - PITCH_OFFSET_DEG
    r = ALPHA * (math.degrees(ang['roll']) + gx * dt) + (1 - ALPHA) * acc_r
    p = ALPHA * (math.degrees(ang['pitch']) + gy * dt) + (1 - ALPHA) * acc_p
    y = math.degrees(ang['yaw']) + gz * dt
    ang['roll'] = math.radians(r)
    ang['pitch'] = math.radians(p)
    ang['yaw'] = math.radians(y)
    return ang, {'roll': math.radians(gx),
                 'pitch': math.radians(gy),
                 'yaw': math.radians(gz)}


# ------------------------------- RUN --------------------------------------
pid = PID(**PID_ROLL, lo=-TORQUE_LIMIT, hi=TORQUE_LIMIT)
rows = []

ref_target = math.radians(ROLL_REF_DEG)

print(f"\nSingle-loop PID roll control:")
print(f"  roll reference = {ROLL_REF_DEG} deg (ramp {REF_RAMP_TIME}s)")
print(f"  PID = {PID_ROLL}")
print(f"  thrust = {BASE_THRUST_NORM}, duration = {RUN_DURATION}s")
print(f"  ABORT if |roll angle| > {ANGLE_LIMIT_DEG} deg\n")

try:
    settle, _ = mix(0.0, 0.0, 0.0, BASE_THRUST_NORM)
    print("Spooling up to base thrust for 2s...")
    _t = time.time()
    while time.time() - _t < 2.0:
        for g, w in zip(GPIO_MOTORS, to_pwm(settle)):
            pi.set_servo_pulsewidth(g, w)
        time.sleep(DT)

    init_attitude()
    pid.reset()
    print("Loop closed.\n")

    t_start = time.time()
    t_prev = t_start
    n_clip = 0
    n_sat = 0

    while True:
        now = time.time()
        dt = now - t_prev
        t_prev = now
        t = now - t_start

        a, w = update_attitude(dt)

        ref = ref_target * min(1.0, t / REF_RAMP_TIME) if REF_RAMP_TIME > 0 \
            else ref_target
        roll_err = ref - a['roll']
        tau_x = pid.update(roll_err, dt)

        motors, clipped = mix(tau_x, 0.0, 0.0, BASE_THRUST_NORM)
        pwms = to_pwm(motors)
        for g, p_ in zip(GPIO_MOTORS, pwms):
            pi.set_servo_pulsewidth(g, p_)

        if clipped: n_clip += 1
        if pid.saturated: n_sat += 1

        rows.append((t, ref,
                     a['roll'], a['pitch'], a['yaw'],
                     w['roll'], w['pitch'], w['yaw'],
                     tau_x, int(clipped), int(pid.saturated),
                     pwms[0], pwms[1], pwms[2], pwms[3]))

        if int(t / DT) % int(LOOP_RATE_HZ / 5) == 0:
            print(f"  t={t:5.2f}  "
                  f"roll={math.degrees(a['roll']):+6.2f}/{math.degrees(ref):+5.1f} deg  "
                  f"rate={math.degrees(w['roll']):+7.2f} deg/s  "
                  f"tau={tau_x:+.3f}  pwm={pwms}")

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
        m, _ = mix(0.0, 0.0, 0.0, thr)
        for g, p_ in zip(GPIO_MOTORS, to_pwm(m)):
            pi.set_servo_pulsewidth(g, p_)
        time.sleep(DT)

    with open(OUTPUT_CSV, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['time', 'roll_ref_rad',
                     'roll_rad', 'pitch_rad', 'yaw_rad',
                     'roll_rate_rad_s', 'pitch_rate_rad_s', 'yaw_rate_rad_s',
                     'tau_x', 'mixer_clipped', 'pid_saturated',
                     'pwm1', 'pwm2', 'pwm3', 'pwm4'])
        wr.writerows(rows)
    print(f"Saved {len(rows)} rows to {OUTPUT_CSV}")

    if rows:
        n = len(rows)
        print(f"\n--- summary ---")
        print(f"  PID saturated  = {100.0*n_sat/n:.1f}% of samples")
        print(f"  mixer clipped  = {100.0*n_clip/n:.1f}% of samples")
        pk_tau = max(abs(r[8]) for r in rows)
        print(f"  peak |tau_x|   = {pk_tau:.3f}")
        tail = [math.degrees(r[2]) for r in rows[int(0.7*n):]]
        print(f"  last 30% mean roll = "
              f"{sum(tail)/len(tail):+.2f} deg "
              f"(target = {ROLL_REF_DEG:+.1f})")
