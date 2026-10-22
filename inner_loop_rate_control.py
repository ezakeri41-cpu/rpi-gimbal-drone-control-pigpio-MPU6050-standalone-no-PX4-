#!/usr/bin/env python3
"""
INNER-LOOP-ONLY roll rate controller. No outer angle loop.

The roll rate reference is a fixed constant you set below. The inner PID
acts on the error between that reference and the measured roll rate from
the MPU6050 gyro.

    ROLL_RATE_REF (constant, deg/s)
          |
         (+)-- > PID --> sat[-1,1] --> tau_x --> mixer --> PWM --> ESCs
          |
    measured roll rate (from gyro, bias-corrected)

Purpose: test the inner PID loop in isolation, before adding the outer
angle loop on top. A well-tuned inner loop should:
  - track a small constant rate reference cleanly (e.g. 5-10 deg/s)
  - not oscillate or saturate
  - settle in under ~1 s

The model says a constant tau_x = 0.1 produces steady roll of ~5.49 deg
over time. That same torque corresponds to a peak transient rate of about
the same order. So a rate reference of 5-10 deg/s is a reasonable,
non-aggressive starting point.

SAFETY
------
A constant rate reference means the vehicle will keep rolling indefinitely
without correction -- the angle abort below is your only protection.
Set ROLL_RATE_REF_DEG_S small (start with 5) and keep the run short.

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
ROLL_RATE_REF_DEG_S = 5.0   # deg/s -- start small, this is a constant
                              # rate command, the vehicle keeps rolling

RUN_DURATION = 10.0
LOOP_RATE_HZ = 50   # matches pigpio's PWM refresh rate -- every gain in this
                    # script was verified by simulation at Ts=0.02s (50 Hz),
                    # and running the computation faster than pigpio's own
                    # 50 Hz PWM refresh buys nothing (intermediate values
                    # just get overwritten before reaching the motors)

# --- inner PID (rate error -> normalized torque) -------------------------
PID_ROLL = {'P': 1.0015, 'I': 1.6973, 'D': 0.03, 'N': 25.0}
TORQUE_LIMIT = 1.0

# --- rate measurement filter -----------------------------------------------
# Matches roll_greybox_id.m's confirmed LPFILTER=true, fcut=8 setting.
# Checked against this plant/PID combination: phase margin drops from
# 50.1 deg (unfiltered) to 25.2 deg with this filter -- lower but stable.
RATE_LPF_HZ = 8.0

# --- safety ---------------------------------------------------------------
ANGLE_LIMIT_DEG = 25.0     # tighter than the closed-loop script -- a constant
                            # rate command keeps rolling so cut off earlier
RATE_LIMIT_DEG_S = 250.0
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

OUTPUT_CSV = 'inner_loop_rate_control_log.csv'

# --- mixer ----------------------------------------------------------------
ROTORS = [
    {'roll': -0.707107, 'pitch':  0.707107, 'yaw':  1.0, 'thrust': 1.0},
    {'roll':  0.707107, 'pitch': -0.707107, 'yaw':  1.0, 'thrust': 1.0},
    {'roll':  0.707107, 'pitch':  0.707107, 'yaw': -1.0, 'thrust': 1.0},
    {'roll': -0.707107, 'pitch': -0.707107, 'yaw': -1.0, 'thrust': 1.0},
]

# ==========================================================================

DT = 1.0 / LOOP_RATE_HZ
ROLL_RATE_REF = math.radians(ROLL_RATE_REF_DEG_S)


class Butter2:
    """Causal 2nd-order Butterworth low-pass (bilinear transform).

    Matches roll_greybox_id.m's LPFILTER=true, fcut=8 setting -- with the
    caveat that filtfilt (used offline) is zero-phase, while this causal
    version adds real phase lag. Checked against the identified roll plant:
    phase margin drops from 50.1 deg (unfiltered) to 25.2 deg with this
    filter -- lower, but still a workable margin.
    """
    def __init__(self, fc, ts):
        K = math.tan(math.pi * fc * ts)
        n = 1.0 / (1.0 + math.sqrt(2.0) * K + K * K)
        self.b0 = K * K * n
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (K * K - 1.0) * n
        self.a2 = (1.0 - math.sqrt(2.0) * K + K * K) * n
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0
        self.primed = False

    def __call__(self, x):
        if not self.primed:
            self.x1 = self.x2 = self.y1 = self.y2 = x
            self.primed = True
        y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
             - self.a1 * self.y1 - self.a2 * self.y2)
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y


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

    # internal estimator state (ang['roll']) uses raw gx for integration,
    # same as always -- the filter below only affects the copy handed to
    # the control loop, so filtering can't feed back into the running
    # complementary-filter estimate.
    raw_roll_rate = math.radians(gx)
    filt_roll_rate = rate_lpf(raw_roll_rate) if rate_lpf else raw_roll_rate

    return ang, {'roll': filt_roll_rate,
                 'pitch': math.radians(gy),
                 'yaw': math.radians(gz)}


# ------------------------------- RUN --------------------------------------
rate_lpf = Butter2(RATE_LPF_HZ, DT) if RATE_LPF_HZ else None
pid = PID(**PID_ROLL, lo=-TORQUE_LIMIT, hi=TORQUE_LIMIT)
rows = []

print(f"\nInner-loop-only rate control:")
print(f"  rate reference = {ROLL_RATE_REF_DEG_S} deg/s (constant)")
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

        rate_err = ROLL_RATE_REF - w['roll']
        tau_x = pid.update(rate_err, dt)

        motors, clipped = mix(tau_x, 0.0, 0.0, BASE_THRUST_NORM)
        pwms = to_pwm(motors)
        for g, p_ in zip(GPIO_MOTORS, pwms):
            pi.set_servo_pulsewidth(g, p_)

        if clipped: n_clip += 1
        if pid.saturated: n_sat += 1

        rows.append((t, ROLL_RATE_REF,
                     a['roll'], a['pitch'], a['yaw'],
                     w['roll'], w['pitch'], w['yaw'],
                     tau_x, int(clipped), int(pid.saturated),
                     pwms[0], pwms[1], pwms[2], pwms[3]))

        if int(t / DT) % int(LOOP_RATE_HZ / 5) == 0:
            print(f"  t={t:5.2f}  "
                  f"rate_ref={ROLL_RATE_REF_DEG_S:+5.1f}  "
                  f"roll_rate={math.degrees(w['roll']):+7.2f} deg/s  "
                  f"roll={math.degrees(a['roll']):+6.2f} deg  "
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
        wr.writerow(['time', 'roll_rate_ref_rad_s',
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
        tail_rate = [math.degrees(r[5]) for r in rows[int(0.7*n):]]
        print(f"  last 30% mean roll rate = "
              f"{sum(tail_rate)/len(tail_rate):+.2f} deg/s "
              f"(ref = {ROLL_RATE_REF_DEG_S:+.1f})")
