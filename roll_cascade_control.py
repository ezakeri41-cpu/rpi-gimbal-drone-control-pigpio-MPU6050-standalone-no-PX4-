#!/usr/bin/env python3
"""
ROLL-ONLY cascaded attitude controller. Written from scratch.
No PX4, no MAVLink -- this script owns the whole chain.

    roll_ref  --(+)-->  Kp  -->  rate_ref --(+)--> PID --> sat[-1,1] --> tau_x
               -|                           -|
          roll_meas                     rate_meas (optionally low-passed)

    [tau_x, 0, 0, thrust] --> mixer --> motor_i (clamped [0,1]) --> PWM --> ESCs

Pitch and yaw torques are held at zero. They are still measured, logged and
watched by the safety aborts, because this rig has real cross-axis coupling.

WHAT THE IDENTIFIED MODEL SAYS THIS SHOULD TAKE
-----------------------------------------------
    phi_ddot = -a1*phi_dot - a0*phi + b*u,   b ~ 24.8, a0 ~ 25.9, a1 ~ 3.05
    DC gain = b/a0 = 0.958 rad per unit u = 54.9 deg per unit u

So holding  5 deg needs tau_x ~ 0.091
       and 10 deg needs tau_x ~ 0.182

At thrust 0.2 that puts the motors at 0.136/0.264 and 0.071/0.329 -- nowhere
near the [0,1] limits. Tracking 5 deg genuinely is not demanding. If this
script ever runs near |tau_x| = 1, something is wrong; look at the saturation
summary printed at the end of the run rather than reaching for the gains.
"""

import time
import csv
import math
import atexit
import signal

import pigpio
from mpu6050 import mpu6050

# ============================== CONFIG ====================================

BASE_THRUST_NORM = 0.30   # see the note on torque budget below
ROLL_REF_DEG     = 5.0
REF_RAMP_TIME    = 2.0    # s, linear ramp 0 -> target, then hold
RUN_DURATION     = 20.0   # s

LOOP_RATE_HZ     = 50     # matches pigpio's PWM refresh rate exactly -- see
                          # "LOOP RATE" note at the bottom of this file for
                          # why running the computation faster than this
                          # buys nothing, and why every gain above was
                          # actually verified at this rate, not 100 Hz

# --- outer loop: angle error -> rate reference ---------------------------
KP_ROLL_OUTER    = 3.0    # rad/s per rad. Sets the outer bandwidth to
                          # ~Kp rad/s = 0.48 Hz, about 1/5 of the inner
                          # loop's ~2.4 Hz crossover -- the usual cascade
                          # separation. Kp = 10 puts the outer loop at
                          # 1.6 Hz, too close to the inner loop.
RATE_LIMIT_ROLL  = math.radians(60.0)

# --- inner loop: rate error -> normalized torque -------------------------
# P/I/N are the discrete-retuned values (Simulink, Ts = 0.02, N fixed at 25).
# D is deliberately far below the Simulink value -- see the DERIVATIVE note.
PID_ROLL = {'P': 1.0015, 'I': 1.6973, 'D': 0.03, 'N': 25.0}
TORQUE_LIMIT = 1.0

# --- rate measurement filter ---------------------------------------------
# CONFIRMED: LPFILTER was actually true for roll during identification
# (contrary to what the .m file's own default shows) -- using 8 Hz, the
# script's own documented cutoff for when LPFILTER=true. Double-check this
# is the value you actually used; edit here if it was different.
#
# CAVEAT: MATLAB's filtfilt is zero-phase (filters forward AND backward),
# not physically realisable in real time. This uses a causal 2nd-order
# Butterworth at the same cutoff instead -- it matches the frequency-domain
# shaping but adds phase lag filtfilt never had. Expect to retune slightly
# (generally: lower gains a touch) versus what the offline fit predicts.
RATE_LPF_HZ = 8.0

# --- safety ---------------------------------------------------------------
ANGLE_LIMIT_DEG   = 35.0
RATE_LIMIT_DEG_S  = 250.0

# --- ESC arming -----------------------------------------------------------
ARM_PWM = 900
ARM_DURATION_S = 5.0

# --- hardware -------------------------------------------------------------
GPIO_MOTORS = [18, 19, 12, 13]        # M1, M2, M3, M4
PWM_MIN, PWM_MAX = 1000, 2000

# --- MPU6050 calibration --------------------------------------------------
GYRO_BIAS = {'x': 5.200038, 'y': -3.915649, 'z': 1.139756}   # deg/s

# Tilt offsets. NOTE THE SIGNS -- these are not both "subtract".
# mpu6050_tilt.py reported  x_tilt = atan2(ax,|az|) = +0.816 deg
#                           y_tilt = atan2(ay,|az|) = +4.841 deg
# The roll formula below is atan2(ay, -az), i.e. the SAME expression as
# y_tilt, so its offset is subtracted.
# The pitch formula below is atan2(-ax, ...), i.e. the NEGATIVE of x_tilt,
# so its offset must be ADDED. Subtracting it (as an earlier version of
# this script did) leaves a standing pitch bias of -2*0.816 = -1.63 deg.
ROLL_OFFSET_DEG  = +4.841
PITCH_OFFSET_DEG = -0.816     # sign already flipped, so it is subtracted below

ALPHA = 0.98   # complementary filter weight (gyro vs accel)

OUTPUT_CSV = 'roll_cascade_log.csv'

# --- mixer ----------------------------------------------------------------
# PX4 quad_x: rows = M1..M4, cols = [roll, pitch, yaw, thrust]
ROTORS = [
    {'roll': -0.707107, 'pitch':  0.707107, 'yaw':  1.0, 'thrust': 1.0},
    {'roll':  0.707107, 'pitch': -0.707107, 'yaw':  1.0, 'thrust': 1.0},
    {'roll':  0.707107, 'pitch':  0.707107, 'yaw': -1.0, 'thrust': 1.0},
    {'roll': -0.707107, 'pitch': -0.707107, 'yaw': -1.0, 'thrust': 1.0},
]

# TORQUE BUDGET
# motor_i = thrust +/- 0.707*(|tau_x|+|tau_y|) +/- |tau_z|, clamped to [0,1].
# So the usable roll+pitch torque is min(thrust, 1-thrust)/0.707:
#     thrust 0.2 -> 0.283      thrust 0.4 -> 0.566
#     thrust 0.3 -> 0.424      thrust 0.5 -> 0.707  (maximum)
# Holding 5 deg needs only 0.09, so 0.2 is enough in principle -- but the
# controller also passes sensor noise through to the motors, and that is
# what eats the budget. 0.30 leaves comfortable margin.

# ==========================================================================

DT = 1.0 / LOOP_RATE_HZ
ANGLE_LIMIT_RAD = math.radians(ANGLE_LIMIT_DEG)
RATE_ABORT_RAD_S = math.radians(RATE_LIMIT_DEG_S)


class Butter2:
    """Causal 2nd-order Butterworth low-pass (bilinear transform).

    NOTE: yaw_greybox_id.m used filtfilt, which is ZERO-PHASE and runs the
    data both forwards and backwards. That is impossible in real time. Any
    causal filter adds phase lag, so a controller filtered this way does not
    see the same signal the model was fitted to. Use sparingly.
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
        if not self.primed:            # start from the current value, not 0
            self.x1 = self.x2 = self.y1 = self.y2 = x
            self.primed = True
        y = (self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
             - self.a1 * self.y1 - self.a2 * self.y2)
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y


class PID:
    """Parallel PID, filtered derivative, conditional-integration anti-windup.

        P + I/s + D*N/(1 + N/s)      (the Simulink compensator form)

    Integral and derivative both use backward Euler.

    DERIVATIVE NOTE: the Simulink tuner works against a noise-free plant, so
    it happily returns a large D. Against the real gyro noise measured in the
    chirp logs (0.070 rad/s RMS above 5 Hz) the Simulink D of 0.124 produces a
    derivative term of 0.25 RMS and 0.98 peak -- comparable to, or larger
    than, the entire torque the axis actually needs (0.09 to hold 5 deg). It
    saturates the mixer on noise alone. D = 0.03 keeps the damping benefit
    with roughly a quarter of the noise gain.
    """

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

        if self.D != 0.0:
            self.dstate = ((self.dstate + self.D * self.N * (e - self.prev_e))
                           / (1.0 + self.N * dt))
        else:
            self.dstate = 0.0
        d = self.dstate

        i_try = self.integ + self.I * e * dt
        u_raw = p + i_try + d
        u = max(self.lo, min(self.hi, u_raw))

        # conditional integration: only integrate if not saturated, or if the
        # error would drive the output back into range
        if u == u_raw:
            self.integ = i_try
        else:
            driving_deeper = ((u_raw > self.hi and e > 0.0) or
                              (u_raw < self.lo and e < 0.0))
            if not driving_deeper:
                self.integ = i_try

        self.prev_e = e
        self.saturated = (u != u_raw)
        return u


def mix(tau_x, tau_y, tau_z, thrust):
    """Allocate to four motors, clamp each to [0,1].

    Returns (motor_commands, any_clamped)."""
    raw = [tau_x * r['roll'] + tau_y * r['pitch'] + tau_z * r['yaw']
           + thrust * r['thrust'] for r in ROTORS]
    out = [max(0.0, min(1.0, v)) for v in raw]
    return out, any(abs(a - b) > 1e-12 for a, b in zip(raw, out))


def to_pwm(motors):
    return [int(round(PWM_MIN + m * (PWM_MAX - PWM_MIN))) for m in motors]


def clamp(v, lim):
    return max(-lim, min(lim, v))


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
print(f"Arming: {ARM_PWM} us to all four ESCs for {ARM_DURATION_S:.0f} s...")
_t0 = time.time()
while time.time() - _t0 < ARM_DURATION_S:
    for g in GPIO_MOTORS:
        pi.set_servo_pulsewidth(g, ARM_PWM)
    time.sleep(0.02)
print("Armed.")


# ------------------------- attitude estimator -----------------------------
mpu = mpu6050(0x68)
ang = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}     # rad

rate_lpf = Butter2(RATE_LPF_HZ, DT) if RATE_LPF_HZ else None


def read_accel_angles():
    """Roll and pitch from gravity alone, in degrees."""
    a = mpu.get_accel_data()
    r = math.degrees(math.atan2(a['y'], -a['z'])) - ROLL_OFFSET_DEG
    p = math.degrees(math.atan2(-a['x'],
                                math.sqrt(a['y'] ** 2 + a['z'] ** 2))) \
        - PITCH_OFFSET_DEG
    return r, p


def init_attitude():
    """Seed the filter from the accelerometer.

    Starting the estimate at zero instead means the complementary filter has
    to converge from 0 to the true angle with its ~1 s time constant, so the
    controller spends its first second chasing an estimate that is still
    settling."""
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

    gx = g['x'] - GYRO_BIAS['x']       # deg/s
    gy = g['y'] - GYRO_BIAS['y']
    gz = g['z'] - GYRO_BIAS['z']

    acc_r = math.degrees(math.atan2(a['y'], -a['z'])) - ROLL_OFFSET_DEG
    acc_p = math.degrees(math.atan2(-a['x'],
                                    math.sqrt(a['y'] ** 2 + a['z'] ** 2))) \
        - PITCH_OFFSET_DEG

    r = ALPHA * (math.degrees(ang['roll']) + gx * dt) + (1 - ALPHA) * acc_r
    p = ALPHA * (math.degrees(ang['pitch']) + gy * dt) + (1 - ALPHA) * acc_p
    y = math.degrees(ang['yaw']) + gz * dt        # gyro only, drifts

    ang['roll'] = math.radians(r)
    ang['pitch'] = math.radians(p)
    ang['yaw'] = math.radians(y)

    raw_roll_rate = math.radians(gx)
    filt_roll_rate = rate_lpf(raw_roll_rate) if rate_lpf else raw_roll_rate

    return ang, {'roll_raw': raw_roll_rate,
                 'roll': filt_roll_rate,
                 'pitch': math.radians(gy),
                 'yaw': math.radians(gz)}


# ------------------------------ run ---------------------------------------
pid = PID(**PID_ROLL, lo=-TORQUE_LIMIT, hi=TORQUE_LIMIT)
ref_target = math.radians(ROLL_REF_DEG)
rows = []

print(f"\nroll ref {ROLL_REF_DEG} deg (ramp {REF_RAMP_TIME} s), "
      f"thrust {BASE_THRUST_NORM}, loop {LOOP_RATE_HZ} Hz")
print(f"Kp_outer={KP_ROLL_OUTER}  PID={PID_ROLL}  "
      f"rate LPF={'off' if not RATE_LPF_HZ else str(RATE_LPF_HZ)+' Hz'}")
print(f"model says tau_x ~ {math.radians(ROLL_REF_DEG)/0.958:.3f} should hold "
      f"{ROLL_REF_DEG} deg\n")

try:
    settle, _ = mix(0.0, 0.0, 0.0, BASE_THRUST_NORM)
    settle_pwm = to_pwm(settle)
    print(f"Spooling up to thrust {BASE_THRUST_NORM} (PWM {settle_pwm}) for 2 s...")
    _t = time.time()
    while time.time() - _t < 2.0:
        for g, w in zip(GPIO_MOTORS, settle_pwm):
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

        err = ref - a['roll']
        rate_ref = clamp(KP_ROLL_OUTER * err, RATE_LIMIT_ROLL)
        rate_err = rate_ref - w['roll']
        tau_x = pid.update(rate_err, dt)

        motors, clipped = mix(tau_x, 0.0, 0.0, BASE_THRUST_NORM)
        pwms = to_pwm(motors)
        for g, p_ in zip(GPIO_MOTORS, pwms):
            pi.set_servo_pulsewidth(g, p_)

        if clipped:
            n_clip += 1
        if pid.saturated:
            n_sat += 1

        rows.append((t, ref, a['roll'], a['pitch'], a['yaw'],
                     rate_ref, w['roll'], w['roll_raw'], w['pitch'], w['yaw'],
                     tau_x, int(clipped), int(pid.saturated),
                     pwms[0], pwms[1], pwms[2], pwms[3]))

        if int(t / DT) % int(LOOP_RATE_HZ / 5) == 0:
            print(f"  t={t:5.2f}  roll={math.degrees(a['roll']):+6.2f}"
                  f"/{math.degrees(ref):+5.2f}  "
                  f"rate={math.degrees(w['roll']):+7.2f}"
                  f"/{math.degrees(rate_ref):+6.2f} deg/s  "
                  f"tau={tau_x:+.3f}  pwm={pwms}")

        for ax in ('roll', 'pitch', 'yaw'):
            if abs(a[ax]) > ANGLE_LIMIT_RAD:
                raise RuntimeError(f"ABORT {ax} angle "
                                   f"{math.degrees(a[ax]):.1f} deg")
            if abs(w[ax]) > RATE_ABORT_RAD_S:
                raise RuntimeError(f"ABORT {ax} rate "
                                   f"{math.degrees(w[ax]):.1f} deg/s")

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
                     'roll_rate_ref_rad_s', 'roll_rate_rad_s',
                     'roll_rate_raw_rad_s', 'pitch_rate_rad_s',
                     'yaw_rate_rad_s',
                     'tau_x', 'mixer_clipped', 'pid_saturated',
                     'pwm1', 'pwm2', 'pwm3', 'pwm4'])
        wr.writerows(rows)
    print(f"Saved {len(rows)} rows to {OUTPUT_CSV}")

    if rows:
        n = len(rows)
        taus = [abs(r[10]) for r in rows]
        tail = rows[int(0.7 * n):]
        print("\n--- run summary ---")
        print(f"  peak |tau_x|        = {max(taus):.3f}"
              f"   (model needs ~{math.radians(ROLL_REF_DEG)/0.958:.3f})")
        print(f"  mixer clipped       = {100.0*n_clip/n:.1f}% of samples")
        print(f"  inner PID saturated = {100.0*n_sat/n:.1f}% of samples")
        if tail:
            fin = [math.degrees(r[2]) for r in tail]
            print(f"  last 30%: mean roll = {sum(fin)/len(fin):+.2f} deg "
                  f"(target {ROLL_REF_DEG})")
        budget = min(BASE_THRUST_NORM, 1.0 - BASE_THRUST_NORM) / 0.707107
        print(f"  torque budget at thrust {BASE_THRUST_NORM} = {budget:.3f}")
        if max(taus) > budget:
            print("  -> peak demand EXCEEDED the budget: raise BASE_THRUST_NORM,")
            print("     or lower KP_ROLL_OUTER / D, before touching anything else.")


# ==========================================================================
# LOOP RATE -- why 50 Hz, not 100/200 Hz
# ==========================================================================
# 50 Hz was chosen to match pigpio's PWM refresh rate exactly, and this is a
# CORRECTION from an earlier version of this note that recommended 100 Hz.
#
# pigpio's set_servo_pulsewidth() emits a 50 Hz pulse train regardless of how
# often it is called. Calling it at 100 or 200 Hz does not make the ESC see
# more updates a second -- it still gets a new pulse every 20 ms, and any
# intermediate values computed between those boundaries are silently
# overwritten before they are ever physically applied. So the achievable
# closed-loop bandwidth is capped by that 20 ms actuation refresh no matter
# how fast the control law is computed. Running the computation faster than
# 50 Hz buys nothing on the actuation side -- it only burns CPU and I2C
# bandwidth on setpoints that never reach the motors.
#
# More importantly: every PID gain and stability margin in this script was
# verified by simulation at Ts = 0.02 s (50 Hz), not 0.01 s. Running the loop
# at 100 Hz would mean using gains that were never actually checked at that
# rate. 50 Hz is not a compromise here -- it is the configuration that
# matches the analysis.
#
# Raising the actuation rate for real (not just the computation rate) would
# need set_PWM_frequency() + set_PWM_dutycycle() instead of
# set_servo_pulsewidth() (most analogue ESCs accept 200-400 Hz), combined
# with re-verifying every gain at the new sample time -- a change worth
# doing deliberately on its own, not as a side effect of picking a bigger
# number for LOOP_RATE_HZ.

