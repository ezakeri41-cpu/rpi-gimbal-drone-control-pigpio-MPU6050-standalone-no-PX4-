#!/usr/bin/env python3
"""
THREE-AXIS cascaded attitude controller. Written from scratch.
No PX4, no MAVLink -- this script owns the whole chain.

Identical cascade on each axis:

    ref  --(+)-->  Kp  -->  rate_ref --(+)--> PID --> sat[-1,1] --> tau
          -|                            -|
      angle_meas                    rate_meas (optionally low-passed)

    [tau_x, tau_y, tau_z, thrust] --> mixer --> motor_i (clamped) --> PWM

All three axes must run in the SAME loop: the mixer combines all three
torques into one set of four motor commands, and one MPU6050 read supplies
all three angles and rates at the same instant.

THE THREE AXES SHARE ONE TORQUE BUDGET. See the note by ROTORS -- roll and
pitch both draw on min(thrust, 1-thrust)/0.707, so an axis that is merely
holding station still consumes authority the other axes then cannot use.
Pitch in particular rests near -4..-5 deg on this rig, so leaving
PITCH_REF_DEG at 0 makes it fight gravity from the first sample.

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

BASE_THRUST_NORM = 0.35   # slightly higher than the roll-only script: three
                          # axes share the budget, so there is more to cover
ROLL_REF_DEG     = 5.0
PITCH_REF_DEG    = -4.5   # the rig's natural resting pitch. Commanding 0 here
                          # makes the pitch loop hold against gravity for the
                          # whole run, spending shared torque budget for
                          # nothing. Set this to whatever reatime_rpy.py shows
                          # when the rig is left alone.
YAW_REF_DEG      = 0.0
REF_RAMP_TIME    = 2.0    # s, linear ramp 0 -> target, then hold
RUN_DURATION     = 20.0   # s

LOOP_RATE_HZ     = 50     # matches pigpio's PWM refresh rate exactly -- every
                          # gain in this script was verified by simulation at
                          # Ts=0.02s (50 Hz). See roll_cascade_control.py's
                          # end-of-file note for the full reasoning.

# --- outer loop: angle error -> rate reference ---------------------------
# rad/s per rad. Sets the outer bandwidth to ~Kp rad/s; keep it near 1/5 of
# the inner loop crossover (~2.4 Hz roll/pitch, ~1.9 Hz yaw). Kp = 10 puts the
# outer loop at 1.6 Hz, too close to the inner loop for a cascade.
KP_ROLL_OUTER    = 3.0
KP_PITCH_OUTER   = 3.0
KP_YAW_OUTER     = 2.0

RATE_LIMIT_ROLL  = math.radians(60.0)
RATE_LIMIT_PITCH = math.radians(60.0)
RATE_LIMIT_YAW   = math.radians(45.0)

# --- inner loop: rate error -> normalized torque -------------------------
# P/I/N are the discrete-retuned values (Simulink, Ts = 0.02, N fixed at 25).
# D is deliberately far below the Simulink value -- see the DERIVATIVE note.
PID_ROLL  = {'P': 1.0015, 'I': 1.6973, 'D': 0.03,  'N': 25.0}
PID_PITCH = {'P': 1.0,    'I': 1.7,    'D': 0.02,  'N': 25.0}
PID_YAW   = {'P': 1.5777, 'I': 12.137, 'D': 0.027, 'N': 25.0}
TORQUE_LIMIT = 1.0

# --- rate measurement filter ---------------------------------------------
# CONFIRMED: LPFILTER was actually true for roll during identification
# (contrary to what the .m files' own defaults show). Using 8 Hz, the
# script's own documented cutoff for when LPFILTER=true -- confirm this
# matches what you actually used, and edit if not.
#
# PITCH uses 15 Hz, NOT the 8 Hz used for roll. 8 Hz was checked against the
# identified pitch model and nearly destabilises the loop (phase margin
# 31.5 deg unfiltered -> 4.4 deg at 8 Hz; 6 Hz and below are outright
# unstable). Pitch's crossover sits higher (2.6 Hz) with less starting
# margin than roll, so it can't tolerate as much filter phase lag. 15 Hz
# keeps ~17 deg of margin. NOTE this means pitch's real-time filtering no
# longer matches pitch_greybox_id.m's identification cutoff if that was
# also 8 Hz -- this is a deliberate stability-over-consistency tradeoff, not
# a match to the offline fit. To use the true identification cutoff safely,
# the PID gains need to be retuned around the filtered loop first.
#
# yaw_greybox_id.m confirms 5 Hz directly in the file, checked safe.
#
# CAVEAT: MATLAB's filtfilt is zero-phase (filters forward AND backward),
# not physically realisable in real time. This uses a causal 2nd-order
# Butterworth at the same cutoff instead -- it matches the frequency-domain
# shaping but adds phase lag filtfilt never had. Expect to retune slightly
# (generally: lower gains a touch) versus what the offline fit predicts.
RATE_LPF_HZ = {'roll': 8.0, 'pitch': 15.0, 'yaw': 5.0}

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

OUTPUT_CSV = 'three_axis_cascade_log.csv'

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

rate_lpf = {ax: (Butter2(fc, DT) if fc else None)
            for ax, fc in RATE_LPF_HZ.items()}

# yaw_greybox_id.m applies filtfilt to BOTH the angle and the rate together
# (yE and ypE), so matching it means filtering yaw angle too, not just yaw
# rate. Same 5 Hz cutoff, same causal-vs-zero-phase caveat as the rate
# filter above.
yaw_angle_lpf = Butter2(5.0, DT)


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

    raw = {'roll': math.radians(gx),
           'pitch': math.radians(gy),
           'yaw': math.radians(gz)}
    filt = {ax: (rate_lpf[ax](raw[ax]) if rate_lpf[ax] else raw[ax])
            for ax in raw}

    # yaw angle: internal state (ang['yaw']) stays raw gyro-integrated --
    # same reasoning as rate, filtering must not feed back into the running
    # integration or error compounds. A separately filtered copy is what the
    # outer loop actually uses, to match yaw_greybox_id.m filtering both
    # signals together.
    yaw_angle_filt = yaw_angle_lpf(ang['yaw'])

    return ang, filt, raw, yaw_angle_filt


# ------------------------------ run ---------------------------------------
pids = {'roll':  PID(**PID_ROLL,  lo=-TORQUE_LIMIT, hi=TORQUE_LIMIT),
        'pitch': PID(**PID_PITCH, lo=-TORQUE_LIMIT, hi=TORQUE_LIMIT),
        'yaw':   PID(**PID_YAW,   lo=-TORQUE_LIMIT, hi=TORQUE_LIMIT)}
KP_OUTER   = {'roll': KP_ROLL_OUTER, 'pitch': KP_PITCH_OUTER, 'yaw': KP_YAW_OUTER}
RATE_LIMIT = {'roll': RATE_LIMIT_ROLL, 'pitch': RATE_LIMIT_PITCH,
              'yaw': RATE_LIMIT_YAW}
ref_target = {'roll':  math.radians(ROLL_REF_DEG),
              'pitch': math.radians(PITCH_REF_DEG),
              'yaw':   math.radians(YAW_REF_DEG)}
rows = []

print(f"\nrefs: roll {ROLL_REF_DEG}, pitch {PITCH_REF_DEG}, yaw {YAW_REF_DEG} deg"
      f" (ramp {REF_RAMP_TIME} s)")
print(f"thrust {BASE_THRUST_NORM}, loop {LOOP_RATE_HZ} Hz")
print(f"Kp_outer: {KP_OUTER}")
print(f"rate LPF: {RATE_LPF_HZ}")
_budget = min(BASE_THRUST_NORM, 1.0 - BASE_THRUST_NORM) / 0.707107
print(f"shared roll+pitch torque budget = {_budget:.3f}")
print(f"model says holding {ROLL_REF_DEG} deg roll needs tau_x ~ "
      f"{math.radians(ROLL_REF_DEG)/0.958:.3f}\n")

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
    for _p in pids.values():
        _p.reset()
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

        a, w, w_raw, yaw_angle_filt = update_attitude(dt)

        ramp = min(1.0, t / REF_RAMP_TIME) if REF_RAMP_TIME > 0 else 1.0
        ref = {ax: ref_target[ax] * ramp for ax in ref_target}

        # angle used for the outer loop -- yaw uses the filtered copy to
        # match yaw_greybox_id.m filtering the angle, not just the rate;
        # roll/pitch use the raw estimator angle unchanged.
        angle_for_control = {'roll': a['roll'], 'pitch': a['pitch'],
                             'yaw': yaw_angle_filt}

        rate_ref = {}
        tau = {}
        for ax in ('roll', 'pitch', 'yaw'):
            e = ref[ax] - angle_for_control[ax]
            rate_ref[ax] = clamp(KP_OUTER[ax] * e, RATE_LIMIT[ax])
            tau[ax] = pids[ax].update(rate_ref[ax] - w[ax], dt)

        tau_x, tau_y, tau_z = tau['roll'], tau['pitch'], tau['yaw']
        motors, clipped = mix(tau_x, tau_y, tau_z, BASE_THRUST_NORM)
        pwms = to_pwm(motors)
        for g, p_ in zip(GPIO_MOTORS, pwms):
            pi.set_servo_pulsewidth(g, p_)

        if clipped:
            n_clip += 1
        any_sat = any(pids[ax].saturated for ax in pids)
        if any_sat:
            n_sat += 1

        rows.append((t,
                     ref['roll'], ref['pitch'], ref['yaw'],
                     a['roll'], a['pitch'], a['yaw'],
                     yaw_angle_filt,
                     rate_ref['roll'], rate_ref['pitch'], rate_ref['yaw'],
                     w['roll'], w['pitch'], w['yaw'],
                     w_raw['roll'], w_raw['pitch'], w_raw['yaw'],
                     tau_x, tau_y, tau_z,
                     int(clipped),
                     int(pids['roll'].saturated),
                     int(pids['pitch'].saturated),
                     int(pids['yaw'].saturated),
                     pwms[0], pwms[1], pwms[2], pwms[3]))

        if int(t / DT) % int(LOOP_RATE_HZ / 5) == 0:
            print(f"  t={t:5.2f}  "
                  f"R={math.degrees(a['roll']):+6.2f}/{math.degrees(ref['roll']):+5.1f} "
                  f"P={math.degrees(a['pitch']):+6.2f}/{math.degrees(ref['pitch']):+5.1f} "
                  f"Y={math.degrees(a['yaw']):+6.2f}/{math.degrees(ref['yaw']):+5.1f}  "
                  f"tau=({tau_x:+.3f},{tau_y:+.3f},{tau_z:+.3f})  pwm={pwms}")

        for ax in ('roll', 'pitch', 'yaw'):
            if abs(a[ax]) > ANGLE_LIMIT_RAD:
                raise RuntimeError(f"ABORT {ax} angle "
                                   f"{math.degrees(a[ax]):.1f} deg")
            if abs(w_raw[ax]) > RATE_ABORT_RAD_S:
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
        wr.writerow(['time',
                     'roll_ref_rad', 'pitch_ref_rad', 'yaw_ref_rad',
                     'roll_rad', 'pitch_rad', 'yaw_rad',
                     'yaw_rad_filtered',
                     'roll_rate_ref_rad_s', 'pitch_rate_ref_rad_s',
                     'yaw_rate_ref_rad_s',
                     'roll_rate_rad_s', 'pitch_rate_rad_s', 'yaw_rate_rad_s',
                     'roll_rate_raw_rad_s', 'pitch_rate_raw_rad_s',
                     'yaw_rate_raw_rad_s',
                     'tau_x', 'tau_y', 'tau_z',
                     'mixer_clipped',
                     'roll_pid_sat', 'pitch_pid_sat', 'yaw_pid_sat',
                     'pwm1', 'pwm2', 'pwm3', 'pwm4'])
        wr.writerows(rows)
    print(f"Saved {len(rows)} rows to {OUTPUT_CSV}")

    if rows:
        n = len(rows)
        tail = rows[int(0.7 * n):]
        pk = {'roll': max(abs(r[17]) for r in rows),
              'pitch': max(abs(r[18]) for r in rows),
              'yaw': max(abs(r[19]) for r in rows)}
        budget = min(BASE_THRUST_NORM, 1.0 - BASE_THRUST_NORM) / 0.707107
        print("\n--- run summary ---")
        print(f"  peak |tau_x|={pk['roll']:.3f}  |tau_y|={pk['pitch']:.3f}  "
              f"|tau_z|={pk['yaw']:.3f}")
        print(f"  peak (|tau_x|+|tau_y|) vs shared budget {budget:.3f}: "
              f"{max(abs(r[17])+abs(r[18]) for r in rows):.3f}")
        print(f"  mixer clipped = {100.0*n_clip/n:.1f}% of samples")
        print(f"  any PID saturated = {100.0*n_sat/n:.1f}% of samples")
        for i, ax, col, tgt in ((0,'roll',4,ROLL_REF_DEG),
                                (1,'pitch',5,PITCH_REF_DEG),
                                (2,'yaw',6,YAW_REF_DEG)):
            v = [math.degrees(r[col]) for r in tail]
            print(f"  last 30% mean {ax:5s} = {sum(v)/len(v):+6.2f} deg "
                  f"(target {tgt:+.1f})")
        if max(abs(r[17])+abs(r[18]) for r in rows) > budget:
            print("  -> roll+pitch demand EXCEEDED the shared budget.")
            print("     Raise BASE_THRUST_NORM (0.5 maximises authority), or")
            print("     lower the outer gains / D, before touching anything else.")


# ==========================================================================
# LOOP RATE -- why 50 Hz, not 100/200 Hz
# ==========================================================================
# See roll_cascade_control.py's end-of-file note for the full reasoning.
# Short version: pigpio's set_servo_pulsewidth() physically refreshes the
# PWM output at 50 Hz regardless of how often the loop computes -- any
# faster computation just gets overwritten before it reaches the motors.
# Every gain in this script (roll, pitch, and yaw) was verified by
# simulation at Ts=0.02s, so 50 Hz matches the analysis exactly rather than
# being a compromise.
