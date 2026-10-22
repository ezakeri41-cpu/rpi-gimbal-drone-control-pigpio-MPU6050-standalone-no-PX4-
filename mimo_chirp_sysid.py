#!/usr/bin/env python3
"""
MIMO system identification: simultaneous, frequency-separated chirp
excitation on all three axes. No PX4, no MAVLink -- this script owns the
whole chain, same as the earlier standalone SISO chirp scripts.

    tau_x = A_x * sin(phase_x(t))   sweeping 0.3 - 0.8 Hz   (roll)
    tau_y = A_y * sin(phase_y(t))   sweeping 1.0 - 1.5 Hz   (pitch)
    tau_z = A_z * sin(phase_z(t))   sweeping 1.8 - 2.3 Hz   (yaw)

All three run AT THE SAME TIME, each in its own non-overlapping frequency
band. This is the standard approach for MIMO frequency-domain system
identification: because the bands don't overlap, each axis's response can
be isolated afterward (e.g. by band-pass filtering the logged roll/pitch/
yaw signals, or in the frequency domain) without needing to excite one
axis at a time and hold the others at zero, as the earlier SISO scripts
did. This lets a single run capture cross-axis coupling terms (e.g.
tau_x -> pitch response) directly, which the SISO campaign could not.

    [tau_x, tau_y, tau_z, thrust]
          |  combine  (quad-X mixer geometry)
      motor_i        (normalized fraction, [0,1])
          |  clamp, then scale
      pwm_i          (microseconds, [1000, 2000])
          |  pigpio
        ESCs

AMPLITUDE / SAFETY MARGIN
-------------------------
With all three chirps at their peak simultaneously and aligned in sign
(the worst case for any single motor), the combined mixer demand is:

    motor_i = A*(0.707 + 0.707 + 1.0) + thrust = 2.414*A + thrust

At A = 0.05 and thrust = 0.2 (the same operating point every SISO
identification run used), that gives motor commands in [0.079, 0.321] in
the worst case -- comfortable margin on both sides, verified before this
script was written rather than assumed. Do not raise A without redoing
this check.

Logs the same columns as the SISO scripts (time, all three commanded
torques, roll/pitch/yaw angles, roll/pitch/yaw rates, all four PWMs) so
existing MATLAB loading code needs only trivial changes.

SAFETY
------
Nothing else is in the loop. There is no PX4 failsafe and no
stabilization of any kind behind this script -- the angle/rate aborts
below are the ONLY automatic protection.

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

# --- operating point -------------------------------------------------------
BASE_THRUST_NORM = 0.2   # SAME operating point as every SISO identification
                          # run, for direct comparability

# --- excitation: independent, non-overlapping bands per axis --------------
AMPLITUDE = 0.05          # normalized, SAME for all three axes -- see the
                          # worst-case combined-saturation check above
CHIRP_DURATION = 40.0     # seconds -- longer than the SISO runs (30s) to
                          # give roll's band (the slowest) enough full
                          # cycles for good spectral resolution: at 0.3 Hz
                          # a 40s run covers 12 full cycles at the low end

ROLL_F0,  ROLL_F1  = 0.3, 0.8    # Hz, tau_x
PITCH_F0, PITCH_F1 = 1.0, 1.5    # Hz, tau_y
YAW_F0,   YAW_F1   = 1.8, 2.3    # Hz, tau_z

LOOP_RATE_HZ = 50         # matches pigpio's PWM refresh rate -- see the
                          # cascade control scripts' end-of-file note for
                          # why running faster buys nothing here

# --- safety -----------------------------------------------------------------
ANGLE_LIMIT_DEG  = 30.0    # abort if any axis angle exceeds this
RATE_LIMIT_DEG_S = 150.0   # abort if any axis rate exceeds this
ANGLE_LIMIT_RAD  = math.radians(ANGLE_LIMIT_DEG)
RATE_LIMIT_RAD_S = math.radians(RATE_LIMIT_DEG_S)

# --- ESC arming -------------------------------------------------------------
ARM_PWM = 900
ARM_DURATION_S = 5.0

# --- GPIO (from your four_motor_bridge.py wiring) --------------------------
GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4 = 18, 19, 12, 13
GPIO_MOTORS = [GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4]
PWM_MIN, PWM_MAX = 1000, 2000

# --- MPU6050 calibration (your latest confirmed values) --------------------
GYRO_BIAS = {'x': 5.200038, 'y': -3.915649, 'z': 1.139756}   # deg/s
ROLL_OFFSET_DEG  = 4.841
PITCH_OFFSET_DEG = -0.816    # NOTE the sign: this is the negative of the
                              # tilt script's measured x_tilt, because the
                              # pitch formula below negates ax -- see
                              # roll_cascade_control.py for the full
                              # derivation of why this sign is correct

ALPHA = 0.98   # complementary filter weight (gyro vs accel)

OUTPUT_CSV = 'mimo_chirp_sysid_log.csv'

# --- mixer geometry (PX4 v1.9.2 quad_x, confirmed from your source) --------
ROTORS = [
    {'roll': -0.707107, 'pitch':  0.707107, 'yaw':  1.0, 'thrust': 1.0},  # M1 front-right, CCW
    {'roll':  0.707107, 'pitch': -0.707107, 'yaw':  1.0, 'thrust': 1.0},  # M2 rear-left,   CCW
    {'roll':  0.707107, 'pitch':  0.707107, 'yaw': -1.0, 'thrust': 1.0},  # M3 front-left,  CW
    {'roll': -0.707107, 'pitch': -0.707107, 'yaw': -1.0, 'thrust': 1.0},  # M4 rear-right,  CW
]

# ==========================================================================


def chirp(t, f0, f1, duration, amplitude):
    """Linear-frequency-sweep chirp from f0 to f1 Hz over `duration` s."""
    if t > duration:
        return 0.0
    k = (f1 - f0) / duration
    phase = 2.0 * math.pi * (f0 * t + 0.5 * k * t * t)
    return amplitude * math.sin(phase)


def mix(tau_x, tau_y, tau_z, thrust, t=None, warn=True):
    """Combine the four normalized commands into four normalized per-motor
    values, then clamp each to [0,1]. Plain combine-and-clamp -- see the
    SISO scripts for the caveat about PX4's airmode desaturation not being
    replicated here."""
    raw = []
    for r in ROTORS:
        raw.append(tau_x * r['roll'] +
                   tau_y * r['pitch'] +
                   tau_z * r['yaw'] +
                   thrust * r['thrust'])

    clamped = []
    for i, v in enumerate(raw):
        if warn and (v < 0.0 or v > 1.0):
            where = f" at t={t:.2f}s" if t is not None else ""
            print(f"  !! SATURATION{where}: motor {i + 1} raw={v:.4f} "
                  f"outside [0,1] -- clamped. This should not happen given "
                  f"the amplitude/thrust margin check above -- investigate "
                  f"if it does.")
        clamped.append(max(0.0, min(1.0, v)))

    return clamped


def to_pwm(motor_norm):
    """Normalized motor fraction [0,1] -> PWM microseconds [1000,2000]."""
    return [int(round(PWM_MIN + m * (PWM_MAX - PWM_MIN))) for m in motor_norm]


# --------------------------- pigpio setup ----------------------------------
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


# ------------------------- 1) ESC ARMING -----------------------------------
# First thing the script does. Nothing here is logged.
print(f"Arming ESCs: {ARM_PWM}us to all four motors for {ARM_DURATION_S}s...")
_arm_t0 = time.time()
while time.time() - _arm_t0 < ARM_DURATION_S:
    for gpio in GPIO_MOTORS:
        pi.set_servo_pulsewidth(gpio, ARM_PWM)
    time.sleep(0.02)
print("Arming complete.")


# ------------------------- MPU6050 ------------------------------------------
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


# --------------------------- 2) MAIN LOOP -----------------------------------
log_rows = []
period = 1.0 / LOOP_RATE_HZ

print(f"MIMO chirp excitation, {CHIRP_DURATION}s, amplitude={AMPLITUDE} on all axes:")
print(f"  tau_x (roll):  {ROLL_F0}-{ROLL_F1} Hz")
print(f"  tau_y (pitch): {PITCH_F0}-{PITCH_F1} Hz")
print(f"  tau_z (yaw):   {YAW_F0}-{YAW_F1} Hz")
print(f"Base thrust={BASE_THRUST_NORM}")
print(f"Aborts: |angle| > {ANGLE_LIMIT_DEG} deg, "
      f"|rate| > {RATE_LIMIT_DEG_S} deg/s (any axis).")

try:
    # settle at symmetric base thrust before exciting anything
    settle_pwms = to_pwm(mix(0.0, 0.0, 0.0, BASE_THRUST_NORM, warn=False))
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

        tau_x = chirp(t, ROLL_F0,  ROLL_F1,  CHIRP_DURATION, AMPLITUDE)
        tau_y = chirp(t, PITCH_F0, PITCH_F1, CHIRP_DURATION, AMPLITUDE)
        tau_z = chirp(t, YAW_F0,   YAW_F1,   CHIRP_DURATION, AMPLITUDE)

        motor_norm = mix(tau_x, tau_y, tau_z, BASE_THRUST_NORM, t=t)
        pwms = to_pwm(motor_norm)

        for gpio, pwm in zip(GPIO_MOTORS, pwms):
            pi.set_servo_pulsewidth(gpio, pwm)

        log_rows.append((t, tau_x, tau_y, tau_z,
                         angles['roll'], angles['pitch'], angles['yaw'],
                         rates['roll'], rates['pitch'], rates['yaw'],
                         pwms[0], pwms[1], pwms[2], pwms[3]))

        if int(t / period) % 10 == 0:
            print(f"  t={t:6.2f}s  "
                  f"tau=({tau_x:+.4f},{tau_y:+.4f},{tau_z:+.4f})  "
                  f"roll={math.degrees(angles['roll']):6.2f}  "
                  f"pitch={math.degrees(angles['pitch']):6.2f}  "
                  f"yaw={math.degrees(angles['yaw']):6.2f}deg  "
                  f"pwm={pwms}")

        for axis in ('roll', 'pitch', 'yaw'):
            if abs(angles[axis]) > ANGLE_LIMIT_RAD:
                print(f"ABORT: {axis} exceeded {ANGLE_LIMIT_DEG} deg.")
                raise KeyboardInterrupt
            if abs(rates[axis]) > RATE_LIMIT_RAD_S:
                print(f"ABORT: {axis} rate exceeded {RATE_LIMIT_DEG_S} deg/s.")
                raise KeyboardInterrupt

        if t > CHIRP_DURATION:
            print("MIMO chirp sequence complete.")
            break

        time.sleep(max(0.0, period - (time.time() - now)))

except KeyboardInterrupt:
    print("\nStopped.")

finally:
    # ramp thrust down rather than cutting abruptly
    print("Ramping thrust down...")
    ramp_steps = int(1.0 * LOOP_RATE_HZ)
    for i in range(ramp_steps):
        thr = BASE_THRUST_NORM * (1.0 - (i + 1) / ramp_steps)
        ramp_pwms = to_pwm(mix(0.0, 0.0, 0.0, thr, warn=False))
        for gpio, pwm in zip(GPIO_MOTORS, ramp_pwms):
            pi.set_servo_pulsewidth(gpio, pwm)
        time.sleep(period)

    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'tau_x_cmd', 'tau_y_cmd', 'tau_z_cmd',
                         'roll_rad', 'pitch_rad', 'yaw_rad',
                         'rollspeed_rad_s', 'pitchspeed_rad_s', 'yawspeed_rad_s',
                         'pwm1', 'pwm2', 'pwm3', 'pwm4'])
        writer.writerows(log_rows)
    print(f"Saved {len(log_rows)} rows to {OUTPUT_CSV}")
    # cut_motors() runs automatically on exit via atexit
