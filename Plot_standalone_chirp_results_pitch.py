import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ============================================================
# Load CSV
# ============================================================

filename = "standalone_chirp_pitch_log.csv"   # edit to match your actual filename

df = pd.read_csv(filename)
df = df.dropna()

# ============================================================
# Extract signals
# ============================================================

time = df["time"].to_numpy()

# Input (normalized tau_y)
tau_y = df["tau_y_cmd"].to_numpy()

# Attitude angles: rad -> degree
roll = np.rad2deg(df["roll_rad"].to_numpy())
pitch = np.rad2deg(df["pitch_rad"].to_numpy())
yaw = np.rad2deg(df["yaw_rad"].to_numpy())

# Pitch rate: rad/s -> degree/s
pitch_rate = np.rad2deg(df["pitchspeed_rad_s"].to_numpy())

# ============================================================
# Figure 1 -- Input, Pitch angle, Pitch rate (3 subplots)
# ============================================================

fig1, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

ax[0].plot(time, tau_y, linewidth=1.2)
ax[0].set_ylabel("Normalized Torque (tau_y)")
ax[0].set_title("Pitch System Identification Experiment")
ax[0].grid(True)

ax[1].plot(time, pitch, linewidth=1.2)
ax[1].set_ylabel("Pitch (deg)")
ax[1].grid(True)

ax[2].plot(time, pitch_rate, linewidth=1.2)
ax[2].set_ylabel("Pitch Rate (deg/s)")
ax[2].set_xlabel("Time (s)")
ax[2].grid(True)

plt.tight_layout()

# ============================================================
# Figure 2 -- Roll, Pitch, Yaw (single plot, 3 lines)
# ============================================================

fig2, ax2 = plt.subplots(figsize=(10, 6))

ax2.plot(time, roll, label="Roll", linewidth=1.2)
ax2.plot(time, pitch, label="Pitch", linewidth=1.2)
ax2.plot(time, yaw, label="Yaw", linewidth=1.2)

ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Angle (deg)")
ax2.set_title("Drone Attitude During Pitch Excitation")
ax2.grid(True)
ax2.legend()

plt.tight_layout()

# ============================================================
# Display figures
# ============================================================

plt.show()
