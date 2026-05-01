"""
Crab-Bot Performance Plots
Plots position, load (torque) and step timing from gait_log.csv

Run: python3 plot_gait.py
Requires: pip install matplotlib pandas
"""

import sys
import os

try:
    import pandas as pd
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
except ImportError:
    print("Installing required libraries...")
    os.system("pip install matplotlib pandas")
    import pandas as pd
    import matplotlib.pyplot as plt
    import matplotlib gridspec as gridspec

LOG_FILE = "gait_log.csv"

# ── Colors per leg ───────────────────────────────────────────────────────────
LEG_COLORS = {
    "FL": "#e74c3c",
    "FR": "#3498db",
    "BL": "#2ecc71",
    "BR": "#f39c12",
}

PITCH_MOTORS = ["FL_PITCH", "FR_PITCH", "BL_PITCH", "BR_PITCH"]
YAW_MOTORS   = ["FL_YAW",   "FR_YAW",   "BL_YAW",   "BR_YAW"]

def get_leg(motor_name):
    return motor_name[:2]

def load_data():
    if not os.path.exists(LOG_FILE):
        print(f"ERROR: {LOG_FILE} not found.")
        print("Run the gait controller first and do some walking steps.")
        sys.exit(1)

    df = pd.read_csv(LOG_FILE)
    print(f"Loaded {len(df)} log entries, {df['step'].nunique()} steps\n")
    return df

def plot_positions(df, ax_pitch, ax_yaw):
    """Plot motor positions over time"""
    ax_pitch.set_title("Pitch Motor Positions Over Time", fontsize=12, fontweight="bold")
    ax_yaw.set_title("Yaw Motor Positions Over Time",   fontsize=12, fontweight="bold")

    for motor in PITCH_MOTORS:
        sub = df[df["leg"] == motor]
        if sub.empty:
            continue
        leg   = get_leg(motor)
        color = LEG_COLORS.get(leg, "gray")
        ax_pitch.plot(sub["time"], sub["position"], label=motor, color=color, linewidth=1.5)

    for motor in YAW_MOTORS:
        sub = df[df["leg"] == motor]
        if sub.empty:
            continue
        leg   = get_leg(motor)
        color = LEG_COLORS.get(leg, "gray")
        ax_yaw.plot(sub["time"], sub["position"], label=motor, color=color, linewidth=1.5)

    for ax in [ax_pitch, ax_yaw]:
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (°)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

def plot_load(df, ax_pitch, ax_yaw):
    """Plot motor load (torque proxy) over time"""
    ax_pitch.set_title("Pitch Motor Load (Torque) Over Time", fontsize=12, fontweight="bold")
    ax_yaw.set_title("Yaw Motor Load (Torque) Over Time",     fontsize=12, fontweight="bold")

    for motor in PITCH_MOTORS:
        sub = df[df["leg"] == motor]
        if sub.empty:
            continue
        leg   = get_leg(motor)
        color = LEG_COLORS.get(leg, "gray")
        ax_pitch.plot(sub["time"], sub["load"], label=motor, color=color, linewidth=1.5)

    for motor in YAW_MOTORS:
        sub = df[df["leg"] == motor]
        if sub.empty:
            continue
        leg   = get_leg(motor)
        color = LEG_COLORS.get(leg, "gray")
        ax_yaw.plot(sub["time"], sub["load"], label=motor, color=color, linewidth=1.5)

    for ax in [ax_pitch, ax_yaw]:
        ax.axhline(y=200, color="red", linestyle="--", alpha=0.5, label="contact threshold")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Load (0-1000)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

def plot_step_timing(df, ax):
    """Plot step cycle times as a proxy for speed"""
    ax.set_title("Step Cycle Time (Speed Proxy)", fontsize=12, fontweight="bold")

    steps     = df.groupby("step")["time"]
    step_nums = []
    durations = []

    prev_end = None
    for step, times in steps:
        start = times.min()
        end   = times.max()
        dur   = end - start
        step_nums.append(step)
        durations.append(dur)

    ax.bar(step_nums, durations, color="#9b59b6", alpha=0.7, edgecolor="white")
    ax.set_xlabel("Step Number")
    ax.set_ylabel("Cycle Duration (s)")
    ax.grid(True, alpha=0.3, axis="y")

    if durations:
        avg = sum(durations) / len(durations)
        ax.axhline(y=avg, color="red", linestyle="--", label=f"avg: {avg:.2f}s")
        ax.legend(fontsize=8)

def plot_load_heatmap(df, ax):
    """Heatmap of average load per motor per phase"""
    ax.set_title("Avg Load per Motor per Phase (Ground Contact Map)", fontsize=12, fontweight="bold")

    motors = PITCH_MOTORS + YAW_MOTORS
    phases = sorted(df["phase"].unique())

    data = []
    for motor in motors:
        row = []
        for phase in phases:
            sub = df[(df["leg"] == motor) & (df["phase"] == phase)]
            avg_load = sub["load"].mean() if not sub.empty else 0
            row.append(avg_load)
        data.append(row)

    im = ax.imshow(data, aspect="auto", cmap="YlOrRd", vmin=0, vmax=1000)
    ax.set_xticks(range(len(phases)))
    ax.set_xticklabels([f"Phase {p}" for p in phases])
    ax.set_yticks(range(len(motors)))
    ax.set_yticklabels(motors)
    plt.colorbar(im, ax=ax, label="Load (0-1000)")

def main():
    df = load_data()

    # Normalize time to start at 0
    df["time"] = df["time"] - df["time"].min()

    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("Crab-Bot Gait Performance Analysis", fontsize=16, fontweight="bold")
    gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    ax_pos_pitch  = fig.add_subplot(gs[0, 0])
    ax_pos_yaw    = fig.add_subplot(gs[0, 1])
    ax_load_pitch = fig.add_subplot(gs[1, 0])
    ax_load_yaw   = fig.add_subplot(gs[1, 1])
    ax_timing     = fig.add_subplot(gs[2, 0])
    ax_heatmap    = fig.add_subplot(gs[2, 1])

    plot_positions(df,   ax_pos_pitch,  ax_pos_yaw)
    plot_load(df,        ax_load_pitch, ax_load_yaw)
    plot_step_timing(df, ax_timing)
    plot_load_heatmap(df, ax_heatmap)

    plt.savefig("gait_analysis.png", dpi=150, bbox_inches="tight")
    print("Saved to gait_analysis.png")
    plt.show()

if __name__ == "__main__":
    main()
