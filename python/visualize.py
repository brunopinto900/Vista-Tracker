#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
import yaml

# ============================================================
# LOAD LOG
# ============================================================

LOG_FILE = "../data/log.csv"
CONFIG_FILE = "../config/config.yaml"

df = pd.read_csv(LOG_FILE)

# ============================================================
# LOAD OBSTACLES FROM YAML
# ============================================================

with open(CONFIG_FILE, "r") as f:
    cfg = yaml.safe_load(f)

obstacles = cfg.get("world", {}).get("obstacles", [])

# ============================================================
# DERIVED SIGNALS
# ============================================================

DESIRED_DISTANCE = cfg.get("controller", {}).get("desired_distance")

df["distance"] = np.sqrt(
    (df["target_x"] - df["drone_x"])**2 +
    (df["target_y"] - df["drone_y"])**2 +
    (df["target_z"] - df["drone_z"])**2
)

df["distance_error"] = df["distance"] - DESIRED_DISTANCE

# ============================================================
# FIGURE
# ============================================================

fig, axs = plt.subplots(2, 2, figsize=(14, 10))

ax_traj = axs[0, 0]
ax_error = axs[0, 1]
ax_vel = axs[1, 0]
ax_placeholder = axs[1, 1]

# ============================================================
# TRAJECTORY PLOT
# ============================================================

ax_traj.set_title("Drone Tracking Scenario")
ax_traj.set_xlabel("X [m]")
ax_traj.set_ylabel("Y [m]")
ax_traj.grid(True)

# Plot complete target trajectory as reference
ax_traj.plot(
    df["target_x"],
    df["target_y"],
    "g--",
    linewidth=1,
    label="Target trajectory"
)

# Obstacles
for obs in obstacles:
    x = obs["x"]
    y = obs["y"]
    size = obs["size"]

    rect = Rectangle(
        (x - size, y - size),
        2 * size,
        2 * size,
        fill=False
    )

    ax_traj.add_patch(rect)

# Animated elements
drone_path, = ax_traj.plot([], [], label="Drone trajectory")
drone_marker, = ax_traj.plot([], [], "o")
target_marker, = ax_traj.plot([], [], "o")

desired_circle, = ax_traj.plot(
    [],
    [],
    "--",
    linewidth=1,
    label="Desired distance"
)

ax_traj.legend()

# Axis limits
margin = 2.0

xmin = min(df["drone_x"].min(), df["target_x"].min()) - margin
xmax = max(df["drone_x"].max(), df["target_x"].max()) + margin

ymin = min(df["drone_y"].min(), df["target_y"].min()) - margin
ymax = max(df["drone_y"].max(), df["target_y"].max()) + margin

ax_traj.set_xlim(xmin, xmax)
ax_traj.set_ylim(ymin, ymax)
ax_traj.set_aspect("equal")

# ============================================================
# DISTANCE ERROR PLOT
# ============================================================

ax_error.set_title("Distance Error")
ax_error.set_xlabel("Time [s]")
ax_error.set_ylabel("Error [m]")
ax_error.grid(True)

error_line, = ax_error.plot([], [])

tmax = max(df["t"].max(), 1e-3)

ax_error.set_xlim(0, tmax)

err_min = df["distance_error"].min()
err_max = df["distance_error"].max()

if abs(err_max - err_min) < 1e-6:
    err_min -= 1
    err_max += 1

ax_error.set_ylim(err_min - 0.5, err_max + 0.5)

# ============================================================
# VELOCITIES
# ============================================================

ax_vel.set_title("Velocities")
ax_vel.set_xlabel("Time [s]")
ax_vel.set_ylabel("Velocity [m/s]")
ax_vel.grid(True)

drone_vx_line, = ax_vel.plot([], [], label="Drone vx")
drone_vy_line, = ax_vel.plot([], [], label="Drone vy")

target_vx_line, = ax_vel.plot([], [], label="Target vx")
target_vy_line, = ax_vel.plot([], [], label="Target vy")

ax_vel.legend()

ax_vel.set_xlim(0, tmax)

vel_min = min(
    df["drone_vx"].min(),
    df["drone_vy"].min(),
    df["target_vx"].min(),
    df["target_vy"].min()
)

vel_max = max(
    df["drone_vx"].max(),
    df["drone_vy"].max(),
    df["target_vx"].max(),
    df["target_vy"].max()
)

if abs(vel_max - vel_min) < 1e-6:
    vel_min -= 1
    vel_max += 1

ax_vel.set_ylim(vel_min - 0.5, vel_max + 0.5)

# ============================================================
# PLACEHOLDER
# ============================================================

ax_placeholder.set_title("Future Metrics")
ax_placeholder.axis("off")

# ============================================================
# ANIMATION
# ============================================================

theta = np.linspace(0, 2*np.pi, 100)

def update(i):

    # -------------------------
    # Trajectory
    # -------------------------

    drone_path.set_data(
        df["drone_x"][:i+1],
        df["drone_y"][:i+1]
    )

    drone_marker.set_data(
        [df["drone_x"][i]],
        [df["drone_y"][i]]
    )

    target_marker.set_data(
        [df["target_x"][i]],
        [df["target_y"][i]]
    )

    tx = df["target_x"][i]
    ty = df["target_y"][i]

    desired_circle.set_data(
        tx + DESIRED_DISTANCE * np.cos(theta),
        ty + DESIRED_DISTANCE * np.sin(theta)
    )

    # -------------------------
    # Distance error
    # -------------------------

    error_line.set_data(
        df["t"][:i+1],
        df["distance_error"][:i+1]
    )

    # -------------------------
    # Velocities
    # -------------------------

    drone_vx_line.set_data(
        df["t"][:i+1],
        df["drone_vx"][:i+1]
    )

    drone_vy_line.set_data(
        df["t"][:i+1],
        df["drone_vy"][:i+1]
    )

    target_vx_line.set_data(
        df["t"][:i+1],
        df["target_vx"][:i+1]
    )

    target_vy_line.set_data(
        df["t"][:i+1],
        df["target_vy"][:i+1]
    )

    return (
        drone_path,
        drone_marker,
        target_marker,
        desired_circle,
        error_line,
        drone_vx_line,
        drone_vy_line,
        target_vx_line,
        target_vy_line
    )

ani = FuncAnimation(
    fig,
    update,
    frames=len(df),
    interval=20,
    blit=False,
    repeat=False
)

plt.tight_layout()
plt.show()