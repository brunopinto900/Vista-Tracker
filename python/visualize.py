#!/usr/bin/env python3

import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import yaml
from matplotlib.animation import FuncAnimation

# ============================================================
# LOAD LOG
# ============================================================

LOG_FILE = os.path.join(os.path.dirname(__file__), "../data/log.csv")

df = pd.read_csv(LOG_FILE)

# ============================================================
# LOAD CONFIG FROM YAML  (supports base: inheritance)
# ============================================================

def _deep_merge(base, override):
    result = dict(base)
    for key, val in override.items():
        if key == "base":
            continue
        if key in result and isinstance(result[key], dict) and isinstance(val, dict):
            result[key] = _deep_merge(result[key], val)
        else:
            result[key] = val
    return result

def load_config(path):
    with open(path) as f:
        node = yaml.safe_load(f) or {}
    if "base" in node:
        base_path = os.path.join(os.path.dirname(os.path.abspath(path)),
                                 node["base"])
        base = load_config(base_path)
        return _deep_merge(base, node)
    return node

_default_cfg = os.path.join(os.path.dirname(__file__), "../config/config.yaml")
_cfg_path    = sys.argv[1] if len(sys.argv) > 1 else _default_cfg

cfg = load_config(_cfg_path)

obstacles        = cfg.get("world", {}).get("obstacles", [])
grid             = cfg.get("world", {}).get("grid", {})
DESIRED_DISTANCE = cfg["controller"]["desired_distance"]

GRID_X_MIN = grid.get("x_min", -12.5)
GRID_X_MAX = grid.get("x_max",  12.5)
GRID_Y_MIN = grid.get("y_min", -12.5)
GRID_Y_MAX = grid.get("y_max",  12.5)

# ============================================================
# DERIVED SIGNALS
# ============================================================

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

ax_traj       = axs[0, 0]
ax_error      = axs[0, 1]
ax_vel        = axs[1, 0]
ax_cmd        = axs[1, 1]

# ============================================================
# TRAJECTORY PLOT
# ============================================================

ax_traj.set_title("Drone Tracking Scenario")
ax_traj.set_xlabel("X [m]")
ax_traj.set_ylabel("Y [m]")
ax_traj.grid(True, zorder=0)
ax_traj.set_xlim(GRID_X_MIN, GRID_X_MAX)
ax_traj.set_ylim(GRID_Y_MIN, GRID_Y_MAX)
ax_traj.set_aspect("equal")

# Obstacles — filled rectangles
for obs in obstacles:
    ox   = obs["x"]
    oy   = obs["y"]
    size = obs["size"]
    rect = patches.Rectangle(
        (ox - size, oy - size),     # bottom-left corner
        2 * size,                   # width
        2 * size,                   # height
        linewidth=1.5,
        edgecolor="#8B0000",
        facecolor="#FF6B6B",
        alpha=0.7,
        zorder=2,
        label="Obstacle" if obs is obstacles[0] else "_nolegend_"
    )
    ax_traj.add_patch(rect)

# Complete target trajectory (static reference)
ax_traj.plot(
    df["target_x"], df["target_y"],
    "g--", linewidth=1, label="Target trajectory", zorder=3
)

# Animated elements
drone_path,    = ax_traj.plot([], [], "b-",  linewidth=1.5,
                               label="Drone trajectory", zorder=4)
drone_marker,  = ax_traj.plot([], [], "bo",  markersize=8,
                               label="Drone", zorder=5)
target_marker, = ax_traj.plot([], [], "g^",  markersize=8,
                               label="Target", zorder=5)
desired_circle,= ax_traj.plot([], [], "b--", linewidth=1,
                               label="Desired distance", zorder=4)

ax_traj.legend(loc="upper left", fontsize=8)

# ============================================================
# DISTANCE ERROR PLOT
# ============================================================

ax_error.set_title("Distance Error")
ax_error.set_xlabel("Time [s]")
ax_error.set_ylabel("Error [m]")
ax_error.axhline(0, color="k", linewidth=0.8, linestyle="--")
ax_error.grid(True)

error_line, = ax_error.plot([], [], "r-", linewidth=1.5)

tmax = max(df["t"].max(), 1e-3)
ax_error.set_xlim(0, tmax)

err_min = df["distance_error"].min()
err_max = df["distance_error"].max()
if abs(err_max - err_min) < 1e-6:
    err_min -= 1.0
    err_max += 1.0
ax_error.set_ylim(err_min - 0.5, err_max + 0.5)

# ============================================================
# VELOCITY PLOT
# ============================================================

ax_vel.set_title("Velocities")
ax_vel.set_xlabel("Time [s]")
ax_vel.set_ylabel("Velocity [m/s]")
ax_vel.grid(True)

drone_vx_line,  = ax_vel.plot([], [], label="Drone vx")
drone_vy_line,  = ax_vel.plot([], [], label="Drone vy")
target_vx_line, = ax_vel.plot([], [], label="Target vx", linestyle="--")
target_vy_line, = ax_vel.plot([], [], label="Target vy", linestyle="--")

ax_vel.legend(fontsize=8)
ax_vel.set_xlim(0, tmax)

vel_min = min(df["drone_vx"].min(), df["drone_vy"].min(),
              df["target_vx"].min(), df["target_vy"].min())
vel_max = max(df["drone_vx"].max(), df["drone_vy"].max(),
              df["target_vx"].max(), df["target_vy"].max())
if abs(vel_max - vel_min) < 1e-6:
    vel_min -= 1.0
    vel_max += 1.0
ax_vel.set_ylim(vel_min - 0.5, vel_max + 0.5)

# ============================================================
# COMMAND VELOCITIES PLOT
# ============================================================

ax_cmd.set_title("Control Commands")
ax_cmd.set_xlabel("Time [s]")
ax_cmd.set_ylabel("Cmd velocity [m/s]")
ax_cmd.grid(True)

cmd_vx_line, = ax_cmd.plot([], [], label="vx_cmd")
cmd_vy_line, = ax_cmd.plot([], [], label="vy_cmd")
cmd_vz_line, = ax_cmd.plot([], [], label="vz_cmd")

ax_cmd.legend(fontsize=8)
ax_cmd.set_xlim(0, tmax)

cmd_min = min(df["vx_cmd"].min(), df["vy_cmd"].min(), df["vz_cmd"].min())
cmd_max = max(df["vx_cmd"].max(), df["vy_cmd"].max(), df["vz_cmd"].max())
if abs(cmd_max - cmd_min) < 1e-6:
    cmd_min -= 1.0
    cmd_max += 1.0
ax_cmd.set_ylim(cmd_min - 0.5, cmd_max + 0.5)

# ============================================================
# ANIMATION
# ============================================================

def init():
    drone_path.set_data([], [])
    drone_marker.set_data([], [])
    target_marker.set_data([], [])
    desired_circle.set_data([], [])
    error_line.set_data([], [])
    drone_vx_line.set_data([], [])
    drone_vy_line.set_data([], [])
    target_vx_line.set_data([], [])
    target_vy_line.set_data([], [])
    cmd_vx_line.set_data([], [])
    cmd_vy_line.set_data([], [])
    cmd_vz_line.set_data([], [])
    return (drone_path, drone_marker, target_marker, desired_circle,
            error_line,
            drone_vx_line, drone_vy_line, target_vx_line, target_vy_line,
            cmd_vx_line, cmd_vy_line, cmd_vz_line)

def update(frame):
    sub = df.iloc[:frame + 1]

    # Trajectory
    drone_path.set_data(sub["drone_x"], sub["drone_y"])
    drone_marker.set_data([sub["drone_x"].iloc[-1]],
                          [sub["drone_y"].iloc[-1]])
    target_marker.set_data([sub["target_x"].iloc[-1]],
                           [sub["target_y"].iloc[-1]])

    # Desired-distance circle around current target
    theta = np.linspace(0, 2 * np.pi, 120)
    cx = sub["target_x"].iloc[-1] + DESIRED_DISTANCE * np.cos(theta)
    cy = sub["target_y"].iloc[-1] + DESIRED_DISTANCE * np.sin(theta)
    desired_circle.set_data(cx, cy)

    # Distance error
    error_line.set_data(sub["t"], sub["distance_error"])

    # Velocities
    drone_vx_line.set_data(sub["t"], sub["drone_vx"])
    drone_vy_line.set_data(sub["t"], sub["drone_vy"])
    target_vx_line.set_data(sub["t"], sub["target_vx"])
    target_vy_line.set_data(sub["t"], sub["target_vy"])

    # Commands
    cmd_vx_line.set_data(sub["t"], sub["vx_cmd"])
    cmd_vy_line.set_data(sub["t"], sub["vy_cmd"])
    cmd_vz_line.set_data(sub["t"], sub["vz_cmd"])

    return (drone_path, drone_marker, target_marker, desired_circle,
            error_line,
            drone_vx_line, drone_vy_line, target_vx_line, target_vy_line,
            cmd_vx_line, cmd_vy_line, cmd_vz_line)

ani = FuncAnimation(
    fig,
    update,
    frames=len(df),
    init_func=init,
    interval=max(1, int(cfg["sim"]["dt"] * 1000)),  # real-time from dt
    blit=True
)

plt.tight_layout()
plt.show()