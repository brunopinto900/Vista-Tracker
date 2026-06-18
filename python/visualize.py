#!/usr/bin/env python3
"""
Vista-Tracker visualiser  –  4 × 2 animated dashboard.

Panels:
  [0,0]  2-D trajectory with obstacles and desired-distance circle
  [0,1]  Tracking distance error
  [1,0]  Yaw: reference, state, and ±half-FoV acceptance band
  [1,1]  Body rates: controller commands vs actual plant response
  [2,0]  Velocities: drone state vs target (reference)
  [2,1]  Roll & pitch: attitude setpoints vs state
  [3,0]  Occlusion metric  (placeholder)
  [3,1]  Reserved          (placeholder)

Static reference signals are pre-plotted for the full horizon.
Animated state signals grow frame-by-frame.
"""
from __future__ import annotations

import os
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import yaml
from matplotlib.animation import FuncAnimation

# ── Load log ──────────────────────────────────────────────────────────────────

LOG_FILE = os.path.join(os.path.dirname(__file__), "../data/log.csv")
df = pd.read_csv(LOG_FILE)

# ── Load config (supports base: inheritance) ──────────────────────────────────

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
        base_path = os.path.join(os.path.dirname(os.path.abspath(path)), node["base"])
        return _deep_merge(load_config(base_path), node)
    return node

_default_cfg = os.path.join(os.path.dirname(__file__), "../config/config.yaml")
cfg = load_config(sys.argv[1] if len(sys.argv) > 1 else _default_cfg)

# ── Config constants ──────────────────────────────────────────────────────────

obstacles        = cfg.get("world", {}).get("obstacles", [])
grid             = cfg.get("world", {}).get("grid", {})
DESIRED_DISTANCE = cfg["controller"]["desired_distance"]
ATT_KP           = cfg["controller"]["attitude_kp"]
FOV_DEG          = cfg.get("camera", {}).get("fov", 40.0)
HALF_FOV         = np.radians(FOV_DEG / 2.0)

GRID_X_MIN = grid.get("x_min", -12.5)
GRID_X_MAX = grid.get("x_max",  12.5)
GRID_Y_MIN = grid.get("y_min", -12.5)
GRID_Y_MAX = grid.get("y_max",  12.5)

tmax = max(df["t"].max(), 1e-3)

# ── Derived signals ───────────────────────────────────────────────────────────

df["distance"]       = np.sqrt((df["target_x"] - df["drone_x"])**2 +
                                (df["target_y"] - df["drone_y"])**2 +
                                (df["target_z"] - df["drone_z"])**2)
df["distance_error"] = df["distance"] - DESIRED_DISTANCE

# Yaw reference: direction from drone to target (camera-facing)
df["ref_yaw"] = np.arctan2(df["target_y"] - df["drone_y"],
                            df["target_x"] - df["drone_x"])

# Heading error: signed angle from ref to drone yaw, wrapped to [-π, π]
df["yaw_error"] = np.arctan2(
    np.sin(df["drone_yaw"] - df["ref_yaw"]),
    np.cos(df["drone_yaw"] - df["ref_yaw"]))

# Attitude setpoints back-computed from inner-loop: rate = att_kp*(des - state)
df["roll_des"]  = df["drone_roll"]  + df["roll_rate"]  / ATT_KP
df["pitch_des"] = df["drone_pitch"] + df["pitch_rate"] / ATT_KP

has_body_rates = "drone_wx" in df.columns

# ── Helper ────────────────────────────────────────────────────────────────────

def _ylim(*cols, pad=0.5):
    lo = min(c.min() for c in cols)
    hi = max(c.max() for c in cols)
    if abs(hi - lo) < 1e-6:
        lo -= 1.0; hi += 1.0
    return lo - pad, hi + pad

def _placeholder(ax, label):
    ax.set_facecolor("#f5f5f5")
    ax.text(0.5, 0.5, label, transform=ax.transAxes,
            ha="center", va="center", fontsize=11, color="#888888",
            style="italic")
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_linestyle("--"); spine.set_edgecolor("#aaaaaa")

# ── Figure layout ─────────────────────────────────────────────────────────────

fig, axs = plt.subplots(4, 2, figsize=(16, 20),
                         gridspec_kw={"height_ratios": [1.4, 1, 1, 1]})
fig.suptitle("Vista-Tracker — Simulation Dashboard", fontsize=13, y=0.995)

ax_traj, ax_err   = axs[0]
ax_yaw,  ax_rates = axs[1]
ax_vel,  ax_att   = axs[2]
ax_occ,  ax_other = axs[3]

# ── Panel (0,0): 2-D Trajectory ───────────────────────────────────────────────

ax_traj.set_title("2-D Trajectory")
ax_traj.set_xlabel("X [m]"); ax_traj.set_ylabel("Y [m]")
ax_traj.set_xlim(GRID_X_MIN, GRID_X_MAX)
ax_traj.set_ylim(GRID_Y_MIN, GRID_Y_MAX)
ax_traj.set_aspect("equal"); ax_traj.grid(True, zorder=0)

for obs in obstacles:
    ox, oy, sz = obs["x"], obs["y"], obs["size"]
    ax_traj.add_patch(patches.Rectangle(
        (ox - sz, oy - sz), 2*sz, 2*sz,
        linewidth=1.5, edgecolor="#8B0000", facecolor="#FF6B6B",
        alpha=0.7, zorder=2,
        label="Obstacle" if obs is obstacles[0] else "_nolegend_"))

ax_traj.plot(df["target_x"], df["target_y"],
             "g--", linewidth=1, label="Target path", zorder=3)

drone_path,     = ax_traj.plot([], [], "b-",  linewidth=1.5, label="Drone", zorder=4)
drone_marker,   = ax_traj.plot([], [], "bo",  markersize=8,  zorder=5)
target_marker,  = ax_traj.plot([], [], "g^",  markersize=8,  label="Target", zorder=5)
desired_circle, = ax_traj.plot([], [], "b--", linewidth=1,   label=f"d={DESIRED_DISTANCE}m", zorder=4)

ARROW_LEN = 3.0  # heading arrow length (metres)
yaw_arrow = ax_traj.quiver(
    df["drone_x"].iloc[0], df["drone_y"].iloc[0],
    ARROW_LEN * np.cos(df["drone_yaw"].iloc[0]),
    ARROW_LEN * np.sin(df["drone_yaw"].iloc[0]),
    angles="xy", scale_units="xy", scale=1,
    color="dodgerblue", width=0.005, zorder=6, label="heading")

ax_traj.legend(loc="upper left", fontsize=8)

# ── Panel (0,1): Tracking Error ───────────────────────────────────────────────

ax_err.set_title("Tracking Distance Error")
ax_err.set_xlabel("Time [s]"); ax_err.set_ylabel("Error [m]")
ax_err.axhline(0, color="k", linewidth=0.8, linestyle="--")
ax_err.set_xlim(0, tmax)
ax_err.set_ylim(*_ylim(df["distance_error"]))
ax_err.grid(True)

error_line, = ax_err.plot([], [], "r-", linewidth=1.5)

# ── Panel (1,0): Yaw heading error ───────────────────────────────────────────

ax_yaw.set_title(f"Heading Error  (FoV = {FOV_DEG:.0f}°, target in FoV if |error| < {FOV_DEG/2:.0f}°)")
ax_yaw.set_xlabel("Time [s]"); ax_yaw.set_ylabel("Heading error [rad]")
ax_yaw.set_xlim(0, tmax)
ax_yaw.set_ylim(*_ylim(df["yaw_error"], pad=HALF_FOV + 0.1))
ax_yaw.grid(True)

# Zero reference and ±half-FoV acceptance band (static)
ax_yaw.axhline(0, color="k", linewidth=0.8, linestyle="--", zorder=2)
ax_yaw.fill_between([0, tmax], -HALF_FOV, HALF_FOV,
                    color="green", alpha=0.15, zorder=2)
ax_yaw.axhline( HALF_FOV, color="green", linewidth=1.2,
               label=f"±{FOV_DEG/2:.0f}° FoV boundary", zorder=3)
ax_yaw.axhline(-HALF_FOV, color="green", linewidth=1.2, zorder=3)

yaw_line, = ax_yaw.plot([], [], "b-", linewidth=1.5, label="heading error", zorder=4)
ax_yaw.legend(fontsize=8)

# ── Panel (1,1): Body Rates ───────────────────────────────────────────────────

ax_rates.set_title("Body Rates  (cmd: dashed  |  actual: solid)")
ax_rates.set_xlabel("Time [s]"); ax_rates.set_ylabel("Rate [rad/s]")
ax_rates.set_xlim(0, tmax)
ax_rates.grid(True)

if has_body_rates:
    rate_cols = [df["roll_rate"], df["pitch_rate"], df["yaw_rate"],
                 df["drone_wx"],  df["drone_wy"],   df["drone_wz"]]
    ax_rates.set_ylim(*_ylim(*rate_cols))

    # Static commands (full horizon)
    ax_rates.plot(df["t"], df["roll_rate"],  "r--", linewidth=1.0, label="wx_cmd (roll)")
    ax_rates.plot(df["t"], df["pitch_rate"], "b--", linewidth=1.0, label="wy_cmd (pitch)")
    ax_rates.plot(df["t"], df["yaw_rate"],   "g--", linewidth=1.0, label="wz_cmd (yaw)")

    # Animated actual rates
    wx_line, = ax_rates.plot([], [], "r-", linewidth=1.5, label="wx actual")
    wy_line, = ax_rates.plot([], [], "b-", linewidth=1.5, label="wy actual")
    wz_line, = ax_rates.plot([], [], "g-", linewidth=1.5, label="wz actual")
    ax_rates.legend(fontsize=7, ncol=2)
else:
    _placeholder(ax_rates, "Body rates not in log\n(rebuild C++ and re-run)")
    wx_line = wy_line = wz_line = ax_rates.plot([], [])[0]

# ── Panel (2,0): Velocities ───────────────────────────────────────────────────

ax_vel.set_title("Velocities  (target: dashed reference  |  drone: solid state)")
ax_vel.set_xlabel("Time [s]"); ax_vel.set_ylabel("Velocity [m/s]")
ax_vel.set_xlim(0, tmax)
ax_vel.set_ylim(*_ylim(df["drone_vx"], df["drone_vy"],
                        df["target_vx"], df["target_vy"]))
ax_vel.grid(True)

# Static target velocities
ax_vel.plot(df["t"], df["target_vx"], "r--", linewidth=1.0, label="target vx")
ax_vel.plot(df["t"], df["target_vy"], "b--", linewidth=1.0, label="target vy")

drone_vx_line, = ax_vel.plot([], [], "r-", linewidth=1.5, label="drone vx")
drone_vy_line, = ax_vel.plot([], [], "b-", linewidth=1.5, label="drone vy")
ax_vel.legend(fontsize=8)

# ── Panel (2,1): Roll & Pitch ─────────────────────────────────────────────────

ax_att.set_title("Roll & Pitch  (setpoint: dashed  |  state: solid)")
ax_att.set_xlabel("Time [s]"); ax_att.set_ylabel("Angle [rad]")
ax_att.set_xlim(0, tmax)
ax_att.set_ylim(*_ylim(df["drone_roll"], df["drone_pitch"],
                        df["roll_des"],  df["pitch_des"]))
ax_att.grid(True)

# Static attitude setpoints
ax_att.plot(df["t"], df["roll_des"],  "r--", linewidth=1.0, label="roll setpoint")
ax_att.plot(df["t"], df["pitch_des"], "b--", linewidth=1.0, label="pitch setpoint")

roll_line,  = ax_att.plot([], [], "r-", linewidth=1.5, label="roll state")
pitch_line, = ax_att.plot([], [], "b-", linewidth=1.5, label="pitch state")
ax_att.legend(fontsize=8)

# ── Panel (3,0): Occlusion ────────────────────────────────────────────────────

_placeholder(ax_occ, "Occlusion metric\n(TBD)")
ax_occ.set_title("Occlusion")

# ── Panel (3,1): Reserved ─────────────────────────────────────────────────────

_placeholder(ax_other, "To be defined")
ax_other.set_title("Reserved")

# ── Animation ─────────────────────────────────────────────────────────────────

_animated = (drone_path, drone_marker, target_marker, desired_circle,
             error_line,
             yaw_line,
             wx_line, wy_line, wz_line,
             drone_vx_line, drone_vy_line,
             roll_line, pitch_line,
             yaw_arrow)

def init():
    for artist in _animated[:-1]:   # all except quiver (no set_data)
        artist.set_data([], [])
    yaw_arrow.set_UVC(0, 0)
    return _animated

def update(frame):
    sub = df.iloc[:frame + 1]
    t   = sub["t"]

    # (0,0) Trajectory
    dx = sub["drone_x"].iloc[-1]
    dy = sub["drone_y"].iloc[-1]
    yaw = sub["drone_yaw"].iloc[-1]
    drone_path.set_data(sub["drone_x"], sub["drone_y"])
    drone_marker.set_data([dx], [dy])
    target_marker.set_data([sub["target_x"].iloc[-1]], [sub["target_y"].iloc[-1]])
    theta = np.linspace(0, 2*np.pi, 120)
    desired_circle.set_data(
        sub["target_x"].iloc[-1] + DESIRED_DISTANCE * np.cos(theta),
        sub["target_y"].iloc[-1] + DESIRED_DISTANCE * np.sin(theta))
    yaw_arrow.set_offsets([[dx, dy]])
    yaw_arrow.set_UVC(ARROW_LEN * np.cos(yaw), ARROW_LEN * np.sin(yaw))

    # (0,1) Error
    error_line.set_data(t, sub["distance_error"])

    # (1,0) Yaw heading error
    yaw_line.set_data(t, sub["yaw_error"])

    # (1,1) Body rates
    if has_body_rates:
        wx_line.set_data(t, sub["drone_wx"])
        wy_line.set_data(t, sub["drone_wy"])
        wz_line.set_data(t, sub["drone_wz"])

    # (2,0) Velocities
    drone_vx_line.set_data(t, sub["drone_vx"])
    drone_vy_line.set_data(t, sub["drone_vy"])

    # (2,1) Roll & Pitch
    roll_line.set_data(t,  sub["drone_roll"])
    pitch_line.set_data(t, sub["drone_pitch"])

    return _animated

ani = FuncAnimation(
    fig, update,
    frames=len(df),
    init_func=init,
    interval=max(1, int(cfg["sim"]["dt"] * 1000)),
    blit=True)

plt.tight_layout()
plt.show()
