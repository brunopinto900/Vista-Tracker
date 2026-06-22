#!/usr/bin/env python3
"""
Vista-Tracker visualiser  –  4 × 2 animated dashboard.

Panels:
  [0,0]  2-D trajectory with obstacles and desired-distance circle
  [0,1]  Altitude: drone height vs desired/reference height
  [1,1]  Lateral tracking error (drone − target in X and Y)
  [2,0]  Yaw: reference, state, and ±half-FoV acceptance band
  [2,1]  Body rates: controller commands vs actual plant response
  [3,0]  Velocities: drone state vs target (reference)
  [3,1]  Roll & pitch: attitude setpoints vs state
  [4,0]  Camera pitch: reference, state, and ±V-FoV acceptance band
  [4,1]  FOV occlusion by obstacles (angular, deg)
  Deadlock shown as text overlay on the XY trajectory plot.

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
from matplotlib.gridspec import GridSpec

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
DESIRED_DISTANCE    = cfg["controller"]["desired_distance"]
ATT_KP              = cfg["controller"]["attitude_kp"]
FOV_DEG             = cfg.get("camera", {}).get("fov", 360.0)
CAMERA_RANGE        = cfg.get("camera", {}).get("range", 6.0)
TRACKING_FOV_DEG    = cfg.get("tracking_camera", {}).get("fov", 60.0)
TRACKING_HALF_FOV   = np.radians(TRACKING_FOV_DEG / 2.0)
TRACKING_HALF_VFOV  = np.radians(cfg.get("tracking_camera", {}).get("vfov", 18.0))
PERSON_TRACK_Z      = cfg.get("target", {}).get("track_z", 1.40)

GRID_X_MIN = grid.get("x_min", -12.5)
GRID_X_MAX = grid.get("x_max",  12.5)
GRID_Y_MIN = grid.get("y_min", -12.5)
GRID_Y_MAX = grid.get("y_max",  12.5)

tmax = max(df["t"].max(), 1e-3)

_vfov_deg = np.degrees(TRACKING_HALF_VFOV)

# ── Derived signals ───────────────────────────────────────────────────────────

_drone_z_arr = df["drone_z"].to_numpy() if "drone_z" in df.columns else np.full(len(df), 2.0)
df["distance"]       = np.sqrt((df["target_x"] - df["drone_x"])**2 +
                                (df["target_y"] - df["drone_y"])**2 +
                                (PERSON_TRACK_Z - _drone_z_arr)**2)
df["distance_error"] = df["distance"] - DESIRED_DISTANCE

df["x_error"]    = df["drone_x"] - df["target_x"]
df["y_error"]    = df["drone_y"] - df["target_y"]
df["xy_distance"] = np.sqrt(df["x_error"]**2 + df["y_error"]**2)

df["ref_yaw"] = np.arctan2(df["target_y"] - df["drone_y"],
                            df["target_x"] - df["drone_x"])

df["yaw_error"] = np.arctan2(
    np.sin(df["drone_yaw"] - df["ref_yaw"]),
    np.cos(df["drone_yaw"] - df["ref_yaw"]))

df["roll_des"]  = df["drone_roll"]  + df["roll_rate"]  / ATT_KP
df["pitch_des"] = df["drone_pitch"] + df["pitch_rate"] / ATT_KP

has_body_rates  = "drone_wx" in df.columns
has_vel_ref     = "vel_ref_x" in df.columns
has_ref_pos     = "ref_x" in df.columns
has_deadlock    = "deadlock_active" in df.columns
has_cam_pitch   = "ref_camera_pitch" in df.columns
has_ref_z       = "ref_z" in df.columns

if has_cam_pitch:
    df["pitch_cam_error"] = np.arctan2(
        np.sin(df["drone_pitch"] - df["ref_camera_pitch"]),
        np.cos(df["drone_pitch"] - df["ref_camera_pitch"]))

# ── Occlusion metric (eq. 4.28–4.29) ─────────────────────────────────────────

D_FOV = DESIRED_DISTANCE

def _los_point_dist(ox, oy, px, py, tx, ty):
    dx, dy  = tx - px, ty - py
    len_sq  = dx*dx + dy*dy
    safe    = np.where(len_sq < 1e-12, 1.0, len_sq)
    t_raw   = ((ox - px)*dx + (oy - py)*dy) / safe
    in_seg  = (t_raw > 0.0) & (t_raw < 1.0) & (len_sq >= 1e-12)
    t_clip  = np.clip(t_raw, 0.0, 1.0)
    d       = np.hypot(ox - (px + t_clip*dx), oy - (py + t_clip*dy))
    return np.where(in_seg, d, np.inf)

def _compute_occlusion_deg(drone_x, drone_y, target_x, target_y, obs_list, fov_deg, d_fov):
    if not obs_list:
        return np.zeros(len(drone_x))
    dists = np.stack([
        np.maximum(0.0,
            _los_point_dist(obs["x"], obs["y"], drone_x, drone_y, target_x, target_y)
            - obs["size"])
        for obs in obs_list
    ])
    d_min = dists.min(axis=0)
    return fov_deg * np.maximum(0.0, 1.0 - d_min / d_fov)

df["occlusion_deg"] = _compute_occlusion_deg(
    df["drone_x"].to_numpy(), df["drone_y"].to_numpy(),
    df["target_x"].to_numpy(), df["target_y"].to_numpy(),
    obstacles, TRACKING_FOV_DEG, D_FOV)

# Out-of-FOV: check yaw error against H-FOV and pitch error against V-FOV.
# Mirrors how the yaw and camera-pitch subplots define "in-frame".
_oof_horiz = np.abs(df["yaw_error"].to_numpy()) > TRACKING_HALF_FOV
_oof_vert  = (np.abs(df["pitch_cam_error"].to_numpy()) > TRACKING_HALF_VFOV
              if has_cam_pitch else np.zeros(len(df), dtype=bool))
_oof_arr   = _oof_horiz | _oof_vert
df["out_of_fov"] = _oof_arr
_dt_sim  = cfg["sim"]["dt"]
_fov_dur = np.zeros(len(_oof_arr))
_acc     = 0.0
for _i, _oof in enumerate(_oof_arr):
    _acc = (_acc + _dt_sim) if _oof else 0.0
    _fov_dur[_i] = _acc
df["fov_loss_duration"] = _fov_dur

# ── Helper ────────────────────────────────────────────────────────────────────

def _esdf_color(dist_to_drone, obs_size):
    d_surf = max(0.0, dist_to_drone - obs_size)
    t = min(1.0, d_surf / DESIRED_DISTANCE)
    return (1.0 - t, 0.1 + t * 0.75, 0.05)

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
# 5 equal rows × 2 cols; XY spans rows 0–1 (left), right col has 5 uniform panels.
#
#  col 0 (left)       col 1 (right)
#  ┌──────────────┐   ┌─────────────┐  row 0
#  │              │   │  altitude   │
#  │  2-D traj    │   ├─────────────┤  row 1
#  │              │   │  x,y error  │
#  ├──────────────┤   ├─────────────┤  row 2
#  │  yaw+pitch   │   │  body rates │
#  ├──────────────┤   ├─────────────┤  row 3
#  │  velocities  │   │  roll/pitch │
#  ├──────────────┤   ├─────────────┤  row 4
#  │  cam pitch   │   │  occlusion  │
#  └──────────────┘   └─────────────┘
# Deadlock: text overlay on the XY plot only.

fig = plt.figure(figsize=(16, 22))
gs  = GridSpec(5, 2, figure=fig, hspace=0.38, wspace=0.28)
fig.suptitle("Vista-Tracker — Simulation Dashboard", fontsize=13, y=0.998)

ax_traj  = fig.add_subplot(gs[0:2, 0])
ax_alt   = fig.add_subplot(gs[0,   1])
ax_err   = fig.add_subplot(gs[1,   1])
ax_yaw   = fig.add_subplot(gs[2,   0])
ax_rates = fig.add_subplot(gs[2,   1])
ax_vel   = fig.add_subplot(gs[3,   0])
ax_att   = fig.add_subplot(gs[3,   1])
ax_pitch = fig.add_subplot(gs[4,   0])  # Camera pitch
ax_occ   = fig.add_subplot(gs[4,   1])  # Occlusion (moved from left)

# ── Panel (0,1): Altitude ─────────────────────────────────────────────────────

ax_alt.set_title("Altitude  (drone vs desired)")
ax_alt.set_xlabel("Time [s]"); ax_alt.set_ylabel("Height [m]")
ax_alt.set_xlim(0, tmax)
ax_alt.grid(True)

_alt_cols = [_drone_z_arr]
if has_ref_z:
    _alt_cols.append(df["ref_z"].to_numpy())
ax_alt.set_ylim(*_ylim(*[pd.Series(c) for c in _alt_cols], pad=0.3))

if has_ref_z:
    ax_alt.plot(df["t"], df["ref_z"], color="orange", linewidth=1.0,
                linestyle="--", label="desired altitude", zorder=3)
ax_alt.axhline(0, color="k", linewidth=0.5, linestyle=":", zorder=2)
alt_drone_line, = ax_alt.plot([], [], color="steelblue", linewidth=1.5,
                               label="drone z", zorder=4)
ax_alt.legend(fontsize=8)

# ── Panel (0,0): 2-D Trajectory ───────────────────────────────────────────────

ax_traj.set_title("2-D Trajectory")
ax_traj.set_xlabel("X [m]"); ax_traj.set_ylabel("Y [m]")
ax_traj.set_xlim(GRID_X_MIN, GRID_X_MAX)
ax_traj.set_ylim(GRID_Y_MIN, GRID_Y_MAX)
ax_traj.set_aspect("equal"); ax_traj.grid(True, zorder=0)

_yaw0 = df["drone_yaw"].iloc[0]
fov_wedge = patches.Wedge(
    (df["drone_x"].iloc[0], df["drone_y"].iloc[0]),
    CAMERA_RANGE,
    np.degrees(_yaw0 - TRACKING_HALF_FOV),
    np.degrees(_yaw0 + TRACKING_HALF_FOV),
    facecolor="lightgreen", alpha=0.22, edgecolor="green",
    linewidth=1, linestyle="--", zorder=1, animated=True,
    label=f"Tracking FoV {TRACKING_FOV_DEG:.0f}°")
ax_traj.add_patch(fov_wedge)

obs_patches = []
for i, obs in enumerate(obstacles):
    ox, oy, sz = obs["x"], obs["y"], obs["size"]
    p = patches.Rectangle(
        (ox - sz, oy - sz), 2*sz, 2*sz,
        linewidth=1.5, edgecolor="#888888", facecolor="#CCCCCC",
        alpha=0.5, zorder=2, animated=True,
        label="Obstacle" if i == 0 else "_nolegend_")
    ax_traj.add_patch(p)
    obs_patches.append(p)

ax_traj.plot(df["target_x"], df["target_y"],
             "g--", linewidth=1, label="Target path", zorder=3)

if has_ref_pos:
    ax_traj.plot(df["ref_x"], df["ref_y"],
                 color="darkorange", linewidth=1, linestyle=":",
                 label="Desired ref path", zorder=3)

drone_path,     = ax_traj.plot([], [], "b-",  linewidth=1.5, label="Drone", zorder=4)
drone_marker,   = ax_traj.plot([], [], "bo",  markersize=8,  zorder=5)
target_marker,  = ax_traj.plot([], [], "g^",  markersize=8,  label="Target", zorder=5)
desired_circle, = ax_traj.plot([], [], "b--", linewidth=1,   label=f"d={DESIRED_DISTANCE}m", zorder=4)

if has_ref_pos:
    ref_marker, = ax_traj.plot([], [], "D", color="darkorange", markersize=7,
                                label="Desired ref", zorder=6)
else:
    ref_marker, = ax_traj.plot([], [], [])

ARROW_LEN = 3.0
yaw_arrow = ax_traj.quiver(
    df["drone_x"].iloc[0], df["drone_y"].iloc[0],
    ARROW_LEN * np.cos(df["drone_yaw"].iloc[0]),
    ARROW_LEN * np.sin(df["drone_yaw"].iloc[0]),
    angles="xy", scale_units="xy", scale=1,
    color="dodgerblue", width=0.005, zorder=6, label="heading")

ax_traj.legend(loc="upper left", fontsize=8)

deadlock_text = ax_traj.text(
    0.02, 0.97, "", transform=ax_traj.transAxes,
    ha="left", va="top", fontsize=9, fontweight="bold",
    color="red", bbox=dict(facecolor="white", edgecolor="red",
                           alpha=0.85, boxstyle="round,pad=0.3"),
    zorder=10, animated=True, visible=False)

# ── Panel (1,1): XY Tracking Distance ────────────────────────────────────────

ax_err.set_title("XY Tracking Distance  (drone − target)")
ax_err.set_xlabel("Time [s]"); ax_err.set_ylabel("Distance [m]")
ax_err.axhline(DESIRED_DISTANCE, color="orange", linewidth=1.2, linestyle="--",
               label=f"desired {DESIRED_DISTANCE:.1f} m")
ax_err.set_xlim(0, tmax)
ax_err.set_ylim(0, max(df["xy_distance"].max(), DESIRED_DISTANCE) + 0.5)
ax_err.grid(True)

xy_dist_line, = ax_err.plot([], [], color="steelblue", linewidth=1.5, label="XY distance")
ax_err.legend(fontsize=8)

# ── Panel (2,0): Yaw ─────────────────────────────────────────────────────────

ax_yaw.set_title(f"Yaw  (tracking FoV ±{TRACKING_FOV_DEG/2:.0f}°, target in FoV if |error| < {TRACKING_FOV_DEG/2:.0f}°)")
ax_yaw.set_xlabel("Time [s]"); ax_yaw.set_ylabel("Angle / Error [rad]")
ax_yaw.set_xlim(0, tmax)
ax_yaw.set_ylim(*_ylim(df["yaw_error"], df["ref_yaw"], df["drone_yaw"], pad=0.2))
ax_yaw.grid(True)

ax_yaw.fill_between([0, tmax], -TRACKING_HALF_FOV, TRACKING_HALF_FOV,
                    color="lightgreen", alpha=0.25, zorder=1)
ax_yaw.axhline( TRACKING_HALF_FOV, color="green", linewidth=1.0, linestyle=":",
               label=f"±{TRACKING_FOV_DEG/2:.0f}° tracking FoV", zorder=2)
ax_yaw.axhline(-TRACKING_HALF_FOV, color="green", linewidth=1.0, linestyle=":", zorder=2)
ax_yaw.axhline(0, color="k", linewidth=0.6, linestyle="--", zorder=2)

ax_yaw.plot(df["t"], df["ref_yaw"], color="orange", linewidth=1.0,
            linestyle="--", label="yaw ref", zorder=3)

yaw_state_line, = ax_yaw.plot([], [], color="steelblue", linewidth=1.5,
                               label="drone yaw", zorder=4)
yaw_line,       = ax_yaw.plot([], [], color="crimson",   linewidth=1.5,
                               label="yaw error", zorder=5)
ax_yaw.legend(fontsize=8)

# ── Panel (2,1): Body Rates ───────────────────────────────────────────────────

ax_rates.set_title("Body Rates  (cmd: dashed  |  actual: solid)")
ax_rates.set_xlabel("Time [s]"); ax_rates.set_ylabel("Rate [rad/s]")
ax_rates.set_xlim(0, tmax)
ax_rates.grid(True)

if has_body_rates:
    rate_cols = [df["roll_rate"], df["pitch_rate"], df["yaw_rate"],
                 df["drone_wx"],  df["drone_wy"],   df["drone_wz"]]
    ax_rates.set_ylim(*_ylim(*rate_cols))
    ax_rates.plot(df["t"], df["roll_rate"],  "r--", linewidth=1.0, label="wx_cmd (roll)")
    ax_rates.plot(df["t"], df["pitch_rate"], "b--", linewidth=1.0, label="wy_cmd (pitch)")
    ax_rates.plot(df["t"], df["yaw_rate"],   "g--", linewidth=1.0, label="wz_cmd (yaw)")
    wx_line, = ax_rates.plot([], [], "r-", linewidth=1.5, label="wx actual")
    wy_line, = ax_rates.plot([], [], "b-", linewidth=1.5, label="wy actual")
    wz_line, = ax_rates.plot([], [], "g-", linewidth=1.5, label="wz actual")
    ax_rates.legend(fontsize=7, ncol=2)
else:
    _placeholder(ax_rates, "Body rates not in log\n(rebuild C++ and re-run)")
    wx_line = wy_line = wz_line = ax_rates.plot([], [])[0]

# ── Panel (3,0): Velocities ───────────────────────────────────────────────────

ax_vel.set_title("Velocities  (target: dashed  |  vel ref: dash-dot  |  drone: solid)")
ax_vel.set_xlabel("Time [s]"); ax_vel.set_ylabel("Velocity [m/s]")
ax_vel.set_xlim(0, tmax)
_vel_cols = [df["drone_vx"], df["drone_vy"], df["target_vx"], df["target_vy"]]
if has_vel_ref:
    _vel_cols += [df["vel_ref_x"], df["vel_ref_y"]]
ax_vel.set_ylim(*_ylim(*_vel_cols))
ax_vel.grid(True)

ax_vel.plot(df["t"], df["target_vx"], "r--", linewidth=1.0, label="target vx")
ax_vel.plot(df["t"], df["target_vy"], "b--", linewidth=1.0, label="target vy")

if has_vel_ref:
    ax_vel.plot(df["t"], df["vel_ref_x"], color="tomato",        linewidth=1.0,
                linestyle="-.", label="vel ref x")
    ax_vel.plot(df["t"], df["vel_ref_y"], color="cornflowerblue", linewidth=1.0,
                linestyle="-.", label="vel ref y")

drone_vx_line, = ax_vel.plot([], [], "r-", linewidth=1.5, label="drone vx")
drone_vy_line, = ax_vel.plot([], [], "b-", linewidth=1.5, label="drone vy")
ax_vel.legend(fontsize=8)

# ── Panel (3,1): Roll & Pitch ─────────────────────────────────────────────────

ax_att.set_title("Roll & Pitch  (setpoint: dashed  |  state: solid)")
ax_att.set_xlabel("Time [s]"); ax_att.set_ylabel("Angle [rad]")
ax_att.set_xlim(0, tmax)
ax_att.set_ylim(*_ylim(df["drone_roll"], df["drone_pitch"],
                        df["roll_des"],  df["pitch_des"]))
ax_att.grid(True)

ax_att.plot(df["t"], df["roll_des"],  "r--", linewidth=1.0, label="roll setpoint")
ax_att.plot(df["t"], df["pitch_des"], "b--", linewidth=1.0, label="pitch setpoint")

roll_line,  = ax_att.plot([], [], "r-", linewidth=1.5, label="roll state")
pitch_line, = ax_att.plot([], [], "b-", linewidth=1.5, label="pitch state")
ax_att.legend(fontsize=8)

# ── Panel (4,0): Camera Pitch ─────────────────────────────────────────────────

ax_pitch.set_title(f"Camera Pitch  (V-FoV ±{_vfov_deg:.1f}°, target in FoV if |error| < {_vfov_deg:.1f}°)")
ax_pitch.set_xlabel("Time [s]"); ax_pitch.set_ylabel("Angle / Error [rad]")
ax_pitch.set_xlim(0, tmax)
ax_pitch.grid(True)

if has_cam_pitch:
    _pitch_cols = [df["ref_camera_pitch"], df["drone_pitch"], df["pitch_cam_error"]]
    ax_pitch.set_ylim(*_ylim(*_pitch_cols, pad=0.2))

    ax_pitch.fill_between([0, tmax], -TRACKING_HALF_VFOV, TRACKING_HALF_VFOV,
                          color="lightgreen", alpha=0.30, zorder=1)
    ax_pitch.axhline( TRACKING_HALF_VFOV, color="green", linewidth=1.0, linestyle=":",
                     label=f"±{_vfov_deg:.1f}° V-FoV", zorder=2)
    ax_pitch.axhline(-TRACKING_HALF_VFOV, color="green", linewidth=1.0, linestyle=":", zorder=2)
    ax_pitch.axhline(0, color="k", linewidth=0.6, linestyle="--", zorder=2)

    ax_pitch.plot(df["t"], df["ref_camera_pitch"], color="orange", linewidth=1.0,
                  linestyle="--", label="cam pitch ref", zorder=3)

    cam_pitch_state_line, = ax_pitch.plot([], [], color="steelblue", linewidth=1.5,
                                           label="drone pitch", zorder=4)
    cam_pitch_err_line,   = ax_pitch.plot([], [], color="crimson",   linewidth=1.5,
                                           label="pitch error", zorder=5)
    ax_pitch.legend(fontsize=8)
else:
    _placeholder(ax_pitch, "Camera pitch not in log\n(rebuild C++ and re-run)")
    cam_pitch_state_line, = ax_pitch.plot([], [])
    cam_pitch_err_line,   = ax_pitch.plot([], [])

# ── Panel (4,1): Occlusion ────────────────────────────────────────────────────

ax_occ.set_title(f"LOS Occlusion  (tracking FoV = {TRACKING_FOV_DEG:.0f}°)")
ax_occ.set_xlabel("Time [s]"); ax_occ.set_ylabel("Occluded angle [deg]")
ax_occ.set_xlim(0, tmax)
ax_occ.set_ylim(0, TRACKING_FOV_DEG * 1.05)
ax_occ.grid(True, zorder=0)

ax_occ.fill_between([0, tmax], 0, TRACKING_FOV_DEG,
                    color="lightgreen", alpha=0.25, zorder=1,
                    label=f"tracking FoV ±{TRACKING_FOV_DEG/2:.0f}°")
ax_occ.axhline(TRACKING_FOV_DEG, color="green", linewidth=1.0, linestyle="--", zorder=2)

occ_line, = ax_occ.plot([], [], color="darkorange", linewidth=1.5,
                         label="occluded angle", zorder=3)

ax_occ.fill_between(df["t"], 0, TRACKING_FOV_DEG * 1.05,
                    where=_oof_arr, color="lightcoral", alpha=0.30, zorder=2,
                    label="target out of FoV")
ax_occ.legend(fontsize=8)

fov_loss_text = ax_occ.text(
    0.98, 0.97, "", transform=ax_occ.transAxes,
    ha="right", va="top", fontsize=9, fontweight="bold",
    color="darkred", bbox=dict(facecolor="mistyrose", edgecolor="red",
                               alpha=0.85, boxstyle="round,pad=0.3"),
    zorder=10, animated=True, visible=False)

# ── Animation ─────────────────────────────────────────────────────────────────

_anim_lines = (drone_path, drone_marker, target_marker, desired_circle,
               ref_marker,
               alt_drone_line,
               xy_dist_line,
               yaw_state_line, yaw_line,
               cam_pitch_state_line, cam_pitch_err_line,
               wx_line, wy_line, wz_line,
               drone_vx_line, drone_vy_line,
               roll_line, pitch_line,
               occ_line)

_anim_patches = tuple(obs_patches) + (fov_wedge,)
_animated     = _anim_lines + _anim_patches + (yaw_arrow, deadlock_text, fov_loss_text)

def init():
    for line in _anim_lines:
        line.set_data([], [])
    yaw_arrow.set_UVC(0, 0)
    deadlock_text.set_visible(False)
    fov_loss_text.set_visible(False)
    return _animated

def update(frame):
    sub = df.iloc[:frame + 1]
    t   = sub["t"]

    # (0,0) Trajectory
    dx  = sub["drone_x"].iloc[-1]
    dy  = sub["drone_y"].iloc[-1]
    yaw = sub["drone_yaw"].iloc[-1]
    drone_path.set_data(sub["drone_x"], sub["drone_y"])
    drone_marker.set_data([dx], [dy])
    target_marker.set_data([sub["target_x"].iloc[-1]], [sub["target_y"].iloc[-1]])
    if has_ref_pos:
        ref_marker.set_data([sub["ref_x"].iloc[-1]], [sub["ref_y"].iloc[-1]])
    theta = np.linspace(0, 2*np.pi, 120)
    desired_circle.set_data(
        sub["target_x"].iloc[-1] + DESIRED_DISTANCE * np.cos(theta),
        sub["target_y"].iloc[-1] + DESIRED_DISTANCE * np.sin(theta))
    yaw_arrow.set_offsets([[dx, dy]])
    yaw_arrow.set_UVC(ARROW_LEN * np.cos(yaw), ARROW_LEN * np.sin(yaw))

    fov_wedge.set_center((dx, dy))
    fov_wedge.set_theta1(np.degrees(yaw - TRACKING_HALF_FOV))
    fov_wedge.set_theta2(np.degrees(yaw + TRACKING_HALF_FOV))

    for patch, obs in zip(obs_patches, obstacles):
        dist = np.hypot(dx - obs["x"], dy - obs["y"])
        if max(0.0, dist - obs["size"]) <= CAMERA_RANGE:
            r, g, b = _esdf_color(dist, obs["size"])
            patch.set_facecolor((r, g, b))
            patch.set_edgecolor((r * 0.55, g * 0.45, b))
            patch.set_alpha(0.82)
        else:
            patch.set_facecolor("#CCCCCC"); patch.set_edgecolor("#888888"); patch.set_alpha(0.5)

    # (0,1) Altitude
    alt_drone_line.set_data(t, _drone_z_arr[:frame + 1])

    # (1,1) XY tracking distance
    xy_dist_line.set_data(t, sub["xy_distance"])

    # (2,0) Yaw
    yaw_state_line.set_data(t, sub["drone_yaw"])
    yaw_line.set_data(t, sub["yaw_error"])

    # (2,1) Body rates
    if has_body_rates:
        wx_line.set_data(t, sub["drone_wx"])
        wy_line.set_data(t, sub["drone_wy"])
        wz_line.set_data(t, sub["drone_wz"])

    # (3,0) Velocities
    drone_vx_line.set_data(t, sub["drone_vx"])
    drone_vy_line.set_data(t, sub["drone_vy"])

    # (3,1) Roll & Pitch
    roll_line.set_data(t,  sub["drone_roll"])
    pitch_line.set_data(t, sub["drone_pitch"])

    # (4,0) Camera pitch
    if has_cam_pitch:
        cam_pitch_state_line.set_data(t, sub["drone_pitch"])
        cam_pitch_err_line.set_data(t, sub["pitch_cam_error"])

    # (4,1) Occlusion
    occ_line.set_data(t, sub["occlusion_deg"])
    if bool(sub["out_of_fov"].iloc[-1]):
        dur = sub["fov_loss_duration"].iloc[-1]
        fov_loss_text.set_text(f"TARGET OUT OF FoV\n{dur:.1f}s")
        fov_loss_text.set_visible(True)
    else:
        fov_loss_text.set_visible(False)

    # Deadlock: text overlay in XY plot only
    if has_deadlock:
        active = bool(sub["deadlock_active"].iloc[-1])
        if active:
            angle_deg = np.degrees(sub["deadlock_angle"].iloc[-1])
            deadlock_text.set_text(
                f"DEADLOCK AVOIDANCE ACTIVE\nviewpoint angle: {angle_deg:.1f}°")
            deadlock_text.set_visible(True)
        else:
            deadlock_text.set_visible(False)

    return _animated

ani = FuncAnimation(
    fig, update,
    frames=len(df),
    init_func=init,
    interval=max(1, int(cfg["sim"]["dt"] * 1000)),
    blit=True)

plt.tight_layout()
plt.show()
