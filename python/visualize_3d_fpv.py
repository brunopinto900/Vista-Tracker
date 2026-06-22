#!/usr/bin/env python3
"""
Vista-Tracker — PyVista 3-D visualisation.

Layout (single pyvista window, shape 1×2):
  Left:  3-D isometric scene  — obstacles (ESDF-coloured cubes), drone, target,
         FOV pyramid, trajectory trails.
  Right: 2-D top-down scene   — same actors projected to XY, camera frustum shown
         as a flat wedge cone.

Run standalone:
    python3 visualize_3d.py [config.yaml]

Launched automatically by run.sh alongside visualize.py.
"""
from __future__ import annotations

import os
import sys
import time
import numpy as np
import pandas as pd
import yaml
import pyvista as pv

# ── Config loader (mirrors visualize.py) ──────────────────────────────────────

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

# ── Load log ──────────────────────────────────────────────────────────────────

LOG_FILE = os.path.join(os.path.dirname(__file__), "../data/log.csv")
df = pd.read_csv(LOG_FILE)

# ── Constants ─────────────────────────────────────────────────────────────────

obstacles        = cfg.get("world", {}).get("obstacles", [])
DESIRED_DISTANCE = cfg["controller"]["desired_distance"]
CAMERA_RANGE     = cfg.get("camera", {}).get("range", 8.0)
TRACKING_FOV_DEG = cfg.get("tracking_camera", {}).get("fov", 60.0)
TRACKING_HALF_FOV = np.radians(TRACKING_FOV_DEG / 2.0)
SIM_DT           = cfg["sim"]["dt"]

grid             = cfg.get("world", {}).get("grid", {})
GRID_X_MIN = grid.get("x_min", -12.5)
GRID_X_MAX = grid.get("x_max",  12.5)
GRID_Y_MIN = grid.get("y_min", -12.5)
GRID_Y_MAX = grid.get("y_max",  12.5)

DRONE_Z = df["drone_z"].iloc[0] if "drone_z" in df.columns else 2.0
TRAIL_LEN = 80   # max frames kept in trail

# ── ESDF helper ───────────────────────────────────────────────────────────────

def esdf_at(px, py, pz, obs_list):
    """Signed distance from point to nearest obstacle surface (positive = outside)."""
    if not obs_list:
        return float("inf")
    min_d = float("inf")
    for obs in obs_list:
        ox, oy, sz = obs["x"], obs["y"], obs["size"]
        oz = sz  # cube sits on ground; centre at z=sz
        dx = max(abs(px - ox) - sz, 0.0)
        dy = max(abs(py - oy) - sz, 0.0)
        dz = max(abs(pz - oz) - sz, 0.0)
        d = np.sqrt(dx*dx + dy*dy + dz*dz)
        min_d = min(min_d, d)
    return min_d

# ── Colour map for ESDF ───────────────────────────────────────────────────────
# 0 m (surface) → red, 4 m (DESIRED_DISTANCE) → green

def esdf_colour(d, d_max=None):
    if d_max is None:
        d_max = DESIRED_DISTANCE
    t = float(np.clip(d / d_max, 0.0, 1.0))
    r = 1.0 - t
    g = t
    b = 0.0
    return (r, g, b)

# ── Build static obstacle meshes ─────────────────────────────────────────────

def _make_obs_cube(obs):
    ox, oy, sz = obs["x"], obs["y"], obs["size"]
    oz = sz  # centre Z (cube base on ground)
    cube = pv.Box(bounds=(ox-sz, ox+sz, oy-sz, oy+sz, 0.0, 2*sz))
    # ESDF colour: distance from cube surface to itself = 0 → red
    # but we colour each face by the ESDF at the face centre
    centres = cube.cell_centers().points
    esdf_vals = np.array([esdf_at(c[0], c[1], c[2], obstacles) for c in centres])
    cube.cell_data["esdf"] = esdf_vals
    return cube

obs_meshes = [_make_obs_cube(o) for o in obstacles]

# ── Ground plane ──────────────────────────────────────────────────────────────

def _make_ground():
    xs = np.linspace(GRID_X_MIN, GRID_X_MAX, 2)
    ys = np.linspace(GRID_Y_MIN, GRID_Y_MAX, 2)
    xx, yy = np.meshgrid(xs, ys)
    zz = np.zeros_like(xx)
    return pv.StructuredGrid(xx, yy, zz)

ground = _make_ground()

# ── FOV pyramid builder ───────────────────────────────────────────────────────

def _fov_pyramid(dx, dy, dz, yaw, half_fov_rad, length, n_sides=20):
    """Cone/pyramid mesh representing the tracking camera FOV."""
    tip = np.array([dx, dy, dz])
    angles = np.linspace(yaw - half_fov_rad, yaw + half_fov_rad, n_sides)
    base_pts = np.array([
        [dx + length * np.cos(a), dy + length * np.sin(a), dz]
        for a in angles
    ])
    pts = np.vstack([tip, base_pts])
    faces = []
    n = len(base_pts)
    for i in range(n - 1):
        faces += [3, 0, i + 1, i + 2]
    faces += [n] + list(range(1, n + 1))  # base polygon
    return pv.PolyData(pts, faces=np.array(faces))

def _fov_pyramid_2d(dx, dy, yaw, half_fov_rad, length, n_sides=20):
    """Flat (Z=0) wedge for the top-down panel."""
    return _fov_pyramid(dx, dy, 0.0, yaw, half_fov_rad, length, n_sides)

# ── Person body builder ───────────────────────────────────────────────────────

def _person_mesh(x, y, z=0.0, height=1.8, radius=0.3):
    body = pv.Cylinder(center=(x, y, z + height/2), direction=(0,0,1),
                        radius=radius, height=height, resolution=12)
    head = pv.Sphere(center=(x, y, z + height + radius * 0.8), radius=radius * 0.9)
    return body.merge(head)

# ── Drone body builder ────────────────────────────────────────────────────────

def _drone_mesh(x, y, z, yaw, arm=0.4, r=0.12):
    parts = []
    body = pv.Sphere(center=(x, y, z), radius=r)
    parts.append(body)
    for ang in [yaw + np.pi/4, yaw - np.pi/4, yaw + 3*np.pi/4, yaw - 3*np.pi/4]:
        ex = x + arm * np.cos(ang)
        ey = y + arm * np.sin(ang)
        rotor = pv.Disc(center=(ex, ey, z), normal=(0, 0, 1),
                         inner=0.0, outer=r * 1.4, r_res=1, c_res=12)
        arm_mesh = pv.Line((x, y, z), (ex, ey, z))
        parts.append(rotor)
        parts.append(arm_mesh)
    result = parts[0]
    for p in parts[1:]:
        result = result.merge(p)
    return result

# ── Trail builder ─────────────────────────────────────────────────────────────

def _trail_mesh(xs, ys, zs):
    if len(xs) < 2:
        return None
    pts = np.column_stack([xs, ys, zs])
    n = len(pts)
    lines = np.array([[2, i, i+1] for i in range(n-1)]).ravel()
    return pv.PolyData(pts, lines=lines)

# ── Plotter setup ─────────────────────────────────────────────────────────────

pl = pv.Plotter(shape=(1, 2), window_size=(1600, 800),
                title="Vista-Tracker — 3D View")

# ── Left panel: 3-D isometric ─────────────────────────────────────────────────

pl.subplot(0, 0)
pl.add_text("3D Scene", font_size=10, position="upper_left")

# Ground
pl.add_mesh(ground, color="#DDDDDD", opacity=0.4, show_edges=False)

# Obstacle cubes (static, ESDF coloured)
for mesh in obs_meshes:
    if mesh.n_cells > 0:
        pl.add_mesh(mesh, scalars="esdf", cmap="RdYlGn",
                    clim=[0, DESIRED_DISTANCE],
                    show_scalar_bar=False, opacity=0.75)

# Dynamic actors — stored as lists so we can remove/re-add each frame
_actors_3d = {}

def _refresh_actor(panel, key, mesh, **kwargs):
    if key in _actors_3d:
        pl.subplot(0, panel)
        pl.remove_actor(_actors_3d[key], render=False)
    pl.subplot(0, panel)
    if mesh is not None:
        _actors_3d[key] = pl.add_mesh(mesh, **kwargs)

pl.camera_position = [
    (GRID_X_MAX * 1.5, GRID_Y_MIN * 2.0, (GRID_X_MAX - GRID_X_MIN) * 0.8),
    ((GRID_X_MAX + GRID_X_MIN) / 2, (GRID_Y_MAX + GRID_Y_MIN) / 2, 0),
    (0, 0, 1)
]

# ── Right panel: 2-D top-down ─────────────────────────────────────────────────

pl.subplot(0, 1)
pl.add_text("Top-Down (2D)", font_size=10, position="upper_left")
pl.view_xy()
pl.camera.parallel_projection = True
span = max(GRID_X_MAX - GRID_X_MIN, GRID_Y_MAX - GRID_Y_MIN)
pl.camera.parallel_scale = span / 2.0
pl.camera.position = ((GRID_X_MAX + GRID_X_MIN)/2,
                       (GRID_Y_MAX + GRID_Y_MIN)/2,
                       span * 2)
pl.camera.focal_point = ((GRID_X_MAX + GRID_X_MIN)/2,
                          (GRID_Y_MAX + GRID_Y_MIN)/2, 0)

ground2d = _make_ground()
pl.add_mesh(ground2d, color="#DDDDDD", opacity=0.4)

for mesh in obs_meshes:
    if mesh.n_cells > 0:
        pl.add_mesh(mesh, scalars="esdf", cmap="RdYlGn",
                    clim=[0, DESIRED_DISTANCE],
                    show_scalar_bar=False, opacity=0.75)

_actors_2d = {}

def _refresh_actor_2d(key, mesh, **kwargs):
    if key in _actors_2d:
        pl.subplot(0, 1)
        pl.remove_actor(_actors_2d[key], render=False)
    pl.subplot(0, 1)
    if mesh is not None:
        _actors_2d[key] = pl.add_mesh(mesh, **kwargs)

# ── Scalar bar (shared, added once) ──────────────────────────────────────────

pl.subplot(0, 0)
if obs_meshes:
    pl.add_scalar_bar("ESDF [m]", n_labels=3, fmt="%.1f",
                      position_x=0.02, position_y=0.05,
                      height=0.3, width=0.08)

# ── Animation callback ────────────────────────────────────────────────────────

_frame = [0]
_drone_trail_x: list[float] = []
_drone_trail_y: list[float] = []
_drone_trail_z: list[float] = []
_target_trail_x: list[float] = []
_target_trail_y: list[float] = []
_target_trail_z: list[float] = []

def _step():
    i = _frame[0]
    if i >= len(df):
        return

    row = df.iloc[i]
    dx   = float(row["drone_x"])
    dy   = float(row["drone_y"])
    dz   = float(row.get("drone_z", DRONE_Z))
    yaw  = float(row["drone_yaw"])
    tx   = float(row["target_x"])
    ty   = float(row["target_y"])
    tz   = 0.0

    # Trails
    _drone_trail_x.append(dx); _drone_trail_y.append(dy); _drone_trail_z.append(dz)
    _target_trail_x.append(tx); _target_trail_y.append(ty); _target_trail_z.append(tz)
    if len(_drone_trail_x) > TRAIL_LEN:
        _drone_trail_x.pop(0); _drone_trail_y.pop(0); _drone_trail_z.pop(0)
    if len(_target_trail_x) > TRAIL_LEN:
        _target_trail_x.pop(0); _target_trail_y.pop(0); _target_trail_z.pop(0)

    # ── 3D panel ──────────────────────────────────────────────────────────────

    pl.subplot(0, 0)

    drone_m  = _drone_mesh(dx, dy, dz, yaw)
    person_m = _person_mesh(tx, ty, tz)
    fov_m    = _fov_pyramid(dx, dy, dz, yaw, TRACKING_HALF_FOV, CAMERA_RANGE)
    d_trail  = _trail_mesh(_drone_trail_x,  _drone_trail_y,  _drone_trail_z)
    t_trail  = _trail_mesh(_target_trail_x, _target_trail_y, _target_trail_z)

    _refresh_actor(0, "drone",   drone_m,  color="royalblue",   opacity=1.0)
    _refresh_actor(0, "person",  person_m, color="tomato",      opacity=0.9)
    _refresh_actor(0, "fov",     fov_m,    color="lightgreen",  opacity=0.20)
    _refresh_actor(0, "d_trail", d_trail,  color="dodgerblue",  line_width=2, opacity=0.6)
    _refresh_actor(0, "t_trail", t_trail,  color="orangered",   line_width=2, opacity=0.6)

    # ── 2D panel ──────────────────────────────────────────────────────────────

    pl.subplot(0, 1)

    fov_2d   = _fov_pyramid_2d(dx, dy, yaw, TRACKING_HALF_FOV, CAMERA_RANGE)
    d_trail2 = _trail_mesh(_drone_trail_x, _drone_trail_y,
                            [0.01]*len(_drone_trail_x))
    t_trail2 = _trail_mesh(_target_trail_x, _target_trail_y,
                            [0.01]*len(_target_trail_x))

    drone_dot  = pv.Sphere(center=(dx, dy, 0.02), radius=0.5)
    target_dot = pv.Sphere(center=(tx, ty, 0.02), radius=0.4)

    _refresh_actor_2d("drone2d",  drone_dot,  color="royalblue",  opacity=1.0)
    _refresh_actor_2d("target2d", target_dot, color="tomato",     opacity=1.0)
    _refresh_actor_2d("fov2d",    fov_2d,     color="lightgreen", opacity=0.25)
    _refresh_actor_2d("dtr2d",    d_trail2,   color="dodgerblue", line_width=2, opacity=0.6)
    _refresh_actor_2d("ttr2d",    t_trail2,   color="orangered",  line_width=2, opacity=0.6)

    pl.render()
    _frame[0] += 1

# ── Run ───────────────────────────────────────────────────────────────────────

interval_ms = max(1, int(SIM_DT * 1000))

pl.show(auto_close=False, interactive_update=True)

try:
    while True:
        _step()
        pl.update(interval_ms)
except Exception:
    pass
finally:
    pl.close()
