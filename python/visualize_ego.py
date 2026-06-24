#!/usr/bin/env python3
"""
Vista-Tracker — Vispy chase camera + mini-map + drone-cam PiP.

3-D panel  : third-person chase camera following the drone through the world.
             Drone = compact X-config quadrotor (arms + rotor circles).
             Person = cylinder + sphere head.
             Obstacles = ESDF-coloured cubes.
             Drone trail (blue) + target trail (orange).
             Camera frustum wireframe (green).

Mini-map   : top-down 2-D overlay (bottom-left).

Drone-cam  : PiP overlay (bottom-right) — what the tracking camera sees.
             Uses exact TurntableCamera inversion for any drone position.
"""
from __future__ import annotations

import os
import sys
import numpy as np
import pandas as pd
import yaml

from vispy import app, scene
from vispy.scene import visuals, transforms, widgets
from vispy.geometry import create_cylinder, create_sphere

# ── Config loading ─────────────────────────────────────────────────────────────

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

LOG_FILE = os.path.join(os.path.dirname(__file__), "../data/log.csv")
df = pd.read_csv(LOG_FILE)

# ── Constants ──────────────────────────────────────────────────────────────────

obstacles         = cfg.get("world", {}).get("obstacles", [])
CAMERA_RANGE      = cfg.get("camera", {}).get("range", 8.0)
TRACKING_FOV_DEG  = cfg.get("tracking_camera", {}).get("fov", 60.0)
TRACKING_HALF_FOV  = np.radians(TRACKING_FOV_DEG / 2.0)
TRACKING_HALF_VFOV = np.arctan(np.tan(TRACKING_HALF_FOV) * 9.0 / 16.0)  # true vertical half-FOV, 16:9
_PIP_VFOV_DEG = float(np.degrees(2.0 * TRACKING_HALF_VFOV))  # ≈36° VFOV_full for 60° HFOV / 16:9
ATT_KP             = cfg.get("controller", {}).get("attitude_kp", 6.680)
SIM_DT             = cfg["sim"]["dt"]
TRAIL_LEN          = 120
_target_cfg    = cfg.get("target", {})
PERSON_HEIGHT  = _target_cfg.get("height",  1.80)  # m — person total height
PERSON_WIDTH   = _target_cfg.get("width",   0.50)  # m — shoulder width
PERSON_TRACK_Z = _target_cfg.get("track_z", 0.90)  # m — camera aim point (vertical centre)

# Minimum standoff so the full bounding box [0, PERSON_HEIGHT] fits within the
# usable VFOV half-angle at the altitude floor — mirrors computeStandoffMin() in main.cpp.
def _compute_standoff_min(min_z, h_aim, h_top, phi_rad):
    tanp = np.tan(phi_rad)
    def _large_root(A, B, C):
        disc = B*B - 4*A*C
        return (-B + np.sqrt(max(disc, 0.0))) / (2*A)
    r_head = _large_root(tanp, -(h_top - h_aim), tanp * (min_z - h_aim) * (min_z - h_top))
    r_feet = _large_root(tanp, -h_aim,            tanp * min_z * (min_z - h_aim))
    return max(r_head, r_feet)

_planner_cfg     = cfg.get("planner", {})
_min_z           = _planner_cfg.get("min_z", 2.0)
_theta_safe_rad  = np.radians(_planner_cfg.get("theta_safe", 3.0))
_phi_rad         = TRACKING_HALF_VFOV - _theta_safe_rad
DESIRED_DISTANCE = _compute_standoff_min(_min_z, PERSON_TRACK_Z, PERSON_HEIGHT, _phi_rad)

grid = cfg.get("world", {}).get("grid", {})
GRID_X_MIN = grid.get("x_min", -12.5)
GRID_X_MAX = grid.get("x_max",  12.5)
GRID_Y_MIN = grid.get("y_min", -12.5)
GRID_Y_MAX = grid.get("y_max",  12.5)

# ── Camera tuning (edit these) ────────────────────────────────────────────────
CHASE_DIST    =  9.0   # metres behind drone — close enough to feel dynamic
CHASE_EL      = 32.0   # elevation above drone (°) — clears ~7 m buildings; 9 m centre block still clips briefly
CAM_SMOOTH    = 1.0 #0.15   # position follow speed  (0 = frozen … 1 = instant)
CAM_SMOOTH_AZ = 1.0 #0.10   # heading follow speed   (0 = frozen … 1 = instant)

# Layout
W, H       = 1040, 760
MINI_W, MINI_H = 220, 220   # mini-map overlay (bottom-left)
PIP_W, PIP_H   = 280, 158   # drone-cam PiP overlay — 16:9 to match tracking camera sensor
_PIP_X0 = W - PIP_W - 10   # = 750
_PIP_Y0 = H - PIP_H - 10   # = 550

# Drone icon sizes
_ARM   = 0.3   # arm length from body centre to rotor hub (m)
_R_ROT = 0.11   # rotor display radius (m)

# ── Helpers ────────────────────────────────────────────────────────────────────

def _esdf_color(drone_x, drone_y, obs):
    d_surf = max(0.0, np.hypot(drone_x - obs["x"], drone_y - obs["y"]) - obs["size"])
    t = min(1.0, d_surf / DESIRED_DISTANCE)
    return (1.0 - t, 0.1 + t * 0.75, 0.05, 0.88)

def _rotor_circle(cx, cy, r=0.55, n=10):
    a = np.linspace(0, 2 * np.pi, n + 1)
    return np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a), np.zeros(n + 1)])

def _frustum_pts(dx, dy, dz, yaw, pitch, half_h, half_v, length):
    """NaN-separated line segments forming a pitched camera frustum pyramid.

    pitch > 0 means camera tilts downward (positive = looking down).
    """
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    cos_p, sin_p = np.cos(pitch), np.sin(pitch)
    fw = np.array([cos_y * cos_p, sin_y * cos_p, -sin_p])   # forward (pitched down)
    rv = np.array([sin_y, -cos_y, 0.0])                      # right (horizontal)
    uv = np.cross(rv, fw); uv /= np.linalg.norm(uv)          # up (perp to rv & fw)
    hw = length * np.tan(half_h)
    hh = length * np.tan(half_v)
    tip = np.array([dx, dy, dz])
    fc  = tip + fw * length
    tl  = (fc - hw * rv + hh * uv).tolist()
    tr  = (fc + hw * rv + hh * uv).tolist()
    bl  = (fc - hw * rv - hh * uv).tolist()
    br  = (fc + hw * rv - hh * uv).tolist()
    ap  = [dx, dy, dz]
    n   = [np.nan, np.nan, np.nan]
    return np.array([
        ap, tl, n, ap, tr, n, ap, bl, n, ap, br, n,
        tl, tr, n, tr, br, n, br, bl, n, bl, tl, n,
    ], dtype=np.float32)

def _project_to_pip(pt3d):
    """
    Project a 3-D world point to PiP canvas pixel coordinates.

    Reads the current _pip_cam state (must be called after _pip_cam is updated).
    Returns (px, py) in canvas pixels, or None if the point is behind the camera.
    The PiP viewport occupies x ∈ [_PIP_X0, _PIP_X0+PIP_W],
                               y ∈ [_PIP_Y0, _PIP_Y0+PIP_H] (y=0 at top).
    """
    az   = np.radians(float(_pip_cam.azimuth))
    el   = np.radians(float(_pip_cam.elevation))
    cc   = _pip_cam.center
    cx, cy, cz = float(cc[0]), float(cc[1]), float(cc[2])
    dist = max(float(_pip_cam.distance), 0.01)

    # Eye position: vispy TurntableCamera formula
    eye = np.array([
        cx + dist * np.sin(az) * np.cos(el),
        cy - dist * np.cos(az) * np.cos(el),
        cz + dist * np.sin(el),
    ])

    # Camera axes
    fwd = np.array([cx, cy, cz]) - eye
    fwd_n = np.linalg.norm(fwd)
    if fwd_n < 1e-6:
        return None
    fwd /= fwd_n

    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    rn = np.linalg.norm(right)
    right = right / rn if rn > 1e-6 else np.array([1.0, 0.0, 0.0])
    up = np.cross(right, fwd)  # normalised by construction

    # Point in camera space
    d     = np.array(pt3d, dtype=float) - eye
    cam_x = np.dot(d, right)
    cam_y = np.dot(d, up)
    cam_z = np.dot(d, fwd)   # positive = in front of camera

    if cam_z < 0.01:
        return None

    # Perspective divide — _pip_cam.fov is the full vertical FOV
    half_vfov = np.radians(float(_pip_cam.fov) / 2.0)
    aspect    = PIP_W / PIP_H
    ndc_y =  cam_y / (cam_z * np.tan(half_vfov))
    ndc_x =  cam_x / (cam_z * np.tan(half_vfov) * aspect)

    # NDC → PiP canvas pixels  (ndc_y=+1 → top of PiP, y=0 is top in canvas)
    px = _PIP_X0 + (ndc_x + 1.0) * 0.5 * PIP_W
    py = _PIP_Y0 + (1.0 - (ndc_y + 1.0) * 0.5) * PIP_H
    return float(px), float(py)


def _pip_camera_angles(dx, dy, dz, tx, ty, tz=0.0):
    """
    (azimuth, elevation, distance) for vispy TurntableCamera centred at
    (tx,ty,tz) with the camera eye at (dx,dy,dz).

    Vispy standard formula:
        cam_pos = center + dist * (sin(az)*cos(el), -cos(az)*cos(el), sin(el))
    Inverting: el = arcsin((dz-tz)/dist),  az = arctan2(dx-tx, ty-dy).
    """
    dist = max(np.sqrt((dx-tx)**2 + (dy-ty)**2 + (dz-tz)**2), 0.01)
    el   = float(np.clip(np.degrees(np.arcsin(np.clip((dz - tz) / dist, -1.0, 1.0))), -89, 89))
    az   = float(np.degrees(np.arctan2(dx - tx, ty - dy)))
    return az, el, dist

# ── Canvas + main 3-D view ────────────────────────────────────────────────────

canvas = scene.SceneCanvas(
    size=(W, H), title="Vista-Tracker — Chase Camera",
    bgcolor="#0d1117", show=True, keys="interactive",
)

view = canvas.central_widget.add_view()

# Chase camera
row0  = df.iloc[0]
_dx0  = float(row0["drone_x"]); _dy0 = float(row0["drone_y"])
_dz0  = float(row0.get("drone_z", 2.0))
_tx0  = float(row0["target_x"]);  _ty0 = float(row0["target_y"])

cam = scene.cameras.TurntableCamera(
    fov=TRACKING_FOV_DEG, elevation=CHASE_EL, azimuth=0.0,
    distance=CHASE_DIST, up="+z",
)
view.camera = cam   # sets cam.parent = view.scene internally

_cam_center = np.array([_dx0, _dy0, _dz0])
# azimuth places camera on opposite side of drone from target
_cam_az     = [float(np.degrees(np.arctan2(_dx0 - _tx0, _ty0 - _dy0)))]

# ── World geometry ────────────────────────────────────────────────────────────

_cx_w    = (GRID_X_MAX + GRID_X_MIN) / 2.0
_cy_w    = (GRID_Y_MAX + GRID_Y_MIN) / 2.0
_gsize_x = GRID_X_MAX - GRID_X_MIN
_gsize_y = GRID_Y_MAX - GRID_Y_MIN

_ground = visuals.Plane(
    width=_gsize_x, height=_gsize_y,
    width_segments=25, height_segments=25,
    color=(0.07, 0.11, 0.07, 1.0),
    edge_color=(0.12, 0.20, 0.12, 0.35),
    parent=view.scene,
)
_ground.transform = transforms.STTransform(translate=(_cx_w, _cy_w, 0.0))

_obs_boxes = []
for obs in obstacles:
    sz  = obs["size"]
    box = visuals.Box(
        width=2*sz, height=2*sz, depth=2*sz,
        color=(0.8, 0.15, 0.05, 0.88),
        edge_color=(0.25, 0.05, 0.02, 0.9),
        parent=view.scene,
    )
    box.transform = transforms.STTransform(translate=(obs["x"], obs["y"], float(sz)))
    _obs_boxes.append(box)

# Shared MeshData — person scaled from PERSON_HEIGHT config value.
# Body: cylinder from z=0 to z=BODY_LEN.  Head: sphere above the body.
_HEAD_R     = 0.22                            # head sphere radius (m) — fixed proportion
_BODY_LEN   = PERSON_HEIGHT - 2.0 * _HEAD_R  # leaves room for a full head diameter
_BODY_CEN_Z = 0.0                            # create_cylinder offset=False: z runs 0→length, bottom already at 0
_HEAD_CEN_Z = _BODY_LEN + _HEAD_R           # head centre (just above body top)
_cyl_md  = create_cylinder(rows=10, cols=14, radius=[0.28, 0.28], length=_BODY_LEN)
_head_md = create_sphere(rows=8, cols=12, radius=_HEAD_R)

_person_cyl = visuals.Mesh(meshdata=_cyl_md, color=(0.82, 0.28, 0.08, 0.95),
                            shading="flat", parent=view.scene)
_cyl_tr = transforms.STTransform()
_person_cyl.transform = _cyl_tr

_person_head = visuals.Mesh(meshdata=_head_md, color=(1.0, 0.78, 0.58, 1.0),
                             shading="smooth", parent=view.scene)
_head_tr = transforms.STTransform()
_person_head.transform = _head_tr

# Drone 3-D model
_drone_arms = visuals.Line(parent=view.scene,
                            color=(0.20, 0.75, 1.0, 0.95), width=2.5, method="gl")
_drone_rotors = visuals.Line(parent=view.scene,
                              color=(0.20, 0.75, 1.0, 0.55), width=1.5, method="gl")
_drone_body_dot = visuals.Markers(parent=view.scene)
_drone_body_dot.set_data(pos=np.zeros((1, 3), dtype=np.float32),
                         face_color=(0.20, 0.85, 1.0, 1.0), size=10, edge_width=0)

# Trails
_trail_3d = visuals.Line(parent=view.scene,
                          color=(1.0, 0.50, 0.10, 0.55), width=3, method="gl")
_trail_3d.visible = False
_world_trail: list[tuple[float, float, float]] = []

_drone_trail_3d = visuals.Line(parent=view.scene,
                                color=(0.20, 0.75, 1.0, 0.40), width=2, method="gl")
_drone_trail_3d.visible = False
_drone_trail_world: list[tuple[float, float, float]] = []

# Camera frustum
_frustum = visuals.Line(parent=view.scene,
                         color=(0.15, 1.0, 0.15, 0.30), width=1.5, method="gl")

# ══ Mini-map (bottom-left) ════════════════════════════════════════════════════

_mini_vb = widgets.ViewBox(
    parent=canvas.scene,
    border_color=(0.55, 0.55, 0.55, 0.9),
    bgcolor=(0.05, 0.08, 0.05, 0.90),
)
_mini_vb.pos  = (10, H - MINI_H - 10)
_mini_vb.size = (MINI_W, MINI_H)

_mini_cam = scene.cameras.PanZoomCamera(aspect=1)
_mini_vb.camera = _mini_cam
_half = max(_gsize_x, _gsize_y) / 2 * 1.05
_mini_cam.rect = (_cx_w - _half, _cy_w - _half, 2 * _half, 2 * _half)

visuals.Line(
    pos=np.array([[GRID_X_MIN, GRID_Y_MIN, 0], [GRID_X_MAX, GRID_Y_MIN, 0],
                  [GRID_X_MAX, GRID_Y_MAX, 0], [GRID_X_MIN, GRID_Y_MAX, 0],
                  [GRID_X_MIN, GRID_Y_MIN, 0]], dtype=np.float32),
    color=(0.35, 0.45, 0.35, 0.6), width=1, method="gl", parent=_mini_vb.scene,
)
_mini_obs_lines = []
for obs in obstacles:
    ox, oy, sz = obs["x"], obs["y"], obs["size"]
    _ln = visuals.Line(
        pos=np.array([[ox-sz, oy-sz, 0], [ox+sz, oy-sz, 0],
                      [ox+sz, oy+sz, 0], [ox-sz, oy+sz, 0],
                      [ox-sz, oy-sz, 0]], dtype=np.float32),
        color=(0.80, 0.25, 0.05, 0.75), width=1.5, method="gl", parent=_mini_vb.scene,
    )
    _mini_obs_lines.append(_ln)

_mini_fov = visuals.Line(parent=_mini_vb.scene,
                          color=(0.20, 1.0, 0.20, 0.25), width=1, method="gl")
_mini_arms = visuals.Line(parent=_mini_vb.scene,
                           color=(0.20, 0.75, 1.0, 0.85), width=2, method="gl")
_mini_rotors = visuals.Line(parent=_mini_vb.scene,
                             color=(0.20, 0.75, 1.0, 0.55), width=1, method="gl")
_mini_drone_dot = visuals.Markers(parent=_mini_vb.scene)
_mini_drone_dot.set_data(pos=np.zeros((1, 3), dtype=np.float32),
                         face_color=(0.20, 0.85, 1.0, 1.0), size=8, edge_width=0)
_mini_target_dot = visuals.Markers(parent=_mini_vb.scene)
_mini_target_dot.set_data(pos=np.zeros((1, 3), dtype=np.float32),
                           face_color=(1.0, 0.35, 0.05, 1.0), size=9, edge_width=0)
_mini_trail = visuals.Line(parent=_mini_vb.scene,
                            color=(1.0, 0.5, 0.1, 0.5), width=1.5, method="gl")
_mini_trail.visible = False

# ══ Drone-cam PiP (bottom-right overlay) ══════════════════════════════════════

_pip_vb = widgets.ViewBox(
    parent=canvas.scene,
    border_color=(0.20, 0.75, 1.0, 0.80),
    bgcolor=(0.03, 0.06, 0.10, 0.92),
)
_pip_vb.pos  = (_PIP_X0, _PIP_Y0)
_pip_vb.size = (PIP_W, PIP_H)

_pip_cam = scene.cameras.TurntableCamera(
    fov=_PIP_VFOV_DEG, elevation=0.0, azimuth=0.0,
    distance=DESIRED_DISTANCE, up="+z",
)
_pip_vb.camera = _pip_cam

# PiP ground
_pip_ground = visuals.Plane(
    width=_gsize_x, height=_gsize_y,
    width_segments=15, height_segments=15,
    color=(0.07, 0.11, 0.07, 1.0),
    edge_color=(0.12, 0.20, 0.12, 0.25),
    parent=_pip_vb.scene,
)
_pip_ground.transform = transforms.STTransform(translate=(_cx_w, _cy_w, 0.0))

# PiP obstacles — Box visuals in _pip_vb.scene so the PiP TurntableCamera handles
# perspective scaling.  depth_test=False bypasses the depth-buffer contamination
# from the main 3-D scene (the ViewBox clears its colour region but not its depth
# region, so meshes with depth_test=True never pass the depth test).
_pip_obs_boxes = []
for _obs in obstacles:
    _sz  = _obs["size"]
    _box = visuals.Box(
        width=2*_sz, height=2*_sz, depth=2*_sz,
        color=(0.8, 0.15, 0.05, 0.88),
        edge_color=(0.20, 0.04, 0.01, 0.7),
        parent=_pip_vb.scene,
    )
    _box.transform = transforms.STTransform(translate=(_obs["x"], _obs["y"], float(_sz)))
    _box.mesh.set_gl_state('translucent', depth_test=False)
    _box.border.set_gl_state('translucent', depth_test=False)
    _pip_obs_boxes.append(_box)

# PiP person — cylinder + head mesh in _pip_vb.scene so the PiP TurntableCamera
# handles perspective scaling natively.  depth_test=False bypasses the depth-buffer
# contamination from the main 3-D scene (same fix as _pip_obs_boxes).
_pip_person_cyl = visuals.Mesh(meshdata=_cyl_md, color=(0.82, 0.28, 0.08, 0.95),
                                shading="flat", parent=_pip_vb.scene)
_pip_person_cyl.set_gl_state('translucent', depth_test=False)
_pip_cyl_tr = transforms.STTransform()
_pip_person_cyl.transform = _pip_cyl_tr

_pip_person_head = visuals.Mesh(meshdata=_head_md, color=(1.0, 0.78, 0.58, 1.0),
                                 shading="smooth", parent=_pip_vb.scene)
_pip_person_head.set_gl_state('translucent', depth_test=False)
_pip_head_tr = transforms.STTransform()
_pip_person_head.transform = _pip_head_tr

# PiP label + crosshair
_ch     = 7
_PIP_CX = _PIP_X0 + PIP_W // 2
_PIP_CY = _PIP_Y0 + PIP_H // 2
visuals.Text(
    "DRONE CAM", color=(0.30, 0.80, 1.0, 0.85), font_size=8, bold=True,
    pos=(_PIP_X0 + 5, _PIP_Y0 + 5), anchor_x="left", anchor_y="top",
    parent=canvas.scene,
)
visuals.Line(
    pos=np.array([[_PIP_CX - _ch, _PIP_CY], [_PIP_CX + _ch, _PIP_CY]], dtype=np.float32),
    color=(1., 1., 1., 0.45), width=1, method="gl", parent=canvas.scene,
)
visuals.Line(
    pos=np.array([[_PIP_CX, _PIP_CY - _ch], [_PIP_CX, _PIP_CY + _ch]], dtype=np.float32),
    color=(1., 1., 1., 0.45), width=1, method="gl", parent=canvas.scene,
)

# 2-D bounding box overlay drawn in canvas (screen) space on the PiP image
_pip_bbox_2d = visuals.Line(
    parent=canvas.scene,
    color=(0.15, 1.0, 0.15, 0.90), width=1.5, method="gl",
)
_pip_bbox_2d.visible = False

# ══ HUD ══════════════════════════════════════════════════════════════════════
# Three separate Text visuals to avoid vispy multiline clipping:
#   _hud_line1  : t + position + range  (y=12)
#   _hud_yaw    : cam yaw act/des/err   (y=30)
#   _hud_pitch  : cam pitch act/des/ideal (y=48)
_hud_line1 = visuals.Text(
    "", color="white", font_size=11,
    pos=(12, 12), anchor_x="left", anchor_y="top",
    parent=canvas.scene,
)
_hud_yaw = visuals.Text(
    "", color="white", font_size=11,
    pos=(12, 30), anchor_x="left", anchor_y="top",
    parent=canvas.scene,
)
_hud_pitch = visuals.Text(
    "", color=(0.60, 0.85, 1.0, 1.0), font_size=11,
    pos=(12, 48), anchor_x="left", anchor_y="top",
    parent=canvas.scene,
)
_hud_status = visuals.Text(
    "", color="#00ff88", font_size=14, bold=True,
    pos=(W // 2, H - 20), anchor_x="center", anchor_y="bottom",
    parent=canvas.scene,
)

# ── Animation ──────────────────────────────────────────────────────────────────

_frame = [0]


def _update_drone_3d(dx, dy, dz, yaw):
    arm_pts = []
    rotor_pts = []
    for off in (45, 135, -135, -45):
        a  = yaw + np.radians(off)
        rx = dx + _ARM * np.cos(a)
        ry = dy + _ARM * np.sin(a)
        arm_pts.extend([[dx, dy, dz], [rx, ry, dz]])
        circ = _rotor_circle(rx, ry, _R_ROT).copy()
        circ[:, 2] = dz
        rotor_pts.extend(circ.tolist())
        rotor_pts.append([np.nan, np.nan, np.nan])
    _drone_arms.set_data(pos=np.array(arm_pts, dtype=np.float32), connect="segments")
    _drone_rotors.set_data(pos=np.array(rotor_pts, dtype=np.float32))
    _drone_body_dot.set_data(pos=np.array([[dx, dy, dz]], dtype=np.float32),
                              face_color=(0.20, 0.85, 1.0, 1.0), size=10, edge_width=0)


def _update_minimap_drone(dx, dy, yaw):
    arm_pts = []
    rotor_pts = []
    for off in (45, 135, -135, -45):
        a  = yaw + np.radians(off)
        rx = dx + _ARM * np.cos(a)
        ry = dy + _ARM * np.sin(a)
        arm_pts.extend([[dx, dy, 0.0], [rx, ry, 0.0]])
        rotor_pts.extend(_rotor_circle(rx, ry, _R_ROT).tolist())
        rotor_pts.append([np.nan, np.nan, np.nan])
    _mini_arms.set_data(pos=np.array(arm_pts, dtype=np.float32), connect="segments")
    _mini_rotors.set_data(pos=np.array(rotor_pts, dtype=np.float32))
    _mini_drone_dot.set_data(pos=np.array([[dx, dy, 0.0]], dtype=np.float32),
                              face_color=(0.20, 0.85, 1.0, 1.0), size=8, edge_width=0)


def _on_timer(event):
    i = _frame[0]
    if i >= len(df):
        timer.stop()
        return

    row  = df.iloc[i]
    dx   = float(row["drone_x"])
    dy   = float(row["drone_y"])
    dz   = float(row["drone_z"]) if "drone_z" in df.columns else 2.0
    tx   = float(row["target_x"])
    ty   = float(row["target_y"])
    tz   = PERSON_TRACK_Z   # upper back / head tracking point
    yaw           = float(row["drone_yaw"])
    pitch         = float(row["drone_pitch"])      if "drone_pitch"      in df.columns else 0.0
    pitch_rate    = float(row["pitch_rate"])       if "pitch_rate"       in df.columns else 0.0
    yaw_des       = float(row["ref_yaw"])          if "ref_yaw"          in df.columns else float(np.arctan2(ty - dy, tx - dx))
    pitch_ref_cam = float(row["ref_camera_pitch"]) if "ref_camera_pitch" in df.columns else 0.0
    # commanded pitch = drone_pitch + attitude-error correction (mirrors visualize.py)
    pitch_des     = pitch + pitch_rate / ATT_KP
    tsim  = float(row["t"])

    # ── Chase camera — always directly behind drone along its yaw heading ────
    _cam_center[:] += CAM_SMOOTH * (np.array([dx, dy, dz]) - _cam_center)
    # vispy eye = center + dist*(sin(az)*cos(el), -cos(az)*cos(el), sin(el))
    # For eye to lie in the -yaw direction: az = atan2(-cos(yaw), sin(yaw))
    az_target = float(np.degrees(np.arctan2(-np.cos(yaw), np.sin(yaw))))
    d_az = (az_target - _cam_az[0] + 180.0) % 360.0 - 180.0
    _cam_az[0] += CAM_SMOOTH_AZ * d_az
    cam.center    = (_cam_center[0], _cam_center[1], _cam_center[2])
    cam.azimuth   = _cam_az[0]
    cam.elevation = CHASE_EL
    cam.distance  = CHASE_DIST

    # ── Obstacles — ESDF colour + visible only when surface in sensing range ────
    for obs, box, pip_box, mini_ln in zip(obstacles, _obs_boxes, _pip_obs_boxes, _mini_obs_lines):
        d_obs    = np.hypot(dx - obs["x"], dy - obs["y"])
        in_range = max(0.0, d_obs - obs["size"]) <= CAMERA_RANGE
        box.visible     = in_range
        pip_box.visible = in_range
        mini_ln.visible = in_range
        if in_range:
            c = _esdf_color(dx, dy, obs)
            box.mesh.color     = c
            pip_box.mesh.color = c
            mini_ln.set_data(color=c)

    # ── Person (1.80 m — body cylinder + head sphere) ────────────────────────
    _cyl_tr.translate  = (tx, ty, _BODY_CEN_Z)
    _head_tr.translate = (tx, ty, _HEAD_CEN_Z)

    # ── Drone 3-D model ───────────────────────────────────────────────────────
    _update_drone_3d(dx, dy, dz, yaw)

    # ── Camera pitch toward tracking point (reference/ideal for HUD display) ─
    horiz_dist = max(float(np.hypot(tx - dx, ty - dy)), 0.01)
    pitch_cam  = float(np.arctan2(dz - tz, horiz_dist))   # positive = pitched down

    # Actual camera boresight: drone body IS the camera platform (no gimbal).
    # bore = unit vector in the direction the camera is pointing (ENU).
    bore = np.array([np.cos(yaw) * np.cos(pitch),
                     np.sin(yaw) * np.cos(pitch),
                     -np.sin(pitch)])

    # ── Camera frustum (actual drone body pitch = camera pitch, no gimbal) ───
    _frustum.set_data(pos=_frustum_pts(dx, dy, dz, yaw, pitch,
                                        TRACKING_HALF_FOV, TRACKING_HALF_VFOV,
                                        CAMERA_RANGE))

    # ── Trails ────────────────────────────────────────────────────────────────
    _world_trail.append((tx, ty, 0.05))   # ground-level path trace
    if len(_world_trail) > TRAIL_LEN: _world_trail.pop(0)
    if len(_world_trail) >= 2:
        _trail_3d.set_data(pos=np.array(_world_trail, dtype=np.float32))
        _trail_3d.visible = True

    _drone_trail_world.append((dx, dy, dz))
    if len(_drone_trail_world) > TRAIL_LEN: _drone_trail_world.pop(0)
    if len(_drone_trail_world) >= 2:
        _drone_trail_3d.set_data(pos=np.array(_drone_trail_world, dtype=np.float32))
        _drone_trail_3d.visible = True

    # ── PiP camera: body-fixed view along actual drone yaw + pitch ──────────
    # Boresight = drone forward axis.  The center point is placed along the
    # boresight at the 3-D distance to the target's mid-body so perspective
    # scaling matches the real sensor.  When yaw or pitch error is non-zero
    # the target drifts away from PiP centre, showing the true tracking error.
    bore_pip = np.array([np.cos(yaw) * np.cos(pitch),
                         np.sin(yaw) * np.cos(pitch),
                         -np.sin(pitch)])
    dist_3d  = max(float(np.linalg.norm(
        np.array([tx - dx, ty - dy, PERSON_TRACK_Z - dz]))), 0.5)
    cx_pip   = float(dx + bore_pip[0] * dist_3d)
    cy_pip   = float(dy + bore_pip[1] * dist_3d)
    cz_pip   = float(dz + bore_pip[2] * dist_3d)
    az_pip, el_pip, dist_pip = _pip_camera_angles(dx, dy, dz, cx_pip, cy_pip, cz_pip)
    _pip_cam.center    = (cx_pip, cy_pip, cz_pip)
    _pip_cam.distance  = float(dist_pip)
    _pip_cam.azimuth   = float(az_pip)
    _pip_cam.elevation = float(el_pip)

    # ── PiP person cylinder + head ────────────────────────────────────────────
    _pip_cyl_tr.translate  = (tx, ty, _BODY_CEN_Z)
    _pip_head_tr.translate = (tx, ty, _HEAD_CEN_Z)

    # ── Bounding box — 2-D overlay projected onto PiP canvas pixels ───────────
    _vdx, _vdy = tx - dx, ty - dy
    _vd_n = max(np.hypot(_vdx, _vdy), 0.01)
    _rvx, _rvy = -_vdy / _vd_n, _vdx / _vd_n
    _bw = PERSON_WIDTH / 2.0
    _corners_3d = [
        [tx - _bw * _rvx, ty - _bw * _rvy, 0.0],
        [tx + _bw * _rvx, ty + _bw * _rvy, 0.0],
        [tx + _bw * _rvx, ty + _bw * _rvy, PERSON_HEIGHT],
        [tx - _bw * _rvx, ty - _bw * _rvy, PERSON_HEIGHT],
    ]
    _sc = [_project_to_pip(c) for c in _corners_3d]
    if all(s is not None for s in _sc):
        _xs = [s[0] for s in _sc]
        _ys = [s[1] for s in _sc]
        x0_bb, x1_bb = min(_xs), max(_xs)
        y0_bb, y1_bb = min(_ys), max(_ys)
        _pip_bbox_2d.set_data(pos=np.array([
            [x0_bb, y0_bb], [x1_bb, y0_bb],
            [x1_bb, y1_bb], [x0_bb, y1_bb],
            [x0_bb, y0_bb],
        ], dtype=np.float32))
        _pip_bbox_2d.visible = True
    else:
        _pip_bbox_2d.visible = False


    # ── Mini-map ──────────────────────────────────────────────────────────────
    _update_minimap_drone(dx, dy, yaw)

    fov_l = yaw + TRACKING_HALF_FOV
    fov_r = yaw - TRACKING_HALF_FOV
    _mini_fov.set_data(pos=np.array([
        [dx, dy, 0],
        [dx + CAMERA_RANGE * np.cos(fov_l), dy + CAMERA_RANGE * np.sin(fov_l), 0],
        [dx + CAMERA_RANGE * np.cos(fov_r), dy + CAMERA_RANGE * np.sin(fov_r), 0],
        [dx, dy, 0],
    ], dtype=np.float32))

    _mini_target_dot.set_data(
        pos=np.array([[tx, ty, 0.0]], dtype=np.float32),
        face_color=(1.0, 0.35, 0.05, 1.0), size=9, edge_width=0,
    )
    if len(_world_trail) >= 2:
        _mini_trail.set_data(
            pos=np.array([[wx, wy, 0.0] for wx, wy, _ in _world_trail], dtype=np.float32)
        )
        _mini_trail.visible = True

    # ── HUD ───────────────────────────────────────────────────────────────────
    ref_yaw = np.arctan2(ty - dy, tx - dx)
    yaw_err = np.arctan2(np.sin(yaw - ref_yaw), np.cos(yaw - ref_yaw))

    # Geometric FOV check — does the target actually fall inside the camera frustum?
    #   horizontal: yaw error to target vs H-FOV half-angle
    #   vertical:   angle from camera boresight to target vs V-FOV half-angle
    #
    # alpha  = angle to target below horizontal (ENU: positive = target below drone)
    # pitch  = body pitch (ENU: positive = nose tilted down = boresight below horizontal)
    # alpha - pitch = target's angular offset from boresight centre (positive = target below boresight)
    horiz_dist_2d = float(np.hypot(tx - dx, ty - dy))
    alpha         = float(np.arctan2(dz - tz, horiz_dist_2d))   # geometric elevation to target
    pitch_to_target = alpha - pitch                              # offset from camera boresight
    dist   = float(np.hypot(horiz_dist_2d, tz - dz))
    in_fov = (abs(yaw_err) <= TRACKING_HALF_FOV) and \
             (abs(pitch_to_target) <= TRACKING_HALF_VFOV)

    # Controller tracking error (for HUD display only — not used for FOV check)
    pitch_cam_error = float(np.arctan2(
        np.sin(pitch_des - pitch_ref_cam),
        np.cos(pitch_des - pitch_ref_cam)))

    _hud_line1.text = (
        f"t = {tsim:.1f} s   drone ({dx:.1f}, {dy:.1f}, {dz:.1f}) m"
        f"   range = {dist:.2f} m   Δ = {dist - DESIRED_DISTANCE:+.2f} m"
    )
    _hud_yaw.text = (
        f"yaw    act {np.degrees(yaw):.1f}°   des {np.degrees(yaw_des):.1f}°"
        f"   err {np.degrees(yaw_err):.1f}°  [lim ±{np.degrees(TRACKING_HALF_FOV):.0f}°]"
    )
    _hud_pitch.text = (
        f"pitch  act {np.degrees(pitch):.1f}°   cmd {np.degrees(pitch_des):.1f}°"
        f"   ref {np.degrees(pitch_ref_cam):.1f}°   err {np.degrees(pitch_cam_error):.1f}°"
        f"  [lim ±{np.degrees(TRACKING_HALF_VFOV):.0f}°]"
    )
    _hud_status.text  = "TARGET IN FOV" if in_fov else "TARGET LOST"
    _hud_status.color = "#00ff88" if in_fov else "#ff4444"

    _frame[0] += 1
    canvas.update()


timer = app.Timer(interval=float(SIM_DT), connect=_on_timer, start=True)

if __name__ == "__main__":
    app.run()
