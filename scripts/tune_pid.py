#!/usr/bin/env python3
"""
Scenario-based cascade PID tuner for Vista-Tracker.

Reads a scenario YAML and optimises PID gains against the actual target
trajectory. The reference at each step is the standoff point:

  ref = target_pos + normalize(drone_pos - target_pos) * desired_distance

No planner is used. The drone starts at the initial standoff position
(direction from target → drone_init), so initial position error is ~zero.
Cost is accumulated only after SETTLE_T seconds (let yaw and integrators settle).

Cost = ITAE(standoff tracking error) + SMOOTHING * integral(wx_cmd² + wy_cmd²)

Usage:
  python3 scripts/tune_pid.py urban_block
  python3 scripts/tune_pid.py config/scenarios/urban_block.yaml
  python3 scripts/tune_pid.py urban_block --no-plot
  python3 scripts/tune_pid.py urban_block --duration 30  # override opt window (s)
"""

from __future__ import annotations

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.optimize import Bounds, minimize

# ── Physical constants ────────────────────────────────────────────────────────
G = 9.81  # m/s²

# ── Tuning constants ──────────────────────────────────────────────────────────
SMOOTHING    = 0.10   # control-effort penalty weight (matches tune_pid_sine_tracking)
SETTLE_T     = 2.0    # seconds excluded from cost (initial yaw + integrator settling)
OPT_DURATION = 25.0   # optimisation window (s) — full scenario used for plot only
N_INNER      = 50     # inner sub-steps per outer dt for 2nd-order dynamics
MAX_ANGLE    = 0.5    # rad tilt limit
MAX_THRUST   = 2.0    # normalised

# ── Config loading ────────────────────────────────────────────────────────────

def _deep_merge(base: dict, override: dict) -> dict:
    result = dict(base)
    for k, v in override.items():
        if k == "base":
            continue
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def load_config(path: str) -> dict:
    with open(path) as f:
        node = yaml.safe_load(f) or {}
    if "base" in node:
        base_path = os.path.join(os.path.dirname(os.path.abspath(path)), node["base"])
        return _deep_merge(load_config(base_path), node)
    return node


def resolve_config(arg: str) -> str:
    if os.path.exists(arg):
        return arg
    candidate = os.path.join(os.path.dirname(__file__), "../config/scenarios", arg + ".yaml")
    if os.path.exists(candidate):
        return candidate
    sys.exit(f"error: scenario '{arg}' not found (tried {candidate})")


# ── Target trajectory simulation — mirrors WaypointFollower.cpp ───────────────
_REACH_TOL = 0.1   # m  — kReachThreshold in C++
_MIN_SPEED  = 0.05  # m/s — kMinSpeedForTurn in C++


def _simulate_target(wps: list, traj: dict, dt: float, n: int):
    """
    Returns (tx, ty, tz) arrays, each of length n.
    Exact Python mirror of WaypointFollower::step().
    """
    tx = np.empty(n)
    ty = np.empty(n)
    tz = np.empty(n)

    max_speed         = traj["max_speed"]
    max_accel         = traj["max_accel"]
    max_lateral_accel = traj["max_lateral_accel"]
    loop_flag         = traj.get("loop", False)

    sx, sy, sz = wps[0]["pos"][0], wps[0]["pos"][1], wps[0]["pos"][2]
    svx = svy = svz = 0.0
    heading      = 0.0
    idx          = 0
    done         = False
    holding      = False
    hold_elapsed = 0.0

    for i in range(n):
        if done or idx >= len(wps):
            tx[i], ty[i], tz[i] = sx, sy, sz
            done = True
            continue

        wp       = wps[idx]
        wpx, wpy, wpz = wp["pos"][0], wp["pos"][1], wp["pos"][2]
        wp_speed = min(wp.get("speed", max_speed), max_speed)
        wp_hold  = wp.get("hold", 0.0)

        if holding:
            hold_elapsed += dt
            svx = svy = svz = 0.0
            if hold_elapsed >= wp_hold:
                holding = False; hold_elapsed = 0.0
                idx += 1
                if idx >= len(wps):
                    done = not loop_flag
                    if loop_flag:
                        idx = 0
            tx[i], ty[i], tz[i] = sx, sy, sz
            continue

        dx   = wpx - sx
        dy   = wpy - sy
        dz   = wpz - sz
        dist = np.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < _REACH_TOL:
            if wp_hold > 0.0:
                holding = True; hold_elapsed = 0.0
            else:
                idx += 1
                if idx >= len(wps):
                    done = not loop_flag
                    if loop_flag:
                        idx = 0
            tx[i], ty[i], tz[i] = sx, sy, sz
            continue

        # Longitudinal speed: cruise vs braking ramp
        cur_speed = np.sqrt(svx*svx + svy*svy + svz*svz)
        v_target  = min(wp_speed, np.sqrt(2.0 * max_accel * dist))
        dv        = np.clip(v_target - cur_speed, -max_accel * dt, max_accel * dt)
        new_speed = max(0.0, cur_speed + dv)

        # Smooth heading (lateral accel constraint)
        desired_heading = np.arctan2(dy, dx)
        heading_err     = (desired_heading - heading + np.pi) % (2 * np.pi) - np.pi
        max_yaw_rate    = max_lateral_accel / max(new_speed, _MIN_SPEED)
        heading        += np.clip(heading_err, -max_yaw_rate * dt, max_yaw_rate * dt)

        uz  = dz / dist
        sx += np.cos(heading) * new_speed * dt
        sy += np.sin(heading) * new_speed * dt
        sz += uz * new_speed * dt
        svx = np.cos(heading) * new_speed
        svy = np.sin(heading) * new_speed
        svz = uz * new_speed

        tx[i], ty[i], tz[i] = sx, sy, sz

    return tx, ty, tz


# ── Cascade PID simulation ────────────────────────────────────────────────────

def simulate(
    gains: np.ndarray,
    ref_xa: np.ndarray, ref_ya: np.ndarray,
    tx: np.ndarray, ty: np.ndarray,
    init_x: float, init_y: float,
    z_ref: float, dt: float,
    wn: float, zeta: float, wn_yaw: float, zeta_yaw: float,
) -> dict:
    """
    Simulate cascade PID tracking pre-computed reference arrays.
    ref_xa/ref_ya = target(t) + fixed_init_dir * desired_dist (state-independent).
    tx/ty are target positions (used only for yaw computation).
    Drone starts at (init_x, init_y, z_ref).
    """
    kp, ki, kd, att_kp, yaw_kp = gains
    n        = len(ref_xa)
    dt_inner = dt / N_INNER

    wn2_rp   = wn * wn;       damp_rp  = 2.0 * zeta     * wn
    wn2_yaw  = wn_yaw * wn_yaw; damp_yaw = 2.0 * zeta_yaw * wn_yaw

    def step2_rp(cmd, val, dval):
        for _ in range(N_INNER):
            ddval = wn2_rp * (cmd - val) - damp_rp * dval
            dval += ddval * dt_inner
            val  += dval  * dt_inner
        return val, dval

    def step2_yaw(cmd, val, dval):
        for _ in range(N_INNER):
            ddval = wn2_yaw * (cmd - val) - damp_yaw * dval
            dval += ddval * dt_inner
            val  += dval  * dt_inner
        return val, dval

    # State
    x, y, z    = init_x, init_y, z_ref
    vx = vy = vz = 0.0
    roll = pitch = yaw = 0.0
    wx,  wx_dot  = 0.0, 0.0
    wy,  wy_dot  = 0.0, 0.0
    wz,  wz_dot  = 0.0, 0.0
    thr, thr_dot = 1.0, 0.0  # start at hover thrust

    ix = iy = iz = 0.0
    prev_ex = prev_ey = prev_ez = 0.0

    x_log      = np.empty(n)
    y_log      = np.empty(n)
    ref_x_log  = np.empty(n)
    ref_y_log  = np.empty(n)
    wx_cmd_log = np.empty(n)
    wy_cmd_log = np.empty(n)
    wz_cmd_log = np.empty(n)
    wx_log     = np.empty(n)
    wy_log     = np.empty(n)
    wz_log     = np.empty(n)

    for i in range(n):
        # ── Reference: pre-computed, state-independent ────────────────────────
        ref_x = ref_xa[i]
        ref_y = ref_ya[i]

        # ── Outer PID ──────────────────────────────────────────────────────────
        ex = ref_x - x;  ey = ref_y - y;  ez = z_ref - z
        ix += ex * dt;   iy += ey * dt;   iz += ez * dt
        dex = (ex - prev_ex) / dt if i > 0 else 0.0
        dey = (ey - prev_ey) / dt if i > 0 else 0.0
        dez = (ez - prev_ez) / dt if i > 0 else 0.0
        ax_des = kp*ex + ki*ix + kd*dex
        ay_des = kp*ey + ki*iy + kd*dey
        az_des = kp*ez + ki*iz + kd*dez
        prev_ex, prev_ey, prev_ez = ex, ey, ez

        # ── Attitude setpoints (body-frame rotation) ───────────────────────────
        cy_now = np.cos(yaw);  sy_now = np.sin(yaw)
        ax_b   =  ax_des * cy_now + ay_des * sy_now
        ay_b   = -ax_des * sy_now + ay_des * cy_now
        pitch_des = float(np.clip(np.arctan2(ax_b, G), -MAX_ANGLE, MAX_ANGLE))
        roll_des  = float(np.clip(-np.arctan2(ay_b, G), -MAX_ANGLE, MAX_ANGLE))
        thr_cmd   = float(np.clip((G + az_des) / G, 0.0, MAX_THRUST))

        # Yaw: face the target
        yaw_des = float(np.arctan2(ty[i] - y, tx[i] - x))
        yaw_err = (yaw_des - yaw + np.pi) % (2.0 * np.pi) - np.pi

        # ── Inner loop ─────────────────────────────────────────────────────────
        roll_rate_cmd  = att_kp * (roll_des  - roll)
        pitch_rate_cmd = att_kp * (pitch_des - pitch)
        yaw_rate_cmd   = yaw_kp * yaw_err

        wx,  wx_dot  = step2_rp(roll_rate_cmd,  wx,  wx_dot)
        wy,  wy_dot  = step2_rp(pitch_rate_cmd, wy,  wy_dot)
        wz,  wz_dot  = step2_yaw(yaw_rate_cmd,  wz,  wz_dot)
        thr, thr_dot = step2_rp(thr_cmd,        thr, thr_dot)

        # ── Attitude + position integration (ZYX Euler) ───────────────────────
        roll  += wx * dt;  pitch += wy * dt;  yaw += wz * dt
        cr, sr = np.cos(roll),  np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw),   np.sin(yaw)
        T_val  = thr * G
        ax = (cy*sp*cr + sy*sr) * T_val
        ay = (sy*sp*cr - cy*sr) * T_val
        az =  cp*cr             * T_val - G
        vx += ax*dt;  x += vx*dt
        vy += ay*dt;  y += vy*dt
        vz += az*dt;  z += vz*dt

        x_log[i]      = x;             y_log[i]      = y
        ref_x_log[i]  = ref_x;         ref_y_log[i]  = ref_y
        wx_cmd_log[i] = roll_rate_cmd;  wy_cmd_log[i] = pitch_rate_cmd
        wz_cmd_log[i] = yaw_rate_cmd
        wx_log[i]     = wx;             wy_log[i]     = wy;  wz_log[i] = wz

    t = np.arange(n) * dt
    return {
        "t": t,
        "x": x_log,       "y": y_log,
        "ref_x": ref_xa,  "ref_y": ref_ya,
        "target_x": tx,   "target_y": ty,
        "wx_cmd": wx_cmd_log, "wy_cmd": wy_cmd_log, "wz_cmd": wz_cmd_log,
        "wx": wx_log,     "wy": wy_log,     "wz": wz_log,
    }


# ── Cost function ─────────────────────────────────────────────────────────────

def itae_cost(
    gains: np.ndarray,
    ref_xa: np.ndarray, ref_ya: np.ndarray,
    tx: np.ndarray, ty: np.ndarray,
    init_x: float, init_y: float,
    z_ref: float, dt: float, settle_idx: int,
    wn: float, zeta: float, wn_yaw: float, zeta_yaw: float,
) -> float:
    if gains[0] <= 0 or gains[3] <= 0 or gains[4] <= 0:
        return 1e9
    d = simulate(gains, ref_xa, ref_ya, tx, ty, init_x, init_y,
                 z_ref, dt, wn, zeta, wn_yaw, zeta_yaw)
    t      = d["t"][settle_idx:]
    ex     = d["x"][settle_idx:]     - ref_xa[settle_idx:]
    ey     = d["y"][settle_idx:]     - ref_ya[settle_idx:]
    wx_cmd = d["wx_cmd"][settle_idx:]
    wy_cmd = d["wy_cmd"][settle_idx:]
    t0     = t[0]  # shift time so ITAE weight starts at 0 after settle window
    itae   = float(np.trapz((t - t0) * (np.abs(ex) + np.abs(ey)), t))
    effort = float(np.trapz(wx_cmd**2 + wy_cmd**2, t))
    return itae + SMOOTHING * effort


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", help="scenario name or YAML path")
    parser.add_argument("--no-plot",  action="store_true")
    parser.add_argument("--duration", type=float, default=OPT_DURATION,
                        help=f"optimisation window in seconds (default {OPT_DURATION})")
    args = parser.parse_args()

    cfg_path = resolve_config(args.scenario)
    cfg      = load_config(cfg_path)

    # ── Extract scenario fields ───────────────────────────────────────────────
    dt        = cfg["sim"]["dt"]
    T_full    = cfg["sim"]["T"]
    wn        = cfg["drone"]["wn"]
    zeta      = cfg["drone"]["zeta"]
    wn_yaw    = cfg["drone"]["wn_yaw"]
    zeta_yaw  = cfg["drone"]["zeta_yaw"]

    desired_dist = cfg["controller"]["desired_distance"]
    z_ref        = cfg["drone_init"]["z"]
    d0x          = cfg["drone_init"]["x"]
    d0y          = cfg["drone_init"]["y"]

    traj = cfg["target_trajectory"]
    wps  = traj["waypoints"]

    # ── Initial drone position: standoff from target's first waypoint ─────────
    t0x = wps[0]["pos"][0];  t0y = wps[0]["pos"][1]
    ddx = d0x - t0x;         ddy = d0y - t0y
    dist0 = np.sqrt(ddx*ddx + ddy*ddy)
    if dist0 > 0.01:
        init_x = t0x + ddx / dist0 * desired_dist
        init_y = t0y + ddy / dist0 * desired_dist
    else:
        init_x = t0x + desired_dist
        init_y = t0y

    # ── Pre-simulate target for full scenario duration ────────────────────────
    N_full    = int(T_full / dt)
    n_opt     = min(N_full, max(1, int(args.duration / dt)))
    settle_idx = int(SETTLE_T / dt)

    print(f"[tune_pid] scenario        : {cfg_path}")
    print(f"[tune_pid] target          : type={traj['type']}"
          f"  waypoints={len(wps)}  max_speed={traj['max_speed']} m/s")
    print(f"[tune_pid] drone dynamics  : wn={wn} rad/s  zeta={zeta}"
          f"  wn_yaw={wn_yaw} rad/s  zeta_yaw={zeta_yaw}")
    print(f"[tune_pid] desired_distance: {desired_dist} m")
    print(f"[tune_pid] drone_init      : ({d0x}, {d0y})  →  standoff ({init_x:.2f}, {init_y:.2f})"
          f"  target ({t0x}, {t0y})")
    print(f"[tune_pid] simulation      : dt={dt} s  opt={args.duration} s"
          f"  settle={SETTLE_T} s  full={T_full} s")

    print("\nPre-simulating target trajectory …")
    tx_full, ty_full, tz_full = _simulate_target(wps, traj, dt, N_full)

    # ── Pre-compute reference: fixed direction (init drone → target[0]), moves with target
    # Using a state-independent reference avoids circular dependency in the cost:
    # if ref = target + normalize(drone - target) * desired_dist, the optimizer can
    # trivially satisfy ref ≈ drone by not moving (reference follows the stationary drone).
    if dist0 > 0.01:
        unit_ix = ddx / dist0  # unit vector from target[0] toward drone_init
        unit_iy = ddy / dist0
    else:
        unit_ix, unit_iy = 1.0, 0.0

    ref_x_full = tx_full + unit_ix * desired_dist
    ref_y_full = ty_full + unit_iy * desired_dist

    ref_x_opt = ref_x_full[:n_opt]
    ref_y_opt = ref_y_full[:n_opt]
    tx_opt    = tx_full[:n_opt]
    ty_opt    = ty_full[:n_opt]

    # Shared args for cost evaluations
    cost_args = (ref_x_opt, ref_y_opt, tx_opt, ty_opt, init_x, init_y,
                 z_ref, dt, settle_idx, wn, zeta, wn_yaw, zeta_yaw)

    # ── Optimise ──────────────────────────────────────────────────────────────
    # Current scenario gains as starting point (fall back to base.yaml defaults)
    ctrl = cfg.get("controller", {})
    x0 = np.array([
        ctrl.get("kp",          8.0),
        ctrl.get("ki",          0.2),
        ctrl.get("kd",          5.0),
        ctrl.get("attitude_kp", 6.0),
        ctrl.get("yaw_kp",      0.3),
    ])
    bounds = Bounds(lb=[0.1, 0.0, 0.0, 1.0,  0.05],
                    ub=[20.0, 3.0, 10.0, 25.0, 5.0])

    print(f"\nInitial cost : {itae_cost(x0, *cost_args):.4f}")
    print("Optimising (L-BFGS-B) …")
    result = minimize(
        itae_cost, x0, args=cost_args,
        method="L-BFGS-B", bounds=bounds,
        options={"maxiter": 500, "ftol": 1e-9},
    )

    kp, ki, kd, att_kp, yaw_kp = result.x
    print(f"\n── Optimised gains ────────────────────────────────────────────")
    print(f"  kp          : {kp:.4f}")
    print(f"  ki          : {ki:.4f}")
    print(f"  kd          : {kd:.4f}")
    print(f"  attitude_kp : {att_kp:.4f}  (roll/pitch inner loop)")
    print(f"  yaw_kp      : {yaw_kp:.4f}  (yaw inner loop — wn={wn_yaw} rad/s plant)")
    print(f"  ITAE cost   : {result.fun:.6f}")
    print(f"\n── Paste into {os.path.basename(cfg_path)} ──────────────────────────")
    print(f"controller:")
    print(f"  kp:          {kp:.3f}")
    print(f"  ki:          {ki:.3f}")
    print(f"  kd:          {kd:.3f}")
    print(f"  attitude_kp: {att_kp:.3f}")
    print(f"  yaw_kp:      {yaw_kp:.3f}")

    if args.no_plot:
        return

    # ── Plot: run both gain sets over full scenario duration ──────────────────
    sim_args = (ref_x_full, ref_y_full, tx_full, ty_full, init_x, init_y,
                z_ref, dt, wn, zeta, wn_yaw, zeta_yaw)

    d0 = simulate(x0,       *sim_args)
    d1 = simulate(result.x, *sim_args)

    fig, axes = plt.subplots(4, 2, figsize=(13, 13))
    settle_t  = SETTLE_T

    for col, (d, label) in enumerate([
        (d0, f"Scenario gains  (cost={itae_cost(x0, *cost_args):.3f})"),
        (d1, f"Optimised  (cost={result.fun:.3f})"),
    ]):
        t = d["t"]

        # ── Row 0: X tracking ─────────────────────────────────────────────────
        axes[0, col].plot(t, d["target_x"], "k:",  lw=1.0, label="target x")
        axes[0, col].plot(t, d["ref_x"],    "k--", lw=1.0, label="ref x (standoff)")
        axes[0, col].plot(t, d["x"],        "b-",  lw=1.5, label="drone x")
        axes[0, col].axvline(settle_t, color="gray", lw=0.8, ls="--", label="settle")
        axes[0, col].set_ylabel("x [m]")
        axes[0, col].set_title(label)
        axes[0, col].legend(fontsize=7)
        axes[0, col].grid(True)

        # ── Row 1: Y tracking ─────────────────────────────────────────────────
        axes[1, col].plot(t, d["target_y"], "k:",  lw=1.0, label="target y")
        axes[1, col].plot(t, d["ref_y"],    "k--", lw=1.0, label="ref y (standoff)")
        axes[1, col].plot(t, d["y"],        "g-",  lw=1.5, label="drone y")
        axes[1, col].axvline(settle_t, color="gray", lw=0.8, ls="--")
        axes[1, col].set_ylabel("y [m]")
        axes[1, col].legend(fontsize=7)
        axes[1, col].grid(True)

        # ── Row 2: Roll / pitch rate ──────────────────────────────────────────
        axes[2, col].plot(t, d["wx_cmd"], "r--", lw=1.0, label="wx_cmd (roll)")
        axes[2, col].plot(t, d["wx"],     "r-",  lw=1.5, label="wx actual")
        axes[2, col].plot(t, d["wy_cmd"], "m--", lw=1.0, label="wy_cmd (pitch)")
        axes[2, col].plot(t, d["wy"],     "m-",  lw=1.5, label="wy actual")
        axes[2, col].axvline(settle_t, color="gray", lw=0.8, ls="--")
        axes[2, col].set_ylabel("roll/pitch rate [rad/s]")
        axes[2, col].set_title(f"Roll & Pitch  (wn={wn} rad/s)")
        axes[2, col].legend(fontsize=7)
        axes[2, col].grid(True)

        # ── Row 3: Yaw rate ───────────────────────────────────────────────────
        axes[3, col].plot(t, d["wz_cmd"], "b--", lw=1.0, label="wz_cmd (yaw)")
        axes[3, col].plot(t, d["wz"],     "b-",  lw=1.5, label="wz actual")
        axes[3, col].axvline(settle_t, color="gray", lw=0.8, ls="--")
        axes[3, col].set_xlabel("Time [s]")
        axes[3, col].set_ylabel("yaw rate [rad/s]")
        axes[3, col].set_title(f"Yaw  (wn={wn_yaw} rad/s — reaction torque limited)")
        axes[3, col].legend(fontsize=7)
        axes[3, col].grid(True)

    plt.suptitle(
        f"{os.path.basename(cfg_path)}  |  "
        f"roll/pitch wn={wn} rad/s  yaw wn={wn_yaw} rad/s  zeta={zeta}  |  "
        f"standoff={desired_dist} m  settle={SETTLE_T} s  opt_window={args.duration} s"
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
