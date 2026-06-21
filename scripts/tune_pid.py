#!/usr/bin/env python3
"""
Scenario-based PX4-style cascade PID tuner for Vista-Tracker.

Reads a scenario YAML and optimises PID gains against the actual target
trajectory using a position-velocity cascade:

  Outer loop (position PI):  vel_sp = kp_pos * pos_err + ki_pos * ∫pos_err
  Inner loop (velocity PID): accel  = kp_vel * vel_err + ki_vel * ∫vel_err

The reference position is computed online from the drone's current position,
mirroring RRTPIDPlanner::computeGoal() exactly:
  ref = target + (drone - target) * desired_dist / dist(drone, target)

Velocity feedforward is intentionally omitted (FF_SCALE = 0) to simulate
worst-case planner lag (discrete waypoints carry no velocity info).  This
forces the optimizer to find non-zero integral gains that eliminate the
≈ V/kp_pos steady-state position error observed without them.

No planner is used. The drone starts at the initial standoff position.
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
SMOOTHING    = 0.10   # control-effort penalty weight (angular-rate effort)
GAIN_REG     = 2.00   # gain-regularisation weight — penalises large PI gains to
                      # create an interior optimum; targets ki_pos ≈ 0.5-1.2 (near
                      # critically damped with kp_pos ≈ 2 → ζ = kp_pos/(2√ki_pos))
SETTLE_T     = 3.0    # seconds excluded from cost (initial yaw + integrator settling)
OPT_DURATION = 25.0   # optimisation window (s) — full scenario used for plot only
N_INNER      = 50     # inner sub-steps per outer dt for 2nd-order dynamics
MAX_ANGLE      = 0.5   # rad tilt limit
MAX_THRUST     = 2.0   # normalised
MAX_IPOS_CONT  = 1.0   # max m/s contribution from position integral (anti-windup)
MAX_IVEL_CONT  = 4.0   # max m/s² contribution from velocity integral (anti-windup)

# Velocity feedforward scale.  Models the reduced effective feedforward that
# arises from the RRT planner's replan hysteresis: static waypoints consume
# the feedforward before it can do its job, so the drone must integrate its
# way out of the residual SS error.  0.5 = 50% effective feedforward, which
# creates a ≈ 0.5*target_speed/kp_pos SS position error that ki_pos must
# eliminate.  The real system retains full feedforward, so ki_pos acts
# conservatively there (integral stays near zero when ex ≈ 0 at full FF).
FF_SCALE = 0.5

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
_REACH_TOL = 0.1   # m
_MIN_SPEED  = 0.05  # m/s


def _simulate_target(wps: list, traj: dict, dt: float, n: int):
    """
    Returns (tx, ty, tz, tvx, tvy) arrays, each of length n.
    Exact Python mirror of WaypointFollower::step().
    """
    tx  = np.empty(n); ty  = np.empty(n); tz  = np.empty(n)
    tvx = np.zeros(n); tvy = np.zeros(n)

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
            tvx[i] = tvy[i] = 0.0
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
            tvx[i] = tvy[i] = 0.0
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
            tvx[i] = tvy[i] = 0.0
            continue

        cur_speed = np.sqrt(svx*svx + svy*svy + svz*svz)
        v_target  = min(wp_speed, np.sqrt(2.0 * max_accel * dist))
        dv        = np.clip(v_target - cur_speed, -max_accel * dt, max_accel * dt)
        new_speed = max(0.0, cur_speed + dv)

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
        tvx[i] = svx
        tvy[i] = svy

    return tx, ty, tz, tvx, tvy


# ── Cascade PI/PID simulation ─────────────────────────────────────────────────

def simulate(
    gains: np.ndarray,
    tx: np.ndarray, ty: np.ndarray,
    tvx: np.ndarray, tvy: np.ndarray,
    init_x: float, init_y: float,
    z_ref: float, desired_dist: float, dt: float,
    wn: float, zeta: float, wn_yaw: float, zeta_yaw: float,
) -> dict:
    """
    Simulate cascade tracking with online standoff-reference computation.

    Reference mirrors RRTPIDPlanner::computeGoal():
        ref = target + (drone - target) * desired_dist / dist(drone, target)

    Velocity feedforward is scaled by FF_SCALE (0 = none, models planner lag).

    gains = [kp_pos, ki_pos, kp_vel, ki_vel, att_kp, yaw_kp]
    """
    kp_pos, ki_pos, kp_vel, ki_vel, att_kp, yaw_kp = gains
    n        = len(tx)
    dt_inner = dt / N_INNER

    wn2_rp   = wn * wn;           damp_rp  = 2.0 * zeta     * wn
    wn2_yaw  = wn_yaw * wn_yaw;   damp_yaw = 2.0 * zeta_yaw * wn_yaw

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
    thr, thr_dot = 1.0, 0.0

    # Outer-loop position-error integrals
    ipx = ipy = ipz = 0.0
    # Inner-loop velocity-error integrals
    ivx = ivy = ivz = 0.0

    x_log      = np.empty(n)
    y_log      = np.empty(n)
    ref_x_log  = np.empty(n)
    ref_y_log  = np.empty(n)
    err_x_log  = np.empty(n)
    err_y_log  = np.empty(n)
    wx_cmd_log = np.empty(n)
    wy_cmd_log = np.empty(n)
    wz_cmd_log = np.empty(n)
    wx_log     = np.empty(n)
    wy_log     = np.empty(n)
    wz_log     = np.empty(n)

    for i in range(n):
        # ── Online standoff reference (mirrors RRTPIDPlanner::computeGoal) ───
        dx_dt = x - tx[i]
        dy_dt = y - ty[i]
        d_t   = np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt)
        if d_t > 1e-6:
            s     = desired_dist / d_t
            ref_x = tx[i] + dx_dt * s
            ref_y = ty[i] + dy_dt * s
        else:
            ref_x = tx[i] + desired_dist
            ref_y = ty[i]

        # Velocity feedforward (scaled — FF_SCALE=0 models no feedforward)
        ref_vx = FF_SCALE * tvx[i]
        ref_vy = FF_SCALE * tvy[i]

        # ── Outer loop: position PI + velocity feedforward ───────────────────
        ex = ref_x - x;   ey = ref_y - y;   ez = z_ref - z
        ipx += ex * dt;   ipy += ey * dt;   ipz += ez * dt
        # Anti-windup: clamp position integral so vel contribution ≤ MAX_IPOS_CONT
        if ki_pos > 1e-9:
            _lim = MAX_IPOS_CONT / ki_pos
            ipx = float(np.clip(ipx, -_lim, _lim))
            ipy = float(np.clip(ipy, -_lim, _lim))
            ipz = float(np.clip(ipz, -_lim, _lim))
        vx_sp = kp_pos * ex + ki_pos * ipx + ref_vx
        vy_sp = kp_pos * ey + ki_pos * ipy + ref_vy
        vz_sp = kp_pos * ez + ki_pos * ipz

        # ── Inner loop: velocity PID ─────────────────────────────────────────
        vel_err_x = vx_sp - vx
        vel_err_y = vy_sp - vy
        vel_err_z = vz_sp - vz
        ivx += vel_err_x * dt
        ivy += vel_err_y * dt
        ivz += vel_err_z * dt
        # Anti-windup: clamp velocity integral so accel contribution ≤ MAX_IVEL_CONT
        if ki_vel > 1e-9:
            _vlim = MAX_IVEL_CONT / ki_vel
            ivx = float(np.clip(ivx, -_vlim, _vlim))
            ivy = float(np.clip(ivy, -_vlim, _vlim))
            ivz = float(np.clip(ivz, -_vlim, _vlim))
        ax_des = kp_vel * vel_err_x + ki_vel * ivx
        ay_des = kp_vel * vel_err_y + ki_vel * ivy
        az_des = kp_vel * vel_err_z + ki_vel * ivz

        # ── Attitude setpoints (body-frame rotation) ─────────────────────────
        cy_now = np.cos(yaw);  sy_now = np.sin(yaw)
        ax_b   =  ax_des * cy_now + ay_des * sy_now
        ay_b   = -ax_des * sy_now + ay_des * cy_now
        pitch_des = float(np.clip(np.arctan2(ax_b, G), -MAX_ANGLE, MAX_ANGLE))
        roll_des  = float(np.clip(-np.arctan2(ay_b, G), -MAX_ANGLE, MAX_ANGLE))
        thr_cmd   = float(np.clip((G + az_des) / G, 0.0, MAX_THRUST))

        # Yaw: face the target
        yaw_des = float(np.arctan2(ty[i] - y, tx[i] - x))
        yaw_err = (yaw_des - yaw + np.pi) % (2.0 * np.pi) - np.pi

        # ── Attitude inner loop ───────────────────────────────────────────────
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
        err_x_log[i]  = ex;            err_y_log[i]  = ey
        wx_cmd_log[i] = roll_rate_cmd;  wy_cmd_log[i] = pitch_rate_cmd
        wz_cmd_log[i] = yaw_rate_cmd
        wx_log[i]     = wx;             wy_log[i]     = wy;  wz_log[i] = wz

    t = np.arange(n) * dt
    return {
        "t": t,
        "x": x_log,        "y": y_log,
        "ref_x": ref_x_log, "ref_y": ref_y_log,
        "err_x": err_x_log, "err_y": err_y_log,
        "target_x": tx,    "target_y": ty,
        "wx_cmd": wx_cmd_log, "wy_cmd": wy_cmd_log, "wz_cmd": wz_cmd_log,
        "wx": wx_log,      "wy": wy_log,      "wz": wz_log,
    }


# ── Cost function ─────────────────────────────────────────────────────────────

def itae_cost(
    gains: np.ndarray,
    tx: np.ndarray, ty: np.ndarray,
    tvx: np.ndarray, tvy: np.ndarray,
    init_x: float, init_y: float,
    z_ref: float, desired_dist: float, dt: float, settle_idx: int,
    wn: float, zeta: float, wn_yaw: float, zeta_yaw: float,
) -> float:
    kp_pos, ki_pos, kp_vel, ki_vel, att_kp, yaw_kp = gains
    if kp_pos <= 0 or kp_vel <= 0 or att_kp <= 0 or yaw_kp <= 0:
        return 1e9
    if ki_pos < 0 or ki_vel < 0:
        return 1e9
    d = simulate(gains, tx, ty, tvx, tvy, init_x, init_y, z_ref, desired_dist, dt,
                 wn, zeta, wn_yaw, zeta_yaw)
    t      = d["t"][settle_idx:]
    ex     = d["err_x"][settle_idx:]
    ey     = d["err_y"][settle_idx:]
    wx_cmd = d["wx_cmd"][settle_idx:]
    wy_cmd = d["wy_cmd"][settle_idx:]
    t0      = t[0]
    itae    = float(np.trapz((t - t0) * (np.abs(ex) + np.abs(ey)), t))
    effort  = float(np.trapz(wx_cmd**2 + wy_cmd**2, t))
    # Regularise position-loop gains — without this, ITAE is monotone in
    # the gains (faster → lower ITAE) and the optimizer saturates at bounds.
    gain_pen = GAIN_REG * (kp_pos**2 + ki_pos**2)
    return itae + SMOOTHING * effort + gain_pen


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

    # ── Pre-simulate target ───────────────────────────────────────────────────
    N_full     = int(T_full / dt)
    n_opt      = min(N_full, max(1, int(args.duration / dt)))
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
          f"  settle={SETTLE_T} s  full={T_full} s  FF_SCALE={FF_SCALE}")

    print("\nPre-simulating target trajectory …")
    tx_full, ty_full, tz_full, tvx_full, tvy_full = _simulate_target(wps, traj, dt, N_full)

    tx_opt  = tx_full[:n_opt];   ty_opt  = ty_full[:n_opt]
    tvx_opt = tvx_full[:n_opt];  tvy_opt = tvy_full[:n_opt]

    cost_args = (tx_opt, ty_opt, tvx_opt, tvy_opt,
                 init_x, init_y, z_ref, desired_dist,
                 dt, settle_idx, wn, zeta, wn_yaw, zeta_yaw)

    # ── Optimise ──────────────────────────────────────────────────────────────
    ctrl = cfg.get("controller", {})
    x0 = np.array([
        ctrl.get("kp_pos",      1.6),
        ctrl.get("ki_pos",      0.3),   # warm-start — must be >0 to escape saddle
        ctrl.get("kp_vel",      5.0),
        ctrl.get("ki_vel",      0.2),
        ctrl.get("attitude_kp", 6.0),
        ctrl.get("yaw_kp",      0.3),
    ])
    bounds = Bounds(lb=[0.5, 0.0, 0.5, 0.0, 1.0,  0.15],
                    ub=[6.0, 5.0, 20.0, 5.0, 25.0, 5.0])

    print(f"\nInitial cost : {itae_cost(x0, *cost_args):.4f}")
    print("Optimising (L-BFGS-B) …")
    result = minimize(
        itae_cost, x0, args=cost_args,
        method="L-BFGS-B", bounds=bounds,
        options={"maxiter": 500, "ftol": 1e-9},
    )

    kp_pos, ki_pos, kp_vel, ki_vel, att_kp, yaw_kp = result.x
    print(f"\n── Optimised gains ────────────────────────────────────────────")
    print(f"  kp_pos      : {kp_pos:.4f}  (outer position loop P)")
    print(f"  ki_pos      : {ki_pos:.4f}  (outer position loop I — eliminates SS error)")
    print(f"  kp_vel      : {kp_vel:.4f}  (inner velocity loop P)")
    print(f"  ki_vel      : {ki_vel:.4f}  (inner velocity loop I)")
    print(f"  attitude_kp : {att_kp:.4f}  (roll/pitch inner loop)")
    print(f"  yaw_kp      : {yaw_kp:.4f}  (yaw inner loop — wn={wn_yaw} rad/s plant)")
    print(f"  ITAE cost   : {result.fun:.6f}")
    print(f"\n── Paste into {os.path.basename(cfg_path)} ──────────────────────────")
    print(f"controller:")
    print(f"  kp_pos:      {kp_pos:.3f}")
    print(f"  ki_pos:      {ki_pos:.3f}")
    print(f"  kp_vel:      {kp_vel:.3f}")
    print(f"  ki_vel:      {ki_vel:.3f}")
    print(f"  attitude_kp: {att_kp:.3f}")
    print(f"  yaw_kp:      {yaw_kp:.3f}")

    if args.no_plot:
        return

    # ── Plot: run both gain sets over full scenario duration ──────────────────
    sim_args = (tx_full, ty_full, tvx_full, tvy_full,
                init_x, init_y, z_ref, desired_dist,
                dt, wn, zeta, wn_yaw, zeta_yaw)

    d0 = simulate(x0,       *sim_args)
    d1 = simulate(result.x, *sim_args)

    cost_args_full = (tx_full, ty_full, tvx_full, tvy_full,
                      init_x, init_y, z_ref, desired_dist,
                      dt, settle_idx, wn, zeta, wn_yaw, zeta_yaw)

    fig, axes = plt.subplots(4, 2, figsize=(13, 13))
    settle_t  = SETTLE_T

    for col, (d, label) in enumerate([
        (d0, f"Scenario gains  (cost={itae_cost(x0, *cost_args_full):.3f})"),
        (d1, f"Optimised  (cost={result.fun:.3f})"),
    ]):
        t = d["t"]

        axes[0, col].plot(t, d["target_x"], "k:",  lw=1.0, label="target x")
        axes[0, col].plot(t, d["ref_x"],    "k--", lw=1.0, label="ref x (standoff)")
        axes[0, col].plot(t, d["x"],        "b-",  lw=1.5, label="drone x")
        axes[0, col].axvline(settle_t, color="gray", lw=0.8, ls="--", label="settle")
        axes[0, col].set_ylabel("x [m]")
        axes[0, col].set_title(label)
        axes[0, col].legend(fontsize=7)
        axes[0, col].grid(True)

        axes[1, col].plot(t, d["target_y"], "k:",  lw=1.0, label="target y")
        axes[1, col].plot(t, d["ref_y"],    "k--", lw=1.0, label="ref y (standoff)")
        axes[1, col].plot(t, d["y"],        "g-",  lw=1.5, label="drone y")
        axes[1, col].axvline(settle_t, color="gray", lw=0.8, ls="--")
        axes[1, col].set_ylabel("y [m]")
        axes[1, col].legend(fontsize=7)
        axes[1, col].grid(True)

        axes[2, col].plot(t, d["wx_cmd"], "r--", lw=1.0, label="wx_cmd (roll)")
        axes[2, col].plot(t, d["wx"],     "r-",  lw=1.5, label="wx actual")
        axes[2, col].plot(t, d["wy_cmd"], "m--", lw=1.0, label="wy_cmd (pitch)")
        axes[2, col].plot(t, d["wy"],     "m-",  lw=1.5, label="wy actual")
        axes[2, col].axvline(settle_t, color="gray", lw=0.8, ls="--")
        axes[2, col].set_ylabel("roll/pitch rate [rad/s]")
        axes[2, col].set_title(f"Roll & Pitch  (wn={wn} rad/s)")
        axes[2, col].legend(fontsize=7)
        axes[2, col].grid(True)

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
        f"standoff={desired_dist} m  settle={SETTLE_T} s  opt_window={args.duration} s  "
        f"FF_SCALE={FF_SCALE}"
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
