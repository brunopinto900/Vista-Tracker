#!/usr/bin/env python3
"""
Cascade PID tuner for Vista-Tracker.

Simulates the full cascade controller + second-order quadrotor dynamics,
optimises gains via ITAE (Integral Time Absolute Error) on a sinusoidal
tracking trajectory, plus a control-effort smoothing penalty.

Cost = ITAE(x error + y error) + SMOOTHING * integral(wx_cmd² + wy_cmd²)

Increase SMOOTHING to trade tracking tightness for softer commands.

Model:
  Outer loop : position PID  →  desired acceleration
  Conversion : accel         →  attitude setpoints  (atan2 / g)
  Inner loop : attitude P    →  body-rate commands
  Plant      : second-order body-rate response (wn, zeta)
  Dynamics   : ZYX Euler attitude + rotation matrix + gravity

Usage:
  python3 scripts/tune_pid.py              # optimise and show plot
  python3 scripts/tune_pid.py --no-plot    # just print gains
"""

from __future__ import annotations

import argparse
import numpy as np
from scipy.optimize import minimize, Bounds
import matplotlib.pyplot as plt

# ── Quadrotor physical parameters (250 mm class) ─────────────────────────────
MASS   = 0.5    # kg
G      = 9.81   # m/s²

# ── Body-rate second-order response ──────────────────────────────────────────
# Roll/pitch: differential thrust → high authority
WN      = 25.0   # natural frequency  (rad/s)
ZETA    =  0.7   # damping ratio      (–)
# Yaw: reaction torque imbalance → much lower authority
WN_YAW   = 4.0   # yaw natural frequency  (rad/s)  τ ≈ 250 ms
ZETA_YAW = 0.7   # yaw damping ratio

# ── Simulation ────────────────────────────────────────────────────────────────
DT       = 0.05    # outer-loop (PID) timestep (s) — matches sim.dt in config
N_INNER  = 50      # inner sub-steps for 2nd-order dynamics
DT_INNER = DT / N_INNER   # = 0.001 s  → wn·dt_inner = 0.025, well inside Euler stability
T        = 8.0    # total time (s)
N        = int(T / DT)

MAX_ANGLE  = 0.5   # rad  (~28°) tilt limit
MAX_THRUST = 2.0   # normalised

# ── Tracking reference (sinusoidal — representative of a moving target) ───────
REF_AMP   = 3.0   # m      amplitude
REF_OMEGA = 0.6   # rad/s  angular frequency  (period ≈ 10 s)

# ── Smoothing weight  (increase to trade tracking error for softer commands) ──
SMOOTHING = 0.10  # multiplied by integral(wx_cmd² + wy_cmd²) — both axes


def simulate(gains: np.ndarray) -> dict[str, np.ndarray]:
    """
    Simulate cascade system tracking sinusoidal references in x and y,
    with camera-facing yaw (atan2 toward reference point).

    Roll/pitch bandwidth: WN=25 rad/s.  Yaw bandwidth: WN_YAW=4 rad/s.
    gains = [kp, ki, kd, att_kp (roll/pitch inner), yaw_kp (yaw inner)]

    Separate yaw_kp is necessary because sharing att_kp between roll/pitch
    (wn=25) and yaw (wn=4) causes the optimizer to compromise both.

    Returns dict with keys:
      t, x, y, ref_x, ref_y,
      wx_cmd, wy_cmd, wz_cmd,   (body-rate commands from controller)
      wx, wy, wz                 (actual body rates after 2nd-order plant)
    """
    kp, ki, kd, att_kp, yaw_kp = gains

    # Position / velocity states
    x, y, z    = 0.0, 0.0, 2.0
    vx, vy, vz = 0.0, 0.0, 0.0

    # Attitude
    roll = pitch = yaw = 0.0

    # Second-order rate states
    wx,  wx_dot  = 0.0, 0.0
    wy,  wy_dot  = 0.0, 0.0
    wz,  wz_dot  = 0.0, 0.0
    thr, thr_dot = 1.0, 0.0   # start at hover

    # PID integrals and previous errors
    ix = iy = iz = 0.0
    prev_ex = prev_ey = prev_ez = 0.0

    # Roll/pitch plant parameters
    wn2_rp  = WN * WN
    damp_rp = 2.0 * ZETA * WN

    # Yaw plant — lower bandwidth due to reaction-torque authority limit
    wn2_yaw  = WN_YAW * WN_YAW
    damp_yaw = 2.0 * ZETA_YAW * WN_YAW

    def step2_rp(cmd, val, dval):
        for _ in range(N_INNER):
            ddval = wn2_rp * (cmd - val) - damp_rp * dval
            dval += ddval * DT_INNER
            val  += dval  * DT_INNER
        return val, dval

    def step2_yaw(cmd, val, dval):
        for _ in range(N_INNER):
            ddval = wn2_yaw * (cmd - val) - damp_yaw * dval
            dval += ddval * DT_INNER
            val  += dval  * DT_INNER
        return val, dval

    t_log      = np.zeros(N)
    x_log      = np.zeros(N)
    y_log      = np.zeros(N)
    ref_x_log  = np.zeros(N)
    ref_y_log  = np.zeros(N)
    wx_cmd_log = np.zeros(N)
    wy_cmd_log = np.zeros(N)
    wz_cmd_log = np.zeros(N)
    wx_log     = np.zeros(N)
    wy_log     = np.zeros(N)
    wz_log     = np.zeros(N)

    for i in range(N):
        t = i * DT

        # Sinusoidal references in both axes — π/2 phase offset avoids identical signals
        ref_x = REF_AMP * np.sin(REF_OMEGA * t)
        ref_y = REF_AMP * np.sin(REF_OMEGA * t + np.pi / 2)
        ref_z = 2.0

        # ── Outer PID ────────────────────────────────────────────────────────
        ex = ref_x - x
        ey = ref_y - y
        ez = ref_z - z

        ix += ex * DT
        iy += ey * DT
        iz += ez * DT

        dex = (ex - prev_ex) / DT if i > 0 else 0.0
        dey = (ey - prev_ey) / DT if i > 0 else 0.0
        dez = (ez - prev_ez) / DT if i > 0 else 0.0

        ax_des = kp * ex + ki * ix + kd * dex
        ay_des = kp * ey + ki * iy + kd * dey
        az_des = kp * ez + ki * iz + kd * dez

        prev_ex, prev_ey, prev_ez = ex, ey, ez

        # ── Attitude setpoints ────────────────────────────────────────────────
        # Rotate world-frame desired acceleration into body horizontal plane so
        # that roll/pitch commands are correct regardless of current yaw angle.
        cy_now = np.cos(yaw)
        sy_now = np.sin(yaw)
        ax_body =  ax_des * cy_now + ay_des * sy_now
        ay_body = -ax_des * sy_now + ay_des * cy_now
        pitch_des = float(np.clip(np.arctan2(ax_body, G), -MAX_ANGLE, MAX_ANGLE))
        roll_des  = float(np.clip(-np.arctan2(ay_body, G), -MAX_ANGLE, MAX_ANGLE))
        thr_cmd   = float(np.clip((G + az_des) / G, 0.0, MAX_THRUST))

        # Camera-facing yaw: point toward the reference point
        yaw_des = float(np.arctan2(ref_y - y, ref_x - x))
        yaw_err = (yaw_des - yaw + np.pi) % (2.0 * np.pi) - np.pi  # wrap to [-π, π]

        # ── Inner loop ────────────────────────────────────────────────────────
        roll_rate_cmd  = att_kp * (roll_des  - roll)
        pitch_rate_cmd = att_kp * (pitch_des - pitch)
        yaw_rate_cmd   = yaw_kp * yaw_err     # separate gain — yaw plant is 6× slower

        # ── Second-order plant (separate dynamics for yaw) ────────────────────
        wx,  wx_dot  = step2_rp(roll_rate_cmd,  wx,  wx_dot)
        wy,  wy_dot  = step2_rp(pitch_rate_cmd, wy,  wy_dot)
        wz,  wz_dot  = step2_yaw(yaw_rate_cmd,  wz,  wz_dot)
        thr, thr_dot = step2_rp(thr_cmd,        thr, thr_dot)

        # ── Attitude integration ──────────────────────────────────────────────
        roll  += wx * DT
        pitch += wy * DT
        yaw   += wz * DT

        # ── World-frame acceleration (ZYX Euler) ─────────────────────────────
        cr, sr = np.cos(roll),  np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw),   np.sin(yaw)
        T_val  = thr * G

        ax = (cy*sp*cr + sy*sr) * T_val
        ay = (sy*sp*cr - cy*sr) * T_val
        az =  cp*cr             * T_val - G

        vx += ax * DT;  x += vx * DT
        vy += ay * DT;  y += vy * DT
        vz += az * DT;  z += vz * DT

        t_log[i]      = t
        x_log[i]      = x
        y_log[i]      = y
        ref_x_log[i]  = ref_x
        ref_y_log[i]  = ref_y
        wx_cmd_log[i] = roll_rate_cmd
        wy_cmd_log[i] = pitch_rate_cmd
        wz_cmd_log[i] = yaw_rate_cmd
        wx_log[i]     = wx
        wy_log[i]     = wy
        wz_log[i]     = wz

    return {
        "t":     t_log,
        "x":     x_log,     "y":     y_log,
        "ref_x": ref_x_log, "ref_y": ref_y_log,
        "wx_cmd": wx_cmd_log, "wy_cmd": wy_cmd_log, "wz_cmd": wz_cmd_log,
        "wx":    wx_log,    "wy":    wy_log,    "wz":    wz_log,
    }


def itae_cost(gains: np.ndarray) -> float:
    """ITAE tracking error + roll/pitch control-effort smoothing penalty.

    cost = integral(t * (|ex| + |ey|)) + SMOOTHING * integral(wx_cmd² + wy_cmd²)

    Yaw effort is excluded: yaw heading is a separate concern from position
    tracking, and including wz_cmd² drives att_kp down (large initial yaw
    errors → large commands), which degrades roll/pitch response as a side effect.
    Yaw lag is still visible in the plots as a system property.
    """
    if gains[0] <= 0 or gains[3] <= 0 or gains[4] <= 0:  # kp, att_kp, yaw_kp must be positive
        return 1e9
    d = simulate(gains)
    t      = d["t"]
    itae   = float(np.trapz(t * (np.abs(d["x"] - d["ref_x"]) + np.abs(d["y"] - d["ref_y"])), t))
    effort = float(np.trapz(d["wx_cmd"]**2 + d["wy_cmd"]**2, t))
    return itae + SMOOTHING * effort


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    # ── Optimise ──────────────────────────────────────────────────────────────
    #   gains = [kp, ki, kd, att_kp (roll/pitch), yaw_kp]
    x0     = np.array([2.0, 0.1, 0.5, 8.0, 2.0])
    bounds = Bounds(lb=[0.1, 0.0, 0.0, 1.0,  0.1],
                    ub=[15.0, 2.0, 5.0, 25.0, 8.0])

    print("Optimising PID gains (ITAE criterion) …")
    result = minimize(itae_cost, x0, method="L-BFGS-B", bounds=bounds,
                      options={"maxiter": 500, "ftol": 1e-9})

    kp, ki, kd, att_kp, yaw_kp = result.x
    print(f"\n── Optimised gains ────────────────────────────────")
    print(f"  kp          : {kp:.4f}")
    print(f"  ki          : {ki:.4f}")
    print(f"  kd          : {kd:.4f}")
    print(f"  attitude_kp : {att_kp:.4f}  (roll/pitch inner loop)")
    print(f"  yaw_kp      : {yaw_kp:.4f}  (yaw inner loop — wn={WN_YAW} rad/s plant)")
    print(f"  ITAE cost   : {result.fun:.6f}")
    print(f"\n── Paste into config/base.yaml ────────────────────")
    print(f"controller:")
    print(f"  kp:          {kp:.3f}")
    print(f"  ki:          {ki:.3f}")
    print(f"  kd:          {kd:.3f}")
    print(f"  attitude_kp: {att_kp:.3f}")
    print(f"  yaw_kp:      {yaw_kp:.3f}")

    if not args.no_plot:
        d0 = simulate(np.array([1.0, 0.2, 0.3, 8.0, 2.0]))   # initial gains
        d1 = simulate(result.x)                           # optimised gains

        fig, axes = plt.subplots(4, 2, figsize=(13, 13))

        for col, (d, label) in enumerate([
            (d0, "Initial gains"),
            (d1, f"Optimised (cost={result.fun:.3f})"),
        ]):
            t = d["t"]

            # ── Row 0: x tracking ──────────────────────────────────────────────
            axes[0, col].plot(t, d["ref_x"], "k--", linewidth=1,   label="ref x")
            axes[0, col].plot(t, d["x"],     "b-",  linewidth=1.5, label="x")
            axes[0, col].set_ylabel("x [m]")
            axes[0, col].set_title(label)
            axes[0, col].legend(fontsize=8)
            axes[0, col].grid(True)

            # ── Row 1: y tracking ──────────────────────────────────────────────
            axes[1, col].plot(t, d["ref_y"], "k--", linewidth=1,   label="ref y")
            axes[1, col].plot(t, d["y"],     "g-",  linewidth=1.5, label="y")
            axes[1, col].set_ylabel("y [m]")
            axes[1, col].legend(fontsize=8)
            axes[1, col].grid(True)

            # ── Row 2: roll & pitch — cmd vs actual ───────────────────────────
            axes[2, col].plot(t, d["wx_cmd"], "r--", linewidth=1.0, label="wx_cmd (roll)")
            axes[2, col].plot(t, d["wx"],     "r-",  linewidth=1.5, label="wx actual")
            axes[2, col].plot(t, d["wy_cmd"], "m--", linewidth=1.0, label="wy_cmd (pitch)")
            axes[2, col].plot(t, d["wy"],     "m-",  linewidth=1.5, label="wy actual")
            axes[2, col].set_ylabel("roll/pitch rate [rad/s]")
            axes[2, col].set_title(f"Roll & Pitch  (wn={WN} rad/s)")
            axes[2, col].legend(fontsize=7)
            axes[2, col].grid(True)

            # ── Row 3: yaw — cmd vs actual ────────────────────────────────────
            axes[3, col].plot(t, d["wz_cmd"], "b--", linewidth=1.0, label="wz_cmd (yaw)")
            axes[3, col].plot(t, d["wz"],     "b-",  linewidth=1.5, label="wz actual")
            axes[3, col].set_xlabel("Time [s]")
            axes[3, col].set_ylabel("yaw rate [rad/s]")
            axes[3, col].set_title(f"Yaw  (wn={WN_YAW} rad/s — reaction torque limited)")
            axes[3, col].legend(fontsize=8)
            axes[3, col].grid(True)

        plt.suptitle(
            f"Tracking  |  roll/pitch wn={WN} rad/s  yaw wn={WN_YAW} rad/s  zeta={ZETA}"
            f"  |  smoothing={SMOOTHING}"
            f"  |  ref: {REF_AMP}m @ {REF_OMEGA:.2f} rad/s"
        )
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
