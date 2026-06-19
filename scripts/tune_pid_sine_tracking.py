#!/usr/bin/env python3
"""
PX4-style cascade PID tuner for Vista-Tracker (sinusoidal reference).

Optimises gains via ITAE on a sinusoidal tracking trajectory using a
position-velocity cascade:

  Outer loop (position P):   vel_sp = kp_pos * pos_err + ref_vel  (feedforward)
  Inner loop (velocity PID): accel  = kp_vel * vel_err + ki_vel * ∫vel_err

Cost = ITAE(x error + y error) + SMOOTHING * integral(wx_cmd² + wy_cmd²)

Usage:
  python3 scripts/tune_pid_sine_tracking.py              # optimise and show plot
  python3 scripts/tune_pid_sine_tracking.py --no-plot    # just print gains
"""

from __future__ import annotations

import argparse
import numpy as np
from scipy.optimize import minimize, Bounds
import matplotlib.pyplot as plt

# ── Quadrotor physical parameters ────────────────────────────────────────────
G      = 9.81   # m/s²

# ── Body-rate second-order response ──────────────────────────────────────────
WN       = 25.0   # roll/pitch natural frequency (rad/s)
ZETA     =  0.7
WN_YAW   =  4.0   # yaw natural frequency (rad/s)
ZETA_YAW =  0.7

# ── Simulation ────────────────────────────────────────────────────────────────
DT       = 0.05
N_INNER  = 50
DT_INNER = DT / N_INNER
T        = 8.0
N        = int(T / DT)

MAX_ANGLE  = 0.5   # rad
MAX_THRUST = 2.0

# ── Tracking reference ────────────────────────────────────────────────────────
REF_AMP   = 3.0   # m
REF_OMEGA = 0.6   # rad/s

# ── Smoothing weight ──────────────────────────────────────────────────────────
SMOOTHING = 0.10


def simulate(gains: np.ndarray) -> dict[str, np.ndarray]:
    """
    Simulate cascade system tracking sinusoidal references.
    gains = [kp_pos, kp_vel, ki_vel, att_kp, yaw_kp]
    """
    kp_pos, kp_vel, ki_vel, att_kp, yaw_kp = gains

    x, y, z    = 0.0, 0.0, 2.0
    vx, vy, vz = 0.0, 0.0, 0.0
    roll = pitch = yaw = 0.0

    wx,  wx_dot  = 0.0, 0.0
    wy,  wy_dot  = 0.0, 0.0
    wz,  wz_dot  = 0.0, 0.0
    thr, thr_dot = 1.0, 0.0

    # Velocity-error integrals
    ivx = ivy = ivz = 0.0

    wn2_rp   = WN * WN;         damp_rp  = 2.0 * ZETA     * WN
    wn2_yaw  = WN_YAW * WN_YAW; damp_yaw = 2.0 * ZETA_YAW * WN_YAW

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

        ref_x = REF_AMP * np.sin(REF_OMEGA * t)
        ref_y = REF_AMP * np.sin(REF_OMEGA * t + np.pi / 2)
        ref_z = 2.0

        # Analytic reference velocity (derivative of sinusoidal reference)
        ref_vx = REF_AMP * REF_OMEGA * np.cos(REF_OMEGA * t)
        ref_vy = REF_AMP * REF_OMEGA * np.cos(REF_OMEGA * t + np.pi / 2)

        # ── Outer loop: position P + velocity feedforward ─────────────────────
        ex = ref_x - x;  ey = ref_y - y;  ez = ref_z - z
        vx_sp = kp_pos * ex + ref_vx
        vy_sp = kp_pos * ey + ref_vy
        vz_sp = kp_pos * ez

        # ── Inner loop: velocity PID ──────────────────────────────────────────
        vel_err_x = vx_sp - vx
        vel_err_y = vy_sp - vy
        vel_err_z = vz_sp - vz
        ivx += vel_err_x * DT
        ivy += vel_err_y * DT
        ivz += vel_err_z * DT
        ax_des = kp_vel * vel_err_x + ki_vel * ivx
        ay_des = kp_vel * vel_err_y + ki_vel * ivy
        az_des = kp_vel * vel_err_z + ki_vel * ivz

        # ── Attitude setpoints ────────────────────────────────────────────────
        cy_now = np.cos(yaw);  sy_now = np.sin(yaw)
        ax_body =  ax_des * cy_now + ay_des * sy_now
        ay_body = -ax_des * sy_now + ay_des * cy_now
        pitch_des = float(np.clip(np.arctan2(ax_body, G), -MAX_ANGLE, MAX_ANGLE))
        roll_des  = float(np.clip(-np.arctan2(ay_body, G), -MAX_ANGLE, MAX_ANGLE))
        thr_cmd   = float(np.clip((G + az_des) / G, 0.0, MAX_THRUST))

        yaw_des = float(np.arctan2(ref_y - y, ref_x - x))
        yaw_err = (yaw_des - yaw + np.pi) % (2.0 * np.pi) - np.pi

        # ── Inner loop ────────────────────────────────────────────────────────
        roll_rate_cmd  = att_kp * (roll_des  - roll)
        pitch_rate_cmd = att_kp * (pitch_des - pitch)
        yaw_rate_cmd   = yaw_kp * yaw_err

        wx,  wx_dot  = step2_rp(roll_rate_cmd,  wx,  wx_dot)
        wy,  wy_dot  = step2_rp(pitch_rate_cmd, wy,  wy_dot)
        wz,  wz_dot  = step2_yaw(yaw_rate_cmd,  wz,  wz_dot)
        thr, thr_dot = step2_rp(thr_cmd,        thr, thr_dot)

        roll  += wx * DT;  pitch += wy * DT;  yaw += wz * DT

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
    if gains[0] <= 0 or gains[1] <= 0 or gains[3] <= 0 or gains[4] <= 0:
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

    # gains = [kp_pos, kp_vel, ki_vel, att_kp, yaw_kp]
    x0     = np.array([1.5, 5.0, 0.2, 8.0, 2.0])
    bounds = Bounds(lb=[0.1, 0.5, 0.0, 1.0,  0.1],
                    ub=[10.0, 30.0, 5.0, 25.0, 8.0])

    print("Optimising PID gains (ITAE criterion) …")
    result = minimize(itae_cost, x0, method="L-BFGS-B", bounds=bounds,
                      options={"maxiter": 500, "ftol": 1e-9})

    kp_pos, kp_vel, ki_vel, att_kp, yaw_kp = result.x
    print(f"\n── Optimised gains ────────────────────────────────")
    print(f"  kp_pos      : {kp_pos:.4f}  (outer position loop P)")
    print(f"  kp_vel      : {kp_vel:.4f}  (inner velocity loop P)")
    print(f"  ki_vel      : {ki_vel:.4f}  (inner velocity loop I)")
    print(f"  attitude_kp : {att_kp:.4f}  (roll/pitch inner loop)")
    print(f"  yaw_kp      : {yaw_kp:.4f}  (yaw inner loop — wn={WN_YAW} rad/s plant)")
    print(f"  ITAE cost   : {result.fun:.6f}")
    print(f"\n── Paste into config/base.yaml ────────────────────")
    print(f"controller:")
    print(f"  kp_pos:      {kp_pos:.3f}")
    print(f"  kp_vel:      {kp_vel:.3f}")
    print(f"  ki_vel:      {ki_vel:.3f}")
    print(f"  attitude_kp: {att_kp:.3f}")
    print(f"  yaw_kp:      {yaw_kp:.3f}")

    if not args.no_plot:
        d0 = simulate(np.array([1.5, 5.0, 0.2, 8.0, 2.0]))
        d1 = simulate(result.x)

        fig, axes = plt.subplots(4, 2, figsize=(13, 13))

        for col, (d, label) in enumerate([
            (d0, "Initial gains"),
            (d1, f"Optimised (cost={result.fun:.3f})"),
        ]):
            t = d["t"]

            axes[0, col].plot(t, d["ref_x"], "k--", lw=1,   label="ref x")
            axes[0, col].plot(t, d["x"],     "b-",  lw=1.5, label="x")
            axes[0, col].set_ylabel("x [m]")
            axes[0, col].set_title(label)
            axes[0, col].legend(fontsize=8)
            axes[0, col].grid(True)

            axes[1, col].plot(t, d["ref_y"], "k--", lw=1,   label="ref y")
            axes[1, col].plot(t, d["y"],     "g-",  lw=1.5, label="y")
            axes[1, col].set_ylabel("y [m]")
            axes[1, col].legend(fontsize=8)
            axes[1, col].grid(True)

            axes[2, col].plot(t, d["wx_cmd"], "r--", lw=1.0, label="wx_cmd (roll)")
            axes[2, col].plot(t, d["wx"],     "r-",  lw=1.5, label="wx actual")
            axes[2, col].plot(t, d["wy_cmd"], "m--", lw=1.0, label="wy_cmd (pitch)")
            axes[2, col].plot(t, d["wy"],     "m-",  lw=1.5, label="wy actual")
            axes[2, col].set_ylabel("roll/pitch rate [rad/s]")
            axes[2, col].set_title(f"Roll & Pitch  (wn={WN} rad/s)")
            axes[2, col].legend(fontsize=7)
            axes[2, col].grid(True)

            axes[3, col].plot(t, d["wz_cmd"], "b--", lw=1.0, label="wz_cmd (yaw)")
            axes[3, col].plot(t, d["wz"],     "b-",  lw=1.5, label="wz actual")
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
