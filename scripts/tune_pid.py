#!/usr/bin/env python3
"""
Cascade PID tuner for Vista-Tracker.

Simulates the full cascade controller + second-order quadrotor dynamics,
optimises gains via ITAE (Integral Time Absolute Error) on a position step,
and prints the recommended YAML values.

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
WN   = 25.0   # natural frequency  (rad/s)
ZETA =  0.7   # damping ratio      (–)

# ── Simulation ────────────────────────────────────────────────────────────────
DT   = 0.02   # outer-loop timestep (s)  — matches sim.dt in config
T    = 8.0    # total time (s)
N    = int(T / DT)

MAX_ANGLE  = 0.5   # rad  (~28°) tilt limit
MAX_THRUST = 2.0   # normalised


def simulate(gains: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simulate the cascade system for a step reference in x.
    Returns (t, x_log, ref_log) arrays.
    """
    kp, ki, kd, att_kp = gains

    # Position / velocity states
    x, y, z    = 0.0, 0.0, 2.0
    vx, vy, vz = 0.0, 0.0, 0.0

    # Attitude
    roll = pitch = yaw = 0.0

    # Second-order rate states
    wx,  wx_dot  = 0.0, 0.0
    wy,  wy_dot  = 0.0, 0.0
    thr, thr_dot = 1.0, 0.0   # start at hover

    # PID integrals and previous errors
    ix = iy = iz = 0.0
    prev_ex = prev_ey = prev_ez = 0.0

    wn2 = WN * WN
    two_zeta_wn = 2.0 * ZETA * WN

    def step2(cmd, val, dval):
        ddval = wn2 * (cmd - val) - two_zeta_wn * dval
        dval += ddval * DT
        val  += dval  * DT
        return val, dval

    t_log   = np.zeros(N)
    x_log   = np.zeros(N)
    ref_log = np.zeros(N)

    for i in range(N):
        t = i * DT

        # Step reference: move to x=5 m at t=0.5 s, hold
        ref_x = 5.0 if t >= 0.5 else 0.0
        ref_y = 0.0
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
        pitch_des = float(np.clip(np.arctan2(ax_des, G), -MAX_ANGLE, MAX_ANGLE))
        roll_des  = float(np.clip(-np.arctan2(ay_des, G), -MAX_ANGLE, MAX_ANGLE))
        thr_cmd   = float(np.clip((G + az_des) / G, 0.0, MAX_THRUST))

        # ── Inner loop ────────────────────────────────────────────────────────
        roll_rate_cmd  = att_kp * (roll_des  - roll)
        pitch_rate_cmd = att_kp * (pitch_des - pitch)

        # ── Second-order plant ────────────────────────────────────────────────
        wx,  wx_dot  = step2(roll_rate_cmd,  wx,  wx_dot)
        wy,  wy_dot  = step2(pitch_rate_cmd, wy,  wy_dot)
        thr, thr_dot = step2(thr_cmd,        thr, thr_dot)

        # ── Attitude integration ──────────────────────────────────────────────
        roll  += wx * DT
        pitch += wy * DT

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

        t_log[i]   = t
        x_log[i]   = x
        ref_log[i] = ref_x

    return t_log, x_log, ref_log


def itae_cost(gains: np.ndarray) -> float:
    """ITAE on x-position error (penalises late errors more)."""
    if np.any(gains <= 0):
        return 1e9
    t, x, ref = simulate(gains)
    error = np.abs(x - ref)
    return float(np.trapz(t * error, t))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    # ── Optimise ──────────────────────────────────────────────────────────────
    x0     = np.array([1.0, 0.2, 0.3, 8.0])   # kp, ki, kd, attitude_kp
    bounds = Bounds(lb=[0.1, 0.0, 0.0, 1.0],
                    ub=[6.0, 2.0, 3.0, 20.0])

    print("Optimising PID gains (ITAE criterion) …")
    result = minimize(itae_cost, x0, method="L-BFGS-B", bounds=bounds,
                      options={"maxiter": 500, "ftol": 1e-9})

    kp, ki, kd, att_kp = result.x
    print(f"\n── Optimised gains ────────────────────────────────")
    print(f"  kp          : {kp:.4f}")
    print(f"  ki          : {ki:.4f}")
    print(f"  kd          : {kd:.4f}")
    print(f"  attitude_kp : {att_kp:.4f}")
    print(f"  ITAE cost   : {result.fun:.6f}")
    print(f"\n── Paste into config/base.yaml ────────────────────")
    print(f"controller:")
    print(f"  kp:          {kp:.3f}")
    print(f"  ki:          {ki:.3f}")
    print(f"  kd:          {kd:.3f}")
    print(f"  attitude_kp: {att_kp:.3f}")

    if not args.no_plot:
        t0, x0_sim, ref0 = simulate(np.array([1.0, 0.2, 0.3, 8.0]))  # initial
        t1, x1_sim, ref1 = simulate(result.x)                          # optimised

        fig, axes = plt.subplots(1, 2, figsize=(12, 4))

        for ax, t, xs, label in [
            (axes[0], t0, x0_sim, "Initial gains"),
            (axes[1], t1, x1_sim, f"Optimised (ITAE={result.fun:.3f})"),
        ]:
            ax.plot(t, ref1, "k--", linewidth=1, label="Reference")
            ax.plot(t, xs,   "b-",  linewidth=1.5, label=label)
            ax.set_xlabel("Time [s]")
            ax.set_ylabel("x [m]")
            ax.set_title(label)
            ax.legend(fontsize=8)
            ax.grid(True)

        plt.suptitle(f"Position step response  |  wn={WN} rad/s  zeta={ZETA}")
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
