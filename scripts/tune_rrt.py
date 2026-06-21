#!/usr/bin/env python3
"""
RRT parameter tuner for Vista-Tracker.

Optimises four planning parameters for a given scenario:
  step_size        — RRT extension step (m)
  goal_bias        — fraction of samples directed at goal [0, 1)
  safety_margin    — min ESDF clearance (m)
  replan_goal_dist — replan when standoff goal drifts more than this (m)
  max_iter         — RRT iteration budget per plan call

wp_reach_thresh is read from the scenario config and held fixed.
It is a sequencer parameter (advance before decelerating to avoid stop-and-go)
whose optimal value is well understood and not derivable from geometry metrics.

Cost per evaluation (N_QUERIES × N_SEEDS runs):
  W_FAIL   * failure_rate          — fraction of calls that return no path
  W_EFF    * mean_path_efficiency  — mean(path_length / straight-line dist)
  W_CORNER * corner_unsafe_rate    — fraction of paths where wp_reach_thresh
                                     causes the sequencer to cut a corner that
                                     is not collision-free
  W_REPLAN * replan_interval_cost  — penalises replan intervals outside [1, 6] s
                                     (derived from replan_goal_dist / target speed)
  W_ITER   * iter_cost             — favours smaller iteration budgets
                                     (normalised to MAX_ITER_REF)

replan_goal_dist optimal value depends on target speed — available in the
scenario config, so including it here avoids a separate tuning step.

Usage:
  python3 scripts/tune_rrt.py config/scenarios/urban_block.yaml
  python3 scripts/tune_rrt.py urban_block           # bare name OK
  python3 scripts/tune_rrt.py urban_block --no-plot
"""

from __future__ import annotations

import argparse
import os
import sys

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.optimize import minimize

# ── Tuning constants ──────────────────────────────────────────────────────────

N_QUERIES      = 15     # start/goal pairs, generated once before optimisation
N_SEEDS        = 3      # RRT seeds per query  (smooths stochasticity)
W_FAIL         = 10.0   # failure-rate weight
W_EFF          =  1.0   # path-efficiency weight
W_CORNER       =  8.0   # corner-unsafety weight (nearly as bad as total failure)
W_REPLAN       =  2.0   # replan-interval weight
W_ITER         =  0.3   # iteration-budget weight (favours faster plans)
REPLAN_MIN_S   =  1.0   # ideal minimum replan interval (s)
REPLAN_MAX_S   =  6.0   # ideal maximum replan interval (s)
MIN_MARGIN     =  0.15  # hard floor on safety_margin (m)
MIN_ITER       =  200   # hard floor on max_iter
MAX_ITER_CAP   = 6000   # hard ceiling on max_iter during tuning
MAX_ITER_REF   = 4000   # normalisation reference for iter cost (deployment default)
EDGE_CHECK_RES =  0.1   # fixed collision-check resolution along edge (m)
GOAL_TOL       =  0.5   # fixed goal-reached radius (m)
MIN_QUERY_DIST =  5.0   # minimum start–goal separation (m)
FREE_MARGIN    =  0.5   # min ESDF clearance required at a query point (m)

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


# ── ESDF — exact mirror of FakeESDFMap::query ─────────────────────────────────

def _esdf(x: float, y: float, obstacles: list) -> float:
    min_d = float("inf")
    for o in obstacles:
        dx = abs(x - o["x"]) - o["size"]
        dy = abs(y - o["y"]) - o["size"]
        outside = np.sqrt(max(dx, 0.0) ** 2 + max(dy, 0.0) ** 2)
        inside = min(max(dx, dy), 0.0)
        d = outside + inside
        if d < min_d:
            min_d = d
    return min_d


def _esdf_batch(pts: np.ndarray, obstacles: list) -> np.ndarray:
    """Vectorised ESDF for an (N, 2) array. Returns shape (N,)."""
    min_d = np.full(len(pts), np.inf)
    for o in obstacles:
        dx = np.abs(pts[:, 0] - o["x"]) - o["size"]
        dy = np.abs(pts[:, 1] - o["y"]) - o["size"]
        outside = np.sqrt(np.maximum(dx, 0.0) ** 2 + np.maximum(dy, 0.0) ** 2)
        inside = np.minimum(np.maximum(dx, dy), 0.0)
        np.minimum(min_d, outside + inside, out=min_d)
    return min_d


# ── 2-D RRT — matches RRT.cpp ─────────────────────────────────────────────────

def _rrt_plan(
    start, goal, obstacles,
    step_size: float, goal_bias: float, safety_margin: float,
    x_min: float, x_max: float, y_min: float, y_max: float,
    rng: np.random.Generator,
    max_iter: int = 1000,
):
    """
    Returns (path, min_clearance) on success, (None, None) on failure.
    path is a list of 1-D numpy arrays [x, y].
    """
    max_nodes = max_iter + 2
    nodes     = np.empty((max_nodes, 2), dtype=float)
    parents   = np.full(max_nodes, -1, dtype=int)
    nodes[0]  = start
    n         = 1

    goal_a = np.array(goal, dtype=float)

    for _ in range(max_iter):
        sample = goal_a if rng.random() < goal_bias else np.array(
            [rng.uniform(x_min, x_max), rng.uniform(y_min, y_max)])

        active = nodes[:n]
        diff   = active - sample
        dists  = np.sqrt((diff * diff).sum(axis=1))
        ni     = int(dists.argmin())
        d_near = float(dists[ni])

        if d_near < 1e-9:
            continue

        edge_len  = min(step_size, d_near)
        direction = (sample - nodes[ni]) / d_near
        new_pt    = nodes[ni] + direction * edge_len

        if _esdf(float(new_pt[0]), float(new_pt[1]), obstacles) < safety_margin:
            continue

        n_check  = max(2, int(np.ceil(edge_len / EDGE_CHECK_RES)) + 1)
        ts       = np.linspace(0.0, 1.0, n_check)
        edge_pts = nodes[ni] + np.outer(ts, new_pt - nodes[ni])
        if np.any(_esdf_batch(edge_pts, obstacles) < safety_margin):
            continue

        nodes[n]   = new_pt
        parents[n] = ni
        new_idx    = n
        n         += 1

        if float(np.linalg.norm(new_pt - goal_a)) < GOAL_TOL:
            path = []
            i = new_idx
            while i >= 0:
                path.append(nodes[i].copy())
                i = int(parents[i])
            path.reverse()
            path.append(goal_a.copy())
            min_c = float(np.min(_esdf_batch(np.array(path), obstacles)))
            return path, min_c

    return None, None


def _path_length(path: list) -> float:
    return float(sum(
        np.linalg.norm(path[i + 1] - path[i]) for i in range(len(path) - 1)))


# ── Waypoint sequencer safety check ───────────────────────────────────────────

def _corner_safe(path: list, wp_reach_thresh: float,
                 obstacles: list, safety_margin: float) -> bool:
    """
    Simulate the waypoint sequencer on a path.

    The sequencer advances from waypoint i to i+1 as soon as the drone is
    within wp_reach_thresh of i.  In the worst case the drone is still
    approaching i from the direction of i-1 — so the shortcut it takes is
    approximately the chord from i-1 to i+1, skipping i entirely.

    For each waypoint i that the drone would skip (because
    dist(i-1, i) ≤ wp_reach_thresh), we check whether the chord
    path[i-1] → path[i+1] is collision-free.  If it is not, the
    sequencer would command the drone through an obstacle.

    Returns True iff every potential shortcut is safe.
    """
    for i in range(1, len(path) - 1):
        spacing = float(np.linalg.norm(path[i] - path[i - 1]))
        if wp_reach_thresh >= spacing:
            # The drone reaches i before it has fully arrived — check the chord
            chord = path[i - 1] + np.outer(
                np.linspace(0.0, 1.0, max(2, int(
                    np.linalg.norm(path[i + 1] - path[i - 1]) / EDGE_CHECK_RES) + 1)),
                path[i + 1] - path[i - 1])
            if np.any(_esdf_batch(chord, obstacles) < safety_margin):
                return False
    return True


# ── Replan interval cost ───────────────────────────────────────────────────────

def _replan_cost(replan_goal_dist: float, target_speed: float) -> float:
    """
    Penalise replan intervals outside [REPLAN_MIN_S, REPLAN_MAX_S].

    replan_interval ≈ replan_goal_dist / target_speed

    - Too short → near-continuous replanning: path instability, CPU waste.
    - Too long  → stale paths: drone chases an old goal while target has moved.
    """
    interval = replan_goal_dist / max(target_speed, 0.1)
    lo_pen = max(0.0, REPLAN_MIN_S - interval) ** 2
    hi_pen = max(0.0, interval - REPLAN_MAX_S) ** 2
    return lo_pen + hi_pen


# ── Cost function ─────────────────────────────────────────────────────────────

_obstacles:        list  = []
_queries:          list  = []
_bounds:           tuple = (0.0, 1.0, 0.0, 1.0)
_target_speed:     float = 1.0
_wp_reach_thresh:  float = 0.8   # fixed — not optimised


def _cost(params: np.ndarray) -> float:
    step_size, goal_bias, safety_margin, replan_goal_dist, max_iter_f = (
        float(params[0]), float(params[1]), float(params[2]),
        float(params[3]), float(params[4]))

    max_iter = max(MIN_ITER, min(MAX_ITER_CAP, int(round(max_iter_f))))

    # Hard bounds
    if step_size        <  0.05:                    return 1e6
    if goal_bias        <= 0.0 or goal_bias >= 1.0: return 1e6
    if safety_margin    <  MIN_MARGIN:              return 1e6
    if replan_goal_dist <= 0.0:                     return 1e6

    wp_reach_thresh = _wp_reach_thresh  # fixed — not a free variable

    x_min, x_max, y_min, y_max = _bounds
    n_total = 0
    n_fail  = 0
    efficiencies:    list[float] = []
    corner_unsafe:   int         = 0
    corner_total:    int         = 0

    for qi, (start, goal) in enumerate(_queries):
        straight = float(np.linalg.norm(goal - start))
        for s in range(N_SEEDS):
            rng  = np.random.default_rng(qi * N_SEEDS + s)
            path, _ = _rrt_plan(
                start, goal, _obstacles,
                step_size, goal_bias, safety_margin,
                x_min, x_max, y_min, y_max, rng,
                max_iter=max_iter)

            n_total += 1
            if path is None:
                n_fail += 1
            else:
                if straight > 1e-6:
                    efficiencies.append(_path_length(path) / straight)
                corner_total += 1
                if not _corner_safe(path, wp_reach_thresh, _obstacles, safety_margin):
                    corner_unsafe += 1

    failure_rate       = n_fail / n_total
    mean_eff           = float(np.mean(efficiencies)) if efficiencies else 3.0
    corner_unsafe_rate = corner_unsafe / corner_total if corner_total > 0 else 1.0
    replan_c           = _replan_cost(replan_goal_dist, _target_speed)
    iter_cost          = max_iter / MAX_ITER_REF

    return (W_FAIL   * failure_rate
          + W_EFF    * mean_eff
          + W_CORNER * corner_unsafe_rate
          + W_REPLAN * replan_c
          + W_ITER   * iter_cost)


# ── Query generation ──────────────────────────────────────────────────────────

def _generate_queries(
    obstacles, x_min, x_max, y_min, y_max, seed: int = 0
) -> list:
    rng = np.random.default_rng(seed)
    queries: list = []
    attempts = 0
    while len(queries) < N_QUERIES and attempts < 20_000:
        attempts += 1
        sx, sy = rng.uniform(x_min, x_max), rng.uniform(y_min, y_max)
        gx, gy = rng.uniform(x_min, x_max), rng.uniform(y_min, y_max)
        if _esdf(sx, sy, obstacles) < FREE_MARGIN: continue
        if _esdf(gx, gy, obstacles) < FREE_MARGIN: continue
        if np.sqrt((gx - sx) ** 2 + (gy - sy) ** 2) < MIN_QUERY_DIST: continue
        queries.append((np.array([sx, sy]), np.array([gx, gy])))
    if len(queries) < N_QUERIES:
        print(f"  Warning: generated only {len(queries)}/{N_QUERIES} queries")
    return queries


# ── Plot ──────────────────────────────────────────────────────────────────────

def _plot(scenario_name: str, x0: np.ndarray, x_opt: np.ndarray) -> None:
    x_min, x_max, y_min, y_max = _bounds
    fig, axes = plt.subplots(1, 2, figsize=(15, 7))

    for col, (params, label) in enumerate([
        (x0,    "Initial params"),
        (x_opt, "Optimised params"),
    ]):
        step, bias, margin, replan, max_iter_f = [float(p) for p in params]
        max_iter = max(MIN_ITER, min(MAX_ITER_CAP, int(round(max_iter_f))))
        thresh   = _wp_reach_thresh
        ax = axes[col]
        ax.set_xlim(x_min, x_max); ax.set_ylim(y_min, y_max)
        ax.set_aspect("equal"); ax.grid(True, alpha=0.4)
        ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")

        for o in _obstacles:
            sz = o["size"]
            ax.add_patch(mpatches.Rectangle(
                (o["x"] - sz, o["y"] - sz), 2 * sz, 2 * sz,
                linewidth=1.5, edgecolor="#444", facecolor="#CCCCCC", alpha=0.85))

        n_show    = min(10, len(_queries))
        successes = 0
        unsafe    = 0
        colors    = plt.cm.tab10(np.linspace(0, 1, n_show))
        for i, (start, goal) in enumerate(_queries[:n_show]):
            rng  = np.random.default_rng(i)
            path, _ = _rrt_plan(
                start, goal, _obstacles,
                step, bias, margin,
                x_min, x_max, y_min, y_max, rng,
                max_iter=max_iter)
            c = colors[i]
            ax.plot(*start, "o", color=c, markersize=6, zorder=5)
            ax.plot(*goal,  "s", color=c, markersize=6, zorder=5)
            if path is not None:
                xs = [p[0] for p in path]; ys = [p[1] for p in path]
                safe = _corner_safe(path, thresh, _obstacles, margin)
                lw   = 1.5 if safe else 2.5
                ls   = "-" if safe else "--"
                ax.plot(xs, ys, ls, color=c, linewidth=lw, alpha=0.8, zorder=4)
                successes += 1
                if not safe:
                    unsafe += 1
            else:
                ax.plot([start[0], goal[0]], [start[1], goal[1]],
                        "--", color=c, linewidth=1, alpha=0.4, zorder=3)
                ax.plot(*goal, "x", color="red", markersize=10, zorder=6)

        interval = replan / max(_target_speed, 0.1)
        ax.set_title(
            f"{label}\n"
            f"step={step:.3f}  bias={bias:.3f}  margin={margin:.3f}\n"
            f"wp_thresh={thresh:.3f}(fixed)  replan={replan:.2f}m "
            f"(≈{interval:.1f}s @ {_target_speed:.1f}m/s)  max_iter={max_iter}\n"
            f"{successes}/{n_show} paths found  |  {unsafe} unsafe corners"
        )

    fig.suptitle(f"RRT Tuner — {scenario_name}", fontsize=12)
    plt.tight_layout()
    plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="RRT parameter tuner for Vista-Tracker")
    parser.add_argument("scenario", help="Scenario YAML path or bare name")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    cfg_path = resolve_config(args.scenario)
    cfg = load_config(cfg_path)

    global _obstacles, _queries, _bounds, _target_speed

    _obstacles    = cfg.get("world", {}).get("obstacles", [])
    grid          = cfg.get("world", {}).get("grid", {})
    x_min         = grid.get("x_min", -12.5)
    x_max         = grid.get("x_max",  12.5)
    y_min         = grid.get("y_min", -12.5)
    y_max         = grid.get("y_max",  12.5)
    _bounds       = (x_min, x_max, y_min, y_max)
    _target_speed = cfg.get("target_trajectory", {}).get("max_speed", 1.0)

    global _wp_reach_thresh
    pcfg = cfg.get("planner", {})
    rcfg = pcfg.get("rrt", {})
    _wp_reach_thresh = pcfg.get("wp_reach_thresh", 0.8)
    x0 = np.array([
        rcfg.get("step_size",              0.8),
        rcfg.get("goal_bias",              0.10),
        rcfg.get("safety_margin",          0.30),
        pcfg.get("replan_goal_dist",       4.0),
        float(rcfg.get("max_iter",         MAX_ITER_REF)),
    ])

    print(f"[tune_rrt] scenario       : {cfg_path}")
    print(f"[tune_rrt] obstacles      : {len(_obstacles)}")
    print(f"[tune_rrt] grid           : x=[{x_min}, {x_max}]  y=[{y_min}, {y_max}]")
    print(f"[tune_rrt] target speed   : {_target_speed} m/s")
    print(f"[tune_rrt] evaluations    : {N_QUERIES} queries × {N_SEEDS} seeds = "
          f"{N_QUERIES * N_SEEDS} runs/eval")
    print(f"[tune_rrt] wp_reach_thresh: {_wp_reach_thresh:.3f}  m  (fixed)")
    print(f"[tune_rrt] initial params : step={x0[0]:.3f}  bias={x0[1]:.3f}  "
          f"margin={x0[2]:.3f}  replan={x0[3]:.2f}  max_iter={int(x0[4])}")

    print("\nGenerating query pairs …")
    _queries = _generate_queries(_obstacles, x_min, x_max, y_min, y_max)
    print(f"  {len(_queries)} pairs generated  (min dist ≥ {MIN_QUERY_DIST} m)")

    c0 = _cost(x0)
    print(f"\nInitial cost : {c0:.4f}")
    print(f"  breakdown  : failure×{W_FAIL}  efficiency×{W_EFF}  "
          f"corner×{W_CORNER}  replan×{W_REPLAN}  iter×{W_ITER}")
    print("Optimising (Nelder-Mead) …\n")

    result = minimize(
        _cost, x0,
        method="Nelder-Mead",
        options={"maxiter": 500, "xatol": 1e-3, "fatol": 1e-3, "disp": True},
    )

    step_size, goal_bias, safety_margin, replan_goal_dist, max_iter_f = (
        max(0.05,       float(result.x[0])),
        float(np.clip(  result.x[1], 0.01, 0.99)),
        max(MIN_MARGIN, float(result.x[2])),
        max(0.1,        float(result.x[3])),
        float(result.x[4]),
    )
    max_iter = max(MIN_ITER, min(MAX_ITER_CAP, int(round(max_iter_f))))
    interval = replan_goal_dist / max(_target_speed, 0.1)

    print(f"\n── Optimised parameters ─────────────────────────────────────────")
    print(f"  step_size        : {step_size:.3f}  m")
    print(f"  goal_bias        : {goal_bias:.3f}")
    print(f"  safety_margin    : {safety_margin:.3f}  m")
    print(f"  wp_reach_thresh  : {_wp_reach_thresh:.3f}  m  (fixed)")
    print(f"  replan_goal_dist : {replan_goal_dist:.3f}  m  "
          f"(≈ {interval:.1f} s at {_target_speed} m/s)")
    print(f"  max_iter         : {max_iter}")
    print(f"  cost             : {result.fun:.4f}  (initial: {c0:.4f})")

    print(f"\n── Paste into your scenario / base.yaml ──────────────────────────")
    print(f"planner:")
    print(f"  replan_goal_dist: {replan_goal_dist:.3f}")
    print(f"  rrt:")
    print(f"    step_size:     {step_size:.3f}")
    print(f"    goal_bias:     {goal_bias:.3f}")
    print(f"    safety_margin: {safety_margin:.3f}")
    print(f"    max_iter:      {max_iter}")

    if not args.no_plot:
        _plot(os.path.basename(cfg_path), x0,
              np.array([step_size, goal_bias, safety_margin,
                        replan_goal_dist, float(max_iter)]))


if __name__ == "__main__":
    main()
