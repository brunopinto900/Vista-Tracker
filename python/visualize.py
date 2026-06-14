import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

df = pd.read_csv("../data/trajectory.csv")

t = df["t"]

# =========================
# FIGURE (2x2 GRID)
# =========================
fig, axs = plt.subplots(2, 2, figsize=(12, 8))

ax_motion = axs[0, 0]
ax_vel = axs[0, 1]
ax_error = axs[1, 0]
ax_empty = axs[1, 1]

# =========================
# MOTION PLOT (TOP LEFT)
# =========================
ax_motion.set_title("2D Tracking")
ax_motion.set_xlabel("X [m]")
ax_motion.set_ylabel("Y [m]")
ax_motion.set_aspect("equal")
ax_motion.grid(True)

ax_motion.set_xlim(
    min(df["target_x"].min(), df["drone_x"].min()) - 2,
    max(df["target_x"].max(), df["drone_x"].max()) + 2
)

ax_motion.set_ylim(
    min(df["target_y"].min(), df["drone_y"].min()) - 2,
    max(df["target_y"].max(), df["drone_y"].max()) + 2
)

target_pt, = ax_motion.plot([], [], "ro", label="Target")
drone_pt, = ax_motion.plot([], [], "bo", label="Drone")

target_path, = ax_motion.plot([], [], "r--", alpha=0.5)
drone_path, = ax_motion.plot([], [], "b--", alpha=0.5)

circle, = ax_motion.plot([], [], "g--", alpha=0.5, label="4m constraint")

ax_motion.legend()

# =========================
# VELOCITIES (TOP RIGHT)
# =========================
ax_vel.set_title("Velocities")

vx_d, = ax_vel.plot([], [], "b", label="drone vx")
vy_d, = ax_vel.plot([], [], "g", label="drone vy")
vz_d, = ax_vel.plot([], [], "r", label="drone vz")

vx_t, = ax_vel.plot([], [], "b--", label="target vx")
vy_t, = ax_vel.plot([], [], "g--", label="target vy")
vz_t, = ax_vel.plot([], [], "r--", label="target vz")

ax_vel.set_xlim(0, df["t"].max())
ax_vel.set_ylim(-2, 2)
ax_vel.grid(True)
ax_vel.legend()

# =========================
# ERROR (BOTTOM LEFT)
# =========================
ax_error.set_title("Distance Error")

err_line, = ax_error.plot([], [], "k")

ax_error.set_xlim(0, df["t"].max())
ax_error.set_ylim(0, 10)
ax_error.grid(True)

# =========================
# EMPTY (BOTTOM RIGHT)
# =========================
ax_empty.set_title("Placeholder")
ax_empty.axis("off")

# =========================
# UPDATE FUNCTION
# =========================
def update(i):

    # ---------- motion ----------
    tx = float(df["target_x"].iloc[i])
    ty = float(df["target_y"].iloc[i])
    dx = df["drone_x"][i]
    dy = df["drone_y"][i]

    target_pt.set_data([tx], [ty])
    drone_pt.set_data([dx], [dy])

    target_path.set_data(df["target_x"][:i], df["target_y"][:i])
    drone_path.set_data(df["drone_x"][:i], df["drone_y"][:i])

    # 4m circle
    theta = np.linspace(0, 2*np.pi, 80)
    circle.set_data(
        tx + 4*np.cos(theta),
        ty + 4*np.sin(theta)
    )

    # ---------- velocity ----------
    vx_d.set_data(t[:i], df["vx_cmd"][:i])
    vy_d.set_data(t[:i], df["vy_cmd"][:i])
    vz_d.set_data(t[:i], df["vz_cmd"][:i])

    vx_t.set_data(t[:i], df["target_vx"][:i])
    vy_t.set_data(t[:i], df["target_vy"][:i])
    vz_t.set_data(t[:i], df["target_vz"][:i])

    # ---------- error ----------
    err = np.sqrt(
        (df["target_x"][:i] - df["drone_x"][:i])**2 +
        (df["target_y"][:i] - df["drone_y"][:i])**2 +
        (df["target_z"][:i] - df["drone_z"][:i])**2
    )

    err_line.set_data(t[:i], err)

    return (
        target_pt, drone_pt,
        target_path, drone_path,
        circle,
        vx_d, vy_d, vz_d,
        vx_t, vy_t, vz_t,
        err_line
    )

# =========================
# ANIMATION
# =========================
ani = FuncAnimation(
    fig,
    update,
    frames=len(df),
    interval=30,
    blit=False
)

plt.tight_layout()
plt.show()