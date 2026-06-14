import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

df = pd.read_csv("../data/trajectory.csv")

fig, ax = plt.subplots(figsize=(8, 6))

ax.set_title("Drone Tracking Target")

ax.set_aspect("equal")
ax.grid(True)

xmin = min(
    df["target_x"].min(),
    df["drone_x"].min()
) - 2

xmax = max(
    df["target_x"].max(),
    df["drone_x"].max()
) + 2

ymin = min(
    df["target_y"].min(),
    df["drone_y"].min()
) - 2

ymax = max(
    df["target_y"].max(),
    df["drone_y"].max()
) + 2

ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)

target_dot, = ax.plot([], [], "ro", label="Target")
drone_dot, = ax.plot([], [], "bo", label="Drone")

target_path, = ax.plot([], [], "--", alpha=0.5)
drone_path, = ax.plot([], [], "--", alpha=0.5)

distance_text = ax.text(
    0.02,
    0.95,
    "",
    transform=ax.transAxes
)

ax.legend()

def update(frame):

    tx = df["target_x"][frame]
    ty = df["target_y"][frame]

    dx = df["drone_x"][frame]
    dy = df["drone_y"][frame]

    target_dot.set_data([tx], [ty])
    drone_dot.set_data([dx], [dy])

    target_path.set_data(
        df["target_x"][:frame],
        df["target_y"][:frame]
    )

    drone_path.set_data(
        df["drone_x"][:frame],
        df["drone_y"][:frame]
    )

    distance = (
        (tx - dx)**2 +
        (ty - dy)**2
    )**0.5

    distance_text.set_text(
        f"Distance = {distance:.2f} m"
    )

    return (
        target_dot,
        drone_dot,
        target_path,
        drone_path,
        distance_text
    )

ani = FuncAnimation(
    fig,
    update,
    frames=len(df),
    interval=30,
    blit=True
)

plt.show()