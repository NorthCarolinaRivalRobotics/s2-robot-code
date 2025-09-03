from __future__ import annotations

import math
from typing import List

try:
    import matplotlib.pyplot as plt
    from matplotlib import animation
except Exception as e:  # pragma: no cover
    plt = None
    animation = None

import numpy as np

import os, sys

# Allow running as a script (uv run trajectory_testing/visualize_trajectory.py)
ROOT = os.path.dirname(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from trajectory_testing.generator import Waypoint, generate


def main():  # pragma: no cover - visualization only
    waypoints: List[Waypoint] = [
        Waypoint(0.0, 0.0, theta=0.0),
        Waypoint(1.0, 0.5),
        Waypoint(2.0, 1.0),
        Waypoint(3.0, 1.0, theta=math.pi / 2),
    ]
    traj = generate(waypoints, v_max=1.0, a_max=1.0, dt=0.02)

    xs = np.array([p.pose.x for p in traj])
    ys = np.array([p.pose.y for p in traj])
    thetas = np.array([getattr(getattr(p.pose, "rot", None), "theta", 0.0) for p in traj])
    speeds = np.array([p.v for p in traj])

    if plt is None:
        print("matplotlib not available. Install with 'uv add matplotlib' to view plots.")
        return

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(12, 6), sharex=True, sharey=True)
    for ax in (ax0, ax1):
        ax.set_aspect("equal")
        ax.grid(True)

    ax0.set_title("With heading arrows (color = speed)")
    ax1.set_title("No arrows (clear colormap)")

    vmin, vmax = float(np.min(speeds)), float(np.max(speeds))
    sc0 = ax0.scatter(xs, ys, c=speeds, cmap="viridis", s=8, vmin=vmin, vmax=vmax)
    sc1 = ax1.scatter(xs, ys, c=speeds, cmap="viridis", s=8, vmin=vmin, vmax=vmax)
    fig.colorbar(sc1, ax=[ax0, ax1], label="speed")

    # waypoints on both
    wx = [w.x for w in waypoints]
    wy = [w.y for w in waypoints]
    ax0.plot(wx, wy, "ko", label="waypoints")
    ax1.plot(wx, wy, "ko", label="waypoints")
    ax0.legend(loc="best")

    # draw heading arrows only on left subplot
    stride = max(1, len(xs) // 30)
    for i in range(0, len(xs), stride):
        ax0.arrow(xs[i], ys[i], 0.15 * math.cos(thetas[i]), 0.15 * math.sin(thetas[i]),
                  head_width=0.05, head_length=0.05, fc="r", ec="r", alpha=0.6)

    # Optional animation on the right subplot (no arrows)
    if animation is not None:
        robot_dot, = ax1.plot([], [], "ro", markersize=6)
        trail, = ax1.plot([], [], "r-", alpha=0.5)

        def init():
            robot_dot.set_data([], [])
            trail.set_data([], [])
            return robot_dot, trail

        def update(frame):
            robot_dot.set_data([xs[frame]], [ys[frame]])
            trail.set_data(xs[:frame + 1], ys[:frame + 1])
            return robot_dot, trail

        animation.FuncAnimation(fig, update, init_func=init, frames=len(xs), interval=20, blit=True)

    plt.show()


if __name__ == "__main__":  # pragma: no cover
    main()
