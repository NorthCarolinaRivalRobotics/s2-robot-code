import math
import os
import sys

import numpy as np

# Ensure project root is on path when pytest alters cwd
ROOT = os.path.dirname(os.path.dirname(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from trajectory_testing.generator import Waypoint, generate


def _nearest_distance(xs: np.ndarray, ys: np.ndarray, x: float, y: float) -> float:
    dx = xs - x
    dy = ys - y
    return float(np.min(np.hypot(dx, dy)))


def test_passes_through_waypoints_basic():
    waypoints = [
        Waypoint(0.0, 0.0),
        Waypoint(1.0, 0.0),
        Waypoint(2.0, 1.0),
    ]
    traj = generate(waypoints, v_max=1.0, a_max=1.0, dt=0.02)

    xs = np.array([p.pose.x for p in traj])
    ys = np.array([p.pose.y for p in traj])

    # each waypoint should be close to some trajectory sample
    for w in waypoints:
        d = _nearest_distance(xs, ys, w.x, w.y)
        assert d < 0.05


def test_velocity_and_accel_limits():
    waypoints = [Waypoint(0.0, 0.0), Waypoint(3.0, 0.0)]
    v_max = 0.8
    a_max = 0.5
    traj = generate(waypoints, v_max=v_max, a_max=a_max, dt=0.01)

    # compute speeds from linear velocities
    speeds = []
    for p in traj:
        v = p.twist.linear
        vx = getattr(v, "x", 0.0)
        vy = getattr(v, "y", 0.0)
        speeds.append(math.hypot(vx, vy))
    speeds = np.array(speeds)

    assert np.all(speeds <= v_max + 1e-3)

    # discrete acceleration bound
    dt = traj[1].t - traj[0].t if len(traj) > 1 else 1.0
    acc = np.diff(speeds) / max(dt, 1e-6)
    assert np.all(np.abs(acc) <= a_max + 5e-2)  # small numerical slack


def test_end_velocity_zero():
    waypoints = [Waypoint(0.0, 0.0), Waypoint(2.0, 1.0), Waypoint(3.0, 1.5)]
    traj = generate(waypoints, v_max=1.2, a_max=0.8, dt=0.02)
    v_end = traj[-1].v
    # Also compute from twist linear magnitude as a cross-check
    v_lin = math.hypot(getattr(traj[-1].twist.linear, "x", 0.0), getattr(traj[-1].twist.linear, "y", 0.0))
    assert v_end <= 1e-6
    assert v_lin < 1e-2


def test_start_velocity_zero():
    waypoints = [Waypoint(0.0, 0.0), Waypoint(3.0, 0.0)]
    traj = generate(waypoints, v_max=1.0, a_max=0.5, dt=0.02)
    v0 = traj[0].v
    v_lin0 = math.hypot(getattr(traj[0].twist.linear, "x", 0.0), getattr(traj[0].twist.linear, "y", 0.0))
    assert v0 <= 1e-6
    assert v_lin0 < 1e-2


def test_c1_continuity_xy():
    waypoints = [
        Waypoint(0.0, 0.0),
        Waypoint(1.0, 0.5),
        Waypoint(2.0, 0.0),
        Waypoint(3.0, -0.2),
    ]
    traj = generate(waypoints, v_max=1.0, a_max=1.0, dt=0.02)
    xs = np.array([p.pose.x for p in traj])
    ys = np.array([p.pose.y for p in traj])
    # first derivative shouldn't jump drastically between samples
    dxs = np.diff(xs)
    dys = np.diff(ys)
    # check ratio of max jump between consecutive derivatives is bounded
    ddxs = np.diff(dxs)
    ddys = np.diff(dys)
    # these shouldn't explode; allow leniency for endpoints
    assert np.max(np.abs(ddxs)) < 0.2
    assert np.max(np.abs(ddys)) < 0.2


def test_orientation_zone_hard_constant_heading():
    # Straight line path through a circular hard zone that enforces pi/2 heading
    waypoints = [Waypoint(0.0, 0.0), Waypoint(2.0, 0.0)]

    from trajectory_testing.generator import CircleRegion, OrientationZone

    zone = OrientationZone(region=CircleRegion(1.0, 0.0, 0.5), mode="hard", heading=math.pi / 2, gain=1.0)

    traj = generate(waypoints, v_max=1.0, a_max=1.0, dt=0.01, zones=[zone])

    xs = np.array([p.pose.x for p in traj])
    ys = np.array([p.pose.y for p in traj])
    thetas = np.array([getattr(getattr(p.pose, "rot", None), "theta", 0.0) for p in traj])

    # For points inside the zone, heading should match pi/2 within tolerance
    inside = (xs - 1.0) ** 2 + (ys - 0.0) ** 2 <= 0.5 ** 2
    if not np.any(inside):
        raise AssertionError("Trajectory did not pass through hard zone for test")
    err = np.abs(((thetas[inside] - (math.pi / 2) + math.pi) % (2 * math.pi)) - math.pi)
    assert np.max(err) < 0.2
