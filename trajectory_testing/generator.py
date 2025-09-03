from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Tuple

import math

try:
    import numpy as np
except Exception as e:  # pragma: no cover - numpy should be available
    raise RuntimeError("numpy is required for trajectory generation") from e

try:
    # Prefer tide geometry if available
    from tide.core.geometry import SE2, SO2
    from tide.models import Twist2D
except Exception:
    # Fallback minimal shims for development without tide
    @dataclass
    class SO2:  # type: ignore
        theta: float

        @staticmethod
        def from_angle(theta: float) -> "SO2":
            return SO2(theta=theta)

    @dataclass
    class SE2:  # type: ignore
        x: float
        y: float
        rot: SO2

        @staticmethod
        def from_xy_theta(x: float, y: float, theta: float) -> "SE2":
            return SE2(x=x, y=y, rot=SO2.from_angle(theta))

    @dataclass
    class Vector2:
        x: float
        y: float

    @dataclass
    class Twist2D:  # type: ignore
        linear: Vector2
        angular: float


@dataclass
class Waypoint:
    x: float
    y: float
    theta: Optional[float] = None
    # Optional tangent hint for Hermite (in world frame)
    dx: Optional[float] = None
    dy: Optional[float] = None


@dataclass
class CircleRegion:
    cx: float
    cy: float
    r: float

    def contains(self, x: float, y: float) -> bool:
        return (x - self.cx) ** 2 + (y - self.cy) ** 2 <= self.r ** 2


@dataclass
class OrientationZone:
    region: CircleRegion
    mode: str  # "hard" | "soft"
    # heading can be fixed angle or callable f(x,y)->theta
    heading: float | Callable[[float, float], float]
    gain: float = 1.0


@dataclass
class TrajectoryPoint:
    t: float
    pose: SE2
    twist: Twist2D
    s: float
    v: float
    kappa: float


class Trajectory(List[TrajectoryPoint]):
    @property
    def duration(self) -> float:
        return self[-1].t if self else 0.0

    def sample(self, t: float) -> TrajectoryPoint:
        if not self:
            raise ValueError("empty trajectory")
        if t <= self[0].t:
            return self[0]
        if t >= self[-1].t:
            return self[-1]
        # binary search then lerp simple fields
        lo, hi = 0, len(self) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if self[mid].t <= t:
                lo = mid
            else:
                hi = mid
        a, b = self[lo], self[hi]
        u = (t - a.t) / (b.t - a.t)
        theta = _wrap_angle(_slerp_angle(_theta_of(a.pose), _theta_of(b.pose), u))
        x = a.pose.x * (1 - u) + b.pose.x * u
        y = a.pose.y * (1 - u) + b.pose.y * u
        v = a.v * (1 - u) + b.v * u
        s = a.s * (1 - u) + b.s * u
        kappa = a.kappa * (1 - u) + b.kappa * u
        twist = Twist2D(linear=getattr(a.twist, "linear", None) or a.twist.linear, angular=a.twist.angular)
        return TrajectoryPoint(
            t=t,
            pose=_make_pose(x, y, theta),
            twist=twist,
            s=s,
            v=v,
            kappa=kappa,
        )


def _theta_of(pose: SE2) -> float:
    # tide SE2 has .theta or .rot.angle; support our shim
    if hasattr(pose, "theta"):
        return float(getattr(pose, "theta"))  # type: ignore[attr-defined]
    rot = getattr(pose, "rot", None)
    return float(getattr(rot, "theta", 0.0))


def _wrap_angle(theta: float) -> float:
    return (theta + math.pi) % (2 * math.pi) - math.pi


def _shortest_delta(a: float, b: float) -> float:
    d = _wrap_angle(b - a)
    return d


def _slerp_angle(a: float, b: float, u: float) -> float:
    return a + _shortest_delta(a, b) * u


def _estimate_tangents(points: np.ndarray) -> np.ndarray:
    # Centripedal Catmull-Rom style tangents
    n = len(points)
    tangents = np.zeros_like(points)
    if n <= 1:
        return tangents
    d = np.linalg.norm(np.diff(points, axis=0), axis=1)
    # prevent zeros
    d = np.maximum(d, 1e-6)
    for i in range(n):
        if i == 0:
            tangents[i] = (points[1] - points[0])
        elif i == n - 1:
            tangents[i] = (points[-1] - points[-2])
        else:
            t_in = (points[i] - points[i - 1]) / d[i - 1]
            t_out = (points[i + 1] - points[i]) / d[i]
            tangents[i] = 0.5 * (t_in + t_out)
    return tangents


def _cubic_hermite_segment(p0, m0, p1, m1, u):
    # p(u) = h00 p0 + h10 m0 + h01 p1 + h11 m1; u in [0,1]
    u2 = u * u
    u3 = u2 * u
    h00 = 2 * u3 - 3 * u2 + 1
    h10 = u3 - 2 * u2 + u
    h01 = -2 * u3 + 3 * u2
    h11 = u3 - u2
    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1


def _cubic_hermite_tangent(p0, m0, p1, m1, u):
    u2 = u * u
    dh00 = 6 * u2 - 6 * u
    dh10 = 3 * u2 - 4 * u + 1
    dh01 = -dh00
    dh11 = 3 * u2 - 2 * u
    return dh00 * p0 + dh10 * m0 + dh01 * p1 + dh11 * m1


def _build_path_xy(waypoints: List[Waypoint], samples_per_seg: int = 50) -> Tuple[np.ndarray, np.ndarray]:
    pts = np.array([[w.x, w.y] for w in waypoints], dtype=float)
    n = len(pts)
    if n < 2:
        raise ValueError("need at least two waypoints")

    # Tangents
    tangents = _estimate_tangents(pts)
    for i, w in enumerate(waypoints):
        if w.dx is not None and w.dy is not None:
            tangents[i] = np.array([w.dx, w.dy], dtype=float)

    xs: List[float] = []
    ys: List[float] = []
    for i in range(n - 1):
        p0 = pts[i]
        p1 = pts[i + 1]
        m0 = tangents[i]
        m1 = tangents[i + 1]
        # sample [0,1) for all but append final endpoint once at the end
        for k in range(samples_per_seg):
            u = k / float(samples_per_seg)
            p = _cubic_hermite_segment(p0, m0, p1, m1, u)
            xs.append(float(p[0]))
            ys.append(float(p[1]))
    # append the very last waypoint exactly
    xs.append(float(pts[-1, 0]))
    ys.append(float(pts[-1, 1]))
    return np.asarray(xs), np.asarray(ys)


def _arc_length_and_curvature(xs: np.ndarray, ys: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    ds = np.hypot(dx, dy)
    s = np.concatenate([[0.0], np.cumsum((ds[:-1] + ds[1:]) * 0.5)])
    # Curvature kappa = (x'y" - y'x") / (x'^2 + y'^2)^(3/2)
    denom = (dx * dx + dy * dy) ** 1.5 + 1e-9
    kappa = (dx * ddy - dy * ddx) / denom
    return s, kappa


def _time_parameterize(
    s: np.ndarray,
    v_max: float,
    a_max: float,
    *,
    v_start: Optional[float] = None,
    v_end: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    n = len(s)
    v = np.full(n, float(v_max))
    # apply start boundary if provided
    if v_start is not None:
        v[0] = max(0.0, min(float(v_max), float(v_start)))
    # forward pass
    for i in range(1, n):
        ds = s[i] - s[i - 1]
        v[i] = min(v[i], math.sqrt(max(v[i - 1] ** 2 + 2 * a_max * ds, 0.0)))
    # backward pass
    if v_end is not None:
        v[-1] = max(0.0, min(v[-1], float(v_end)))
    for i in range(n - 2, -1, -1):
        ds = s[i + 1] - s[i]
        v[i] = min(v[i], math.sqrt(max(v[i + 1] ** 2 + 2 * a_max * ds, 0.0)))
    # integrate time
    t = np.zeros(n)
    for i in range(1, n):
        ds = s[i] - s[i - 1]
        vmid = (v[i] + v[i - 1]) * 0.5
        if vmid <= 1e-6:
            dt = 0.0
        else:
            dt = ds / vmid
        t[i] = t[i - 1] + dt
    return t, v


def _heading_profile(
    xs: np.ndarray,
    ys: np.ndarray,
    waypoints: List[Waypoint],
    zones: Optional[List[OrientationZone]] = None,
    smooth_lambda: float = 1.0,
) -> np.ndarray:
    # Base strategy: tangent heading, with waypoint theta blending near waypoints
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    tangent_theta = np.arctan2(dy, dx)
    theta0 = tangent_theta.copy()

    # Blend waypoint thetas locally to seed the solver
    n = len(xs)
    if len(waypoints) >= 2:
        for i, w in enumerate(waypoints):
            if w.theta is None:
                continue
            idx = round((i / (len(waypoints) - 1)) * (n - 1))
            window = 3
            for j in range(max(0, idx - window), min(n, idx + window + 1)):
                u = 1.0 - (abs(j - idx) / (window + 1))
                delta = _shortest_delta(theta0[j], w.theta)
                theta0[j] = _wrap_angle(theta0[j] + u * delta)

    if not zones:
        # Mild smoothing for continuity
        for _ in range(2):
            theta0 = np.convolve(theta0, np.ones(5) / 5.0, mode="same")
        return np.vectorize(_wrap_angle)(theta0)

    # Build soft and hard constraints from zones
    w_soft = np.zeros(n)
    mu_soft = np.zeros(n)
    has_soft = np.zeros(n, dtype=bool)
    w_hard = np.zeros(n)
    mu_hard = np.zeros(n)

    for z in zones:
        for i, (x, y) in enumerate(zip(xs, ys)):
            if not z.region.contains(x, y):
                continue
            # determine reference heading at this (x,y)
            if isinstance(z.heading, (int, float)):
                th_ref = float(z.heading)
            else:
                try:
                    th_ref = float(z.heading(float(x), float(y)))
                except Exception:
                    th_ref = float(theta0[i])
            # unwrap ref near current seed
            th_ref_unwrap = float(theta0[i]) + _shortest_delta(float(theta0[i]), th_ref)
            if z.mode == "hard":
                # large weight to enforce almost exactly
                w_hard[i] = max(w_hard[i], 1000.0)
                mu_hard[i] = th_ref_unwrap
            else:  # soft
                # accumulate weighted circular mean: handle by accumulating to vector and compute atan2 at the end
                if not has_soft[i]:
                    w_soft[i] = 0.0
                    mu_soft[i] = 0.0  # store angle placeholder; we will keep vector sums separately
                    has_soft[i] = True
                # To avoid storing vectors per i, accumulate in two arrays
                # We'll reuse mu_soft to store sin component temporarily via a side array
                # Create side arrays on first use
    # vector accumulators for soft refs
    c_acc = np.zeros(n)
    s_acc = np.zeros(n)
    for z in zones:
        if z.mode != "soft":
            continue
        for i, (x, y) in enumerate(zip(xs, ys)):
            if not z.region.contains(x, y):
                continue
            if isinstance(z.heading, (int, float)):
                th_ref = float(z.heading)
            else:
                try:
                    th_ref = float(z.heading(float(x), float(y)))
                except Exception:
                    th_ref = float(theta0[i])
            th_ref_unwrap = float(theta0[i]) + _shortest_delta(float(theta0[i]), th_ref)
            w = float(max(z.gain, 0.0))
            c_acc[i] += w * math.cos(th_ref_unwrap)
            s_acc[i] += w * math.sin(th_ref_unwrap)
            w_soft[i] += w

    # finalize soft refs
    for i in range(n):
        if w_soft[i] > 0.0:
            mu_soft[i] = math.atan2(s_acc[i], c_acc[i])
        else:
            mu_soft[i] = theta0[i]

    # Solve for theta minimizing smoothness + zone penalties
    theta = _solve_theta_quadratic(theta0, w_soft, mu_soft, w_hard, mu_hard, smooth_lambda)
    theta = np.vectorize(_wrap_angle)(theta)
    return theta


def _solve_theta_quadratic(
    theta0: np.ndarray,
    w_soft: np.ndarray,
    mu_soft: np.ndarray,
    w_hard: np.ndarray,
    mu_hard: np.ndarray,
    smooth_lambda: float,
) -> np.ndarray:
    n = len(theta0)
    # Build tridiagonal system A theta = b
    A = np.zeros((n, n), dtype=float)
    b = np.zeros(n, dtype=float)

    # Smoothness term: lambda * sum (theta[i+1]-theta[i])^2
    for i in range(n - 1):
        A[i, i] += smooth_lambda
        A[i + 1, i + 1] += smooth_lambda
        A[i, i + 1] += -smooth_lambda
        A[i + 1, i] += -smooth_lambda

    # Soft penalties
    for i in range(n):
        if w_soft[i] > 0.0:
            # unwrap mu near current theta0 to avoid discontinuity
            mu = float(theta0[i]) + _shortest_delta(float(theta0[i]), float(mu_soft[i]))
            A[i, i] += w_soft[i]
            b[i] += w_soft[i] * mu

    # Hard penalties (large weights)
    for i in range(n):
        if w_hard[i] > 0.0:
            mu = float(theta0[i]) + _shortest_delta(float(theta0[i]), float(mu_hard[i]))
            A[i, i] += w_hard[i]
            b[i] += w_hard[i] * mu

    # Add small regularization to keep near theta0 if unconstrained
    eps = 1e-6
    for i in range(n):
        A[i, i] += eps
        b[i] += eps * theta0[i]

    theta = np.linalg.solve(A, b)
    return theta


def generate(
    waypoints: List[Waypoint],
    v_max: float,
    a_max: float,
    *,
    omega_max: Optional[float] = None,
    alpha_max: Optional[float] = None,
    zones: Optional[List[OrientationZone]] = None,
    dt: float = 0.02,
    v_start: Optional[float] = 0.0,
    v_end: float = 0.0,
) -> Trajectory:
    if len(waypoints) < 2:
        raise ValueError("need at least two waypoints")

    xs, ys = _build_path_xy(waypoints)
    s, kappa = _arc_length_and_curvature(xs, ys)
    t_arr, v_arr = _time_parameterize(s, v_max, a_max, v_start=v_start, v_end=v_end)
    theta_arr = _heading_profile(xs, ys, waypoints, zones=zones)

    # If angular constraints are provided, clamp linear speed to satisfy |omega| <= omega_max approximately
    if omega_max is not None:
        # dtheta/ds approx finite diff over s
        dtheta = np.gradient(theta_arr)
        ds = np.gradient(s)
        dtheta_ds = np.divide(dtheta, np.maximum(ds, 1e-6))
        v_omega = np.maximum(np.abs(dtheta_ds), 1e-9)
        v_limit = omega_max / v_omega
        v_arr = np.minimum(v_arr, v_limit)
        # recompute time after clamping
        t_arr = np.zeros_like(t_arr)
        for i in range(1, len(s)):
            ds_i = s[i] - s[i - 1]
            vmid = 0.5 * (v_arr[i] + v_arr[i - 1])
            t_arr[i] = t_arr[i - 1] + (ds_i / max(vmid, 1e-6))

    # Build dense sampling at fixed dt
    T = float(t_arr[-1])
    n_samples = int(max(1, math.ceil(T / dt))) + 1
    ts = np.linspace(0.0, T, n_samples)

    # interpolate along arrays
    xs_i = np.interp(ts, t_arr, xs)
    ys_i = np.interp(ts, t_arr, ys)
    s_i = np.interp(ts, t_arr, s)
    v_i = np.interp(ts, t_arr, v_arr)
    kappa_i = np.interp(ts, t_arr, kappa)
    # Precompute path tangent direction as a function of original parameter (t_arr)
    # Derivatives w.r.t. arc length s for unit tangent
    # Protect gradient with s spacing to avoid division by zero
    dx_ds_full = np.gradient(xs, s, edge_order=2)
    dy_ds_full = np.gradient(ys, s, edge_order=2)
    denom = np.hypot(dx_ds_full, dy_ds_full) + 1e-9
    tcos_full = dx_ds_full / denom
    tsin_full = dy_ds_full / denom
    # Interpolate unit tangent at time samples
    tcos_i = np.interp(ts, t_arr, tcos_full)
    tsin_i = np.interp(ts, t_arr, tsin_full)
    # angle interpolation requires care: do piecewise using slerp over nearest indices
    theta_i = np.empty_like(ts)
    for idx, t in enumerate(ts):
        # find bracket
        j = np.searchsorted(t_arr, t)
        if j <= 0:
            theta_i[idx] = theta_arr[0]
        elif j >= len(t_arr):
            theta_i[idx] = theta_arr[-1]
        else:
            t0, t1 = t_arr[j - 1], t_arr[j]
            u = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            theta_i[idx] = _slerp_angle(theta_arr[j - 1], theta_arr[j], float(u))

    traj = Trajectory()
    for i, t in enumerate(ts):
        pose = _make_pose(float(xs_i[i]), float(ys_i[i]), float(theta_i[i]))
        # World-frame linear velocity aligned with path tangent, magnitude = time-parameterized speed
        vx = float(v_i[i] * tcos_i[i])
        vy = float(v_i[i] * tsin_i[i])
        # Angular rate from heading finite difference; enforce zero at final sample
        if i == 0:
            omega = (theta_i[i + 1] - theta_i[i]) / max(ts[i + 1] - ts[i], 1e-6)
        elif i == len(ts) - 1:
            omega = 0.0
        else:
            omega = _wrap_angle(theta_i[i + 1] - theta_i[i - 1]) / max(ts[i + 1] - ts[i - 1], 1e-6)

        # Construct Twist2D safely without assuming tide presence
        try:
            from tide.models import Vector2 as TideVec2  # type: ignore
            lin = TideVec2(x=float(vx), y=float(vy))  # type: ignore[attr-defined]
            twist = Twist2D(linear=lin, angular=float(omega))  # type: ignore
        except Exception:
            lin = type("Vector2", (), {"x": float(vx), "y": float(vy)})()
            twist = Twist2D(linear=lin, angular=float(omega))  # type: ignore
        traj.append(
            TrajectoryPoint(
                t=float(t),
                pose=pose,
                twist=twist,
                s=float(s_i[i]),
                v=float(v_i[i]),
                kappa=float(kappa_i[i]),
            )
        )
    return traj


def _make_pose(x: float, y: float, theta: float):
    # Try tide's SE2/SO2 if available; otherwise create a simple object with expected attributes
    try:
        if hasattr(SE2, "from_xy_theta"):
            return SE2.from_xy_theta(x, y, theta)  # type: ignore[attr-defined]
    except Exception:
        pass
    try:
        if hasattr(SO2, "from_angle"):
            return SE2(x=x, y=y, rot=SO2.from_angle(theta))  # type: ignore[call-arg,attr-defined]
    except Exception:
        pass
    # Fallback plain object
    class Rot:
        def __init__(self, theta: float):
            self.theta = theta

    class Pose:
        def __init__(self, x: float, y: float, theta: float):
            self.x = x
            self.y = y
            self.rot = Rot(theta)

    return Pose(x, y, theta)


__all__ = [
    "Waypoint",
    "CircleRegion",
    "OrientationZone",
    "TrajectoryPoint",
    "Trajectory",
    "generate",
]
