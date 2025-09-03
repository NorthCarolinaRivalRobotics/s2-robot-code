Goal: design and implement a holonomic SE(2) trajectory generator with constraints, tests, and visualization. This document specifies scope, API, algorithms, and validation strategy to implement in `trajectory_testing/generator.py` plus example viz in `trajectory_testing/visualize_trajectory.py`.

Requirements
- Holonomic robot: trajectories in SE(2) with independent heading control.
- Waypoints: configurable pose waypoints `(x, y, theta)` that the path must pass through.
- Kinematic limits: global max linear velocity and acceleration bounds; optional angular bounds.
- Orientation zones: spatial regions where heading is constrained (hard or soft) to face a direction or track a field-aligned function.
- Continuity: at least C1 continuous path (position and tangent continuous). Heading is at least C0, preferably C1 where feasible.
- Libraries: use `tide-sdk` geometry primitives (`SE2`, `SO2`). Additional PyPI allowed via `uv add ...` if needed.
- Tooling: run examples with `uv run ...`. Tests with `pytest` using red/green/refactor.
- Visualization: matplotlib static plot and simple animation; optional rerun stream using `rerun-sdk` (bundled with tide dependency tree).

High-Level Approach
- Separate translational path planning from heading planning:
  - Compute a 2D C1+ path through `(x, y)` waypoints using a parametric spline over arc length `s`.
  - Generate a heading profile `theta(s)` composed of: (a) waypoint heading interpolation, (b) zone constraints (hard/soft), and (c) optional look-ahead facing (tangent, target-facing). Then time-parameterize `s(t)` under kinematic limits to produce time-indexed `SE2` poses and `Twist2D`.

Core Data Types (Python)
- `Waypoint`: position `(x, y)`, optional `theta`, optional tangent hint `(dx, dy)` and `weight` for soft adherence to heading.
- `OrientationZone`:
  - `region`: polygon or circle in world coordinates.
  - `mode`: `"hard" | "soft"`.
  - `heading`: either fixed angle `theta_ref`, a function `theta_ref(x, y)`, or a vector field.
  - `gain`: soft-cost gain if `mode == "soft"`.
- `TrajectoryPoint`: `t, pose: SE2, twist: Twist2D, s, kappa`.
- `Trajectory`: list of `TrajectoryPoint` with helpers (`sample(t)`, `duration`, `as_arrays()`).

Public API (to implement in `generator.py`)
- `generate(
    waypoints: list[Waypoint],
    v_max: float,
    a_max: float,
    *,
    omega_max: float | None = None,
    alpha_max: float | None = None,
    zones: list[OrientationZone] | None = None,
    dt: float = 0.02,
    path_method: str = "cubic_hermite",
    heading_strategy: str = "waypoint_tangent",
    soft_zone_default_gain: float = 1.0,
) -> Trajectory`
- `arc_length(path_xy, samples=...) -> (s_array, x_array, y_array)`.
- `time_parameterize(s, kappa, v_max, a_max) -> (t_array, s_array, v_array)` using forward-backward pass.
- `heading_profile(s, x(s), y(s), waypoints, zones, strategy, ...) -> theta(s)`.

Translational Path (x(s), y(s))
- Use parametric cubic Hermite spline between waypoints to ensure C1 continuity (C2 interior optional). Tangents can be:
  - Estimated via Catmull–Rom style centripetal tangents for natural-looking paths, or
  - Provided per waypoint to force entry/exit directions when needed.
- Compute cumulative arc length `s` by numerical integration and store curvature `kappa(s)` for time-scaling and visualization.

Heading Profile theta(s)
- Strategies:
  - `waypoint_tangent`: interpolate heading from waypoint thetas; where missing, align heading with path tangent `atan2(dy/ds, dx/ds)`.
  - `face_target`: face a global target point `(x_t, y_t)` or moving target function.
  - `zone_constrained`: enforce orientation zones:
    - Hard zones: override/interpolate `theta(s)` to meet the constraint while inside the region, with C0 continuity at region boundaries; optionally smooth with short transition ramps to maintain near-C1.
    - Soft zones: formulate `theta` as the minimizer of a 1D smoothing objective:
      `J(theta) = ∫ [w_smooth (dtheta/ds)^2 + sum_i w_i(s) wrap_pi(theta - theta_ref_i)^2] ds`.
      Discretize along `s` to a banded least-squares system and solve for `theta` that balances smoothness and zone alignment.

Time Parameterization s(t)
- Use standard forward–backward pass along `s` with velocity and acceleration limits:
  - Forward pass: `v[i] = min(v_max, sqrt(v[i-1]^2 + 2*a_max*ds))`.
  - Backward pass: `v[i] = min(v[i], sqrt(v[i+1]^2 + 2*a_max*ds))`.
  - Integrate `dt_i = 2*ds/(v[i]+v[i+1])` to get `t` array.
- Optional curvature-based velocity limit if desired: `v_kappa_max = sqrt(a_lat_max / |kappa|)` (nice-to-have; off by default).
- Angular limits (if provided): adjust `v(s)` to respect derived `|omega| = |dtheta/ds| * v <= omega_max` and `alpha` bounds via an iterative clamp.

Outputs
- Dense trajectory sampled at `dt`:
  - `pose(t): SE2` with `SO2` for heading using `tide.core.geometry`.
  - `twist(t): Twist2D` (linear x/y in world or body frame; choose world-frame linear by default; also provide body-frame on request).
  - Metadata: `s(t)`, `v(t)`, curvature `kappa(s)`.

Edge Cases and Guarantees
- Degenerate waypoints (duplicates): coalesce or add tiny epsilon to avoid zero-length segments.
- Minimal headings: normalize with `wrap_pi` and choose shortest-turn interpolation unless zone requires absolute alignment.
- C1 guarantee: path is C1 in (x, y). Heading continuity is enforced by smoothing objective or short ramps at zone edges.
- Feasibility: if hard zones conflict with waypoint headings, raise an exception with diagnostics and a suggestion to switch to soft mode.

Testing Plan (pytest)
- `test_passes_through_waypoints`: nearest-sample to each waypoint is within tolerance in both position and, where specified, heading.
- `test_velocity_acceleration_limits`: `|v| <= v_max + eps` and discrete acceleration within `a_max + eps` throughout.
- `test_orientation_zone_hard`: inside a hard zone, heading tracks reference within small tolerance; smooth transition at edges.
- `test_orientation_zone_soft`: with soft gain, heading deviates but cost decreases as gain increases; verify monotonicity.
- `test_c1_continuity`: finite differences on `(x, y)` produce continuous tangent; no large jumps between segments.
- `test_dt_sampling_duration`: duration is consistent with path length and v/a constraints; sampling count matches `ceil(T/dt)+1`.
- Optional property tests: random waypoint sets remain bounded and produce monotonic `s` and non-NaN outputs.

Visualization (`visualize_trajectory.py`)
- Matplotlib static:
  - Plot `(x, y)` path, waypoints, tangents.
  - Color by speed; draw heading arrows at a stride.
  - Mark orientation zones (polygons/circles) and desired heading field.
- Matplotlib animation:
  - Animate pose over time with trail; optional subplots for `v(t)`, `omega(t)`.
- Rerun (optional):
  - Log world frame, 2D line strip for trajectory, and time-indexed pose as a 3D box at z=0.

Performance and Dependencies
- Default implementation uses NumPy and SciPy sparse (if available) for the soft-heading least squares; however, we can start with pure-NumPy. If SciPy is missing and needed, add via `uv add scipy`.
- Keep allocations linear in the number of samples; typical `N ≈ 200–1000` per trajectory.

Implementation Roadmap (R/G/R friendly)
1) Path spline and arc length utilities; tests: waypoint pass-through, C1 continuity.
2) Time parameterization forward/backward; tests: v/a limits and duration sanity.
3) Basic heading interpolation (waypoint_tangent); tests for heading continuity.
4) Hard orientation zones with boundary ramps; tests verifying zone adherence.
5) Soft orientation zones via least-squares smoothing; tests for cost behavior.
6) Matplotlib visualization static + animation; optional rerun stream.

Notes
- Use `tide.core.geometry.SE2` and `SO2` for poses/headings to stay consistent with the rest of the stack.
- The generator should be deterministic given the same inputs (set random seeds only if randomized smoothing is added—avoid it by default).
