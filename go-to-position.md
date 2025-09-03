Goal: drive-to-position from the Driver Station using a simple PD controller per axis, using existing odometry and field-relative driving. The target pose is set from the gamepad, visualized in Rerun, and a button press initiates motion. Movement cancels on stick input and completes on precise tolerances or “settled” velocity.

What’s implemented
- Controller: P-only per axis in field frame, converted to robot-frame twist once.
  - Linear: `v = kp_lin * pos_err`
  - Angular: `w = kp_ang * th_err`
  - Saturation to `max_linear_speed` and `max_angular_speed`.
- Completion: stop when either condition is satisfied:
  - Position within 3 cm and heading within 1 degree, or
  - After movement has started, measured velocity is near-zero for 1s (<= 0.05 m/s and <= 0.05 rad/s).
- Cancellation: any joystick movement or the Cross button cancels and returns to normal teleop.
- Target visualization: target pose is published to `/{robot_id}/ui/target_pose2d` and shown in Rerun as a red point with a heading tick.

Driver controls (PS5)
- Triangle: Set target to current pose (seed target).
- D-Pad: Nudge target X/Y in the field frame while held (0.3 m/s rate).
- L1/R1: Rotate target heading while held (45 deg/s).
- Square: Start go-to-position to the current target.
- Cross: Cancel go-to-position.
- Any stick movement: Cancel go-to-position and return to teleop.

Files changed
- `driver-station/nodes/teleop_node.py`
  - Adds go-to-position mode with simple P controller.
  - Publishes target pose to `ui/target_pose2d`.
  - Reads PS5 buttons/hat to manage target and start/cancel.
  - Replaces SE2 field-relative mapping with a simple rotation using odometry yaw to avoid double transforms.
- `driver-station/nodes/rerun_viz.py`
  - Subscribes to `ui/target_pose2d` and draws the target point and heading.

Tuning and config
- Defaults (override via TeleopNode config):
  - `kp_lin=1.0`, `kp_ang=2.0` (derivative gains disabled by default)
  - `stop_pos_tol_m=0.03`, `stop_ang_tol_rad=deg2rad(1)`
  - `near_zero_lin=0.05 m/s`, `near_zero_ang=0.05 rad/s`
  - Respects existing `max_linear_speed`, `max_angular_speed`, and `field_relative`.

Notes
- The controller runs in the driver station and sends robot-frame twists at the normal teleop rate. Odometry provides pose and is only used for heading and settle detection, not for derivative control.
- The D-pad and L1/R1 adjustments are continuous while held; use Triangle to re-seed target at any time.
- Rerun renders the target at z=0 in world space with a small heading arrow.
