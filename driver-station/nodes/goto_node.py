#!/usr/bin/env python3
"""
Go-To-Position node: drives the robot to a target pose using the same,
validated PD math as the previous TeleopNode implementation. It subscribes
to UI events for start/cancel, to the target pose, and cancels on any
teleop activity. Outputs robot-frame Twist2D on a dedicated autonomy topic
for the MuxNode to arbitrate.
"""

import time
import math
import logging
from typing import Optional, Tuple

import numpy as np

from tide.core.node import BaseNode
from tide.namespaces import robot_topic
from tide.models import Twist2D, Vector2, Pose2D
from tide.models.serialization import to_zenoh_value


class GoToPositionNode(BaseNode):
    def __init__(self, *, config=None):
        super().__init__(config=config)

        # Defaults (match the validated parameters and math)
        self.robot_id = "cash"
        self.update_rate = 30.0
        self.kp_lin = 3.0
        self.kd_lin = 0.0
        self.kp_ang = 0.8
        self.kd_ang = 0.2
        self.stop_pos_tol_m = 0.03
        self.stop_ang_tol_rad = math.radians(1.0)
        self.near_zero_lin = 0.05
        self.near_zero_ang = 0.05

        if config:
            self.robot_id = config.get("robot_id", self.robot_id)
            self.update_rate = float(config.get("update_rate", self.update_rate))
            self.kp_lin = float(config.get("kp_lin", self.kp_lin))
            self.kd_lin = float(config.get("kd_lin", self.kd_lin))
            self.kp_ang = float(config.get("kp_ang", self.kp_ang))
            self.kd_ang = float(config.get("kd_ang", self.kd_ang))
            self.stop_pos_tol_m = float(config.get("stop_pos_tol_m", self.stop_pos_tol_m))
            self.stop_ang_tol_rad = float(config.get("stop_ang_tol_rad", self.stop_ang_tol_rad))
            self.near_zero_lin = float(config.get("near_zero_lin", self.near_zero_lin))
            self.near_zero_ang = float(config.get("near_zero_ang", self.near_zero_ang))

        # Rate
        self.hz = self.update_rate

        # Topics
        self.odom_topic = robot_topic(self.robot_id, "state/pose2d")
        self.target_topic = robot_topic(self.robot_id, "ui/target_pose2d")
        self.goto_start_topic = robot_topic(self.robot_id, "ui/goto/start")
        self.goto_cancel_topic = robot_topic(self.robot_id, "ui/goto/cancel")
        self.teleop_twist_topic = robot_topic(self.robot_id, "cmd/teleop")
        self.autonomy_twist_topic = robot_topic(self.robot_id, "cmd/autonomy")

        # State
        self.logger = logging.getLogger(f"GoTo_{self.robot_id}")
        self.seq = 0
        self._last_time: Optional[float] = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self._last_pose: Optional[Tuple[float, float, float]] = None
        self._last_pose_ts: Optional[float] = None
        self._prev_pose: Optional[Tuple[float, float, float]] = None
        self._prev_pose_ts: Optional[float] = None

        self.target_pose = np.array([0.0, 0.0, 0.0])
        self.has_target = False

        self.goto_enabled = False
        self.goto_started = False
        self.goto_started_time = 0.0
        self.goto_still_time = 0.0
        self.has_moved = False

        # Subscriptions
        self.subscribe(self.odom_topic, self._on_odometry)
        self.subscribe(self.target_topic, self._on_target_pose)
        self.subscribe(self.goto_start_topic, self._on_goto_start)
        self.subscribe(self.goto_cancel_topic, self._on_goto_cancel)
        self.subscribe(self.teleop_twist_topic, self._on_teleop_twist)

        self.logger.info(f"GoToPositionNode started for robot {self.robot_id}")

    # Subscriptions
    def _on_odometry(self, sample):
        try:
            if isinstance(sample, dict) and all(k in sample for k in ("x", "y", "theta")):
                x = float(sample["x"])  # type: ignore[index]
                y = float(sample["y"])  # type: ignore[index]
                theta = float(sample["theta"])  # type: ignore[index]
            elif isinstance(sample, dict) and "pose" in sample:
                pose = sample["pose"]  # type: ignore[index]
                x = float(pose["x"])  # type: ignore[index]
                y = float(pose["y"])  # type: ignore[index]
                theta = float(pose["theta"])  # type: ignore[index]
            else:
                return
            self.current_x = x
            self.current_y = y
            self.current_theta = math.atan2(math.sin(theta), math.cos(theta))
            if self._last_pose is not None:
                self._prev_pose = self._last_pose
                self._prev_pose_ts = self._last_pose_ts
            self._last_pose = (x, y, self.current_theta)
            ts = None
            try:
                ts = float(sample.get("timestamp")) if isinstance(sample, dict) else None
            except Exception:
                ts = None
            self._last_pose_ts = ts or time.time()
        except Exception as e:
            self.logger.debug(f"Failed to parse odometry sample: {e}")

    def _on_target_pose(self, sample):
        try:
            if isinstance(sample, dict) and all(k in sample for k in ("x", "y", "theta")):
                self.target_pose = np.array([float(sample["x"]), float(sample["y"]), float(sample["theta"])])  # type: ignore[index]
                self.has_target = True
            elif isinstance(sample, Pose2D):
                self.target_pose = np.array([float(sample.x), float(sample.y), float(sample.theta)])
                self.has_target = True
        except Exception as e:
            self.logger.debug(f"Failed to parse target pose: {e}")

    def _on_goto_start(self, sample):
        # Any message triggers start
        self.goto_enabled = True
        self.goto_started = False
        self.goto_started_time = 0.0
        self.goto_still_time = 0.0
        self.has_moved = False

    def _on_goto_cancel(self, sample):
        self.goto_enabled = False

    def _on_teleop_twist(self, sample):
        try:
            # Expect Twist2D-like dict
            if isinstance(sample, dict) and "linear" in sample and "angular" in sample:
                lx = float(sample["linear"]["x"])  # type: ignore[index]
                ly = float(sample["linear"]["y"])  # type: ignore[index]
                az = float(sample["angular"])  # type: ignore[index]
            elif isinstance(sample, Twist2D):
                lx = float(sample.linear.x)
                ly = float(sample.linear.y)
                az = float(sample.angular)
            else:
                return
            if self.goto_enabled and (abs(lx) > 0.0 or abs(ly) > 0.0 or abs(az) > 0.0):
                self.goto_enabled = False
        except Exception:
            pass

    # Helpers
    def _compute_odometry_derivatives(self, dt: float):
        if self._last_pose is None or self._prev_pose is None or self._last_pose_ts is None or self._prev_pose_ts is None:
            return 0.0, 0.0, 0.0
        x0, y0, th0 = self._prev_pose
        x1, y1, th1 = self._last_pose
        dt_pose = max(self._last_pose_ts - self._prev_pose_ts, 1e-6)
        dtheta = math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))
        return (x1 - x0) / dt_pose, (y1 - y0) / dt_pose, dtheta / dt_pose

    def _goto_twist(self, dt: float):
        # Same math as validated implementation
        err_x = float(self.target_pose[0] - self.current_x)
        err_y = float(self.target_pose[1] - self.current_y)
        err_th = math.atan2(math.sin(self.target_pose[2] - self.current_theta), math.cos(self.target_pose[2] - self.current_theta))

        vfx = self.kp_lin * err_x * clipped_cos(err_th)
        vfy = -self.kp_lin * err_y * clipped_cos(err_th)
        wcmd = -self.kp_ang * err_th

        pos_err = math.hypot(err_x, err_y)
        at_pos = pos_err <= self.stop_pos_tol_m and abs(err_th) <= self.stop_ang_tol_rad

        vx_m, vy_m, w_m = self._compute_odometry_derivatives(dt)
        speed_mag = math.hypot(vx_m, vy_m)
        near_zero_now = speed_mag <= self.near_zero_lin and abs(w_m) <= self.near_zero_ang

        now = time.time()
        if not self.goto_started:
            self.goto_started = True
            self.goto_started_time = now
            self.goto_still_time = 0.0
            self.has_moved = False
        else:
            if speed_mag > (self.near_zero_lin * 1.5) or abs(w_m) > (self.near_zero_ang * 1.5):
                self.has_moved = True
            if self.has_moved and near_zero_now:
                self.goto_still_time += dt
            else:
                self.goto_still_time = 0.0

        done = at_pos or (self.has_moved and self.goto_still_time >= 1.0)

        return vfx, vfy, wcmd, done

    def _create_twist_message(self, linear_x, linear_y, angular_z):
        self.seq += 1
        return Twist2D(linear=Vector2(x=linear_x, y=linear_y), angular=angular_z)

    # Main loop
    def step(self):
        try:
            now = time.time()
            if self._last_time is None:
                dt = 1.0 / max(self.update_rate, 1e-6)
            else:
                dt = max(now - self._last_time, 1e-6)
            self._last_time = now

            if self.goto_enabled and self.has_target:
                vbx, vby, wz, done = self._goto_twist(dt)
                twist_msg = self._create_twist_message(vbx, vby, wz)
                self.put(self.autonomy_twist_topic, to_zenoh_value(twist_msg))
                if done:
                    self.goto_enabled = False
            else:
                # Not publishing when disabled; Mux will select teleop
                pass

            if self.seq % 90 == 0:
                self.logger.info(f"GoTo {'EN' if self.goto_enabled else 'DIS'} - target={'set' if self.has_target else 'unset'}")

        except Exception as e:
            self.logger.error(f"Error in GoToPositionNode step: {e}")

    def stop(self):
        try:
            # publish zero to autonomy on stop
            zero = self._create_twist_message(0.0, 0.0, 0.0)
            self.put(self.autonomy_twist_topic, to_zenoh_value(zero))
        except Exception:
            pass
        self.logger.info("GoToPositionNode stopped")
        super().stop()


def clipped_cos(x):
    x = max(min(x, np.pi/2), -np.pi/2)
    return math.cos(x)

