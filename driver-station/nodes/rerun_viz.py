#!/usr/bin/env python3
"""
Rerun Visualization Node

Subscribes to robot odometry and IMU topics and visualizes the robot in 3D
using rerun-sdk. The robot is shown as a 12in x 12in x 8in cube.

If an odometry topic with (x,y,theta) is not available, this node can
optionally integrate `/state/twist` with IMU yaw to derive pose.
"""

from __future__ import annotations

import time
import math
import logging
from typing import Optional, Mapping
from collections import deque

import numpy as np
import rerun as rr

from tide.core.node import BaseNode
from tide.namespaces import robot_topic

from tide import StateTopic
from tide.models.serialization import from_zenoh_value
from tide.models import Twist2D

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rerun_adapters import (
    tide_quat_to_rr_rotation,
    pose2d_to_transform,
    normalize_angle,
    tide_vec2_to_tuple,
    tide_vec3_to_tuple,
)


logger = logging.getLogger(__name__)


class RerunVizNode(BaseNode):
    """Tide node that renders robot pose from odometry + IMU using Rerun."""

    def __init__(self, *, config=None):
        super().__init__(config=config)

        # Defaults
        self.robot_id = "cash"
        self.update_rate = 30.0  # Hz
        # Topics (Tide namespacing); allow override via config
        # Prefer Pose2D from odometry node
        self.odom_topic = robot_topic(self.robot_id, "state/pose2d")
        self.twist_topic = robot_topic(self.robot_id, "state/twist")
        self.imu_quat_topic = robot_topic(self.robot_id, "sensor/imu/quat")
        self.target_topic = robot_topic(self.robot_id, "ui/target_pose2d")
        self.arm_pos_topic = robot_topic(self.robot_id, "state/arm/position")
        self.arm_cmd_topic = robot_topic(self.robot_id, "cmd/arm/target")

        # Whether to integrate twist if odometry is absent
        self.integrate_twist_if_needed = False
        self.trail_length = 100  # number of historical points to render in trajectory
        self.heading_line_length = 0.15  # meters, ~half the robot length

        if config:
            self.robot_id = config.get("robot_id", self.robot_id)
            self.update_rate = config.get("update_rate", self.update_rate)
            self.odom_topic = config.get("odom_topic", robot_topic(self.robot_id, "state/pose2d"))
            self.twist_topic = config.get("twist_topic", robot_topic(self.robot_id, "state/twist"))
            self.imu_quat_topic = config.get("imu_quat_topic", robot_topic(self.robot_id, "sensor/imu/quat"))
            self.integrate_twist_if_needed = config.get("integrate_twist_if_needed", self.integrate_twist_if_needed)
            self.trail_length = int(config.get("trail_length", self.trail_length))
            self.heading_line_length = float(config.get("heading_line_length", self.heading_line_length))

        # Rerun setup: spawn viewer window and set an app id
        rr.init(f"DriverStation-Rerun-{self.robot_id}", spawn=True)

        # Establish update rate
        self.hz = self.update_rate

        # Subscribe to inputs
        self.subscribe(self.imu_quat_topic, self._on_imu_quat)
        # Prefer odometry if present
        self.subscribe(self.odom_topic, self._on_odometry)
        if self.integrate_twist_if_needed:
            self.subscribe(self.twist_topic, self._on_twist)
        # Optional target pose for visualization
        self.subscribe(self.target_topic, self._on_target_pose)
        # Arm joint state (Vector2: x=shoulder revs, y=elbow revs)
        self.subscribe(self.arm_pos_topic, self._on_arm_position)
        # Arm target commands (Vector3 preferred; Vector2 fallback)
        self.subscribe(self.arm_cmd_topic, self._on_arm_target)

        # State for visualization
        self._last_pose = np.array([0.0, 0.0, 0.0])  # x, y, yaw
        self._last_twist = None  # dict with linear_x, linear_y, angular_z
        self._last_time = None
        self._imu_rotation = None  # rr.Quaternion
        self._imu_yaw = None  # float
        self._trail_points = deque(maxlen=self.trail_length)  # world-frame (x,y,0) points
        self._target_pose = None  # np.array([x,y,yaw])
        self._arm_pos: Optional[tuple[float, float]] = None  # (shoulder_rev, elbow_rev)
        self._arm_target: Optional[tuple[float, float]] = None  # (shoulder_target, elbow_target)

        # Robot geometry (in meters): 12in x 12in x 8in
        self.robot_size = (0.3048, 0.3048, 0.2032)
        self.robot_center_z = self.robot_size[2] / 2.0

        # Create a static box geometry under world/<robot>/geom
        robot_path = f"world/{self.robot_id}"
        rr.log(robot_path + "/geom", rr.Boxes3D(centers=[(0.0, 0.0, self.robot_center_z)], sizes=[self.robot_size]))

        logger.info(
            f"RerunVizNode initialized: odom={self.odom_topic}, twist={self.twist_topic}, imu={self.imu_quat_topic}, hz={self.update_rate}"
        )

    # ---- Topic callbacks ----
    def _on_odometry(self, sample):
        """
        Handle odometry samples.
        Accepts either a dict with x,y,theta or a Pose-like mapping.
        """
        try:
            logger.info(f"ODOMETRY IN RERUN Odometry sample: {sample}")
            # Support common shapes: {x,y,theta} or nested {pose:{x,y,theta}}
            if isinstance(sample, Mapping) and all(k in sample for k in ("x", "y", "theta")):
                x, y, yaw = float(sample["x"]), float(sample["y"]), float(sample["theta"])  # type: ignore[index]
            elif isinstance(sample, Mapping) and "pose" in sample:
                pose = sample["pose"]  # type: ignore[index]
                x, y, yaw = float(pose["x"]), float(pose["y"]), float(pose["theta"])  # type: ignore[index]
            else:
                # Unknown schema; ignore
                return

            self._last_pose = np.array([x, y, yaw], dtype=float)
            self._last_time = time.time() if self._last_time is None else self._last_time
        except Exception as e:
            logger.error(f"Failed to parse odometry: {e}")

    def _on_twist(self, sample):
        """
        Handle Twist2D-like dict: {linear:{x,y}, angular: w_z}.
        """
        try:
            if not isinstance(sample, Mapping):
                return
            self._last_twist = {
                "linear_x": float(sample["linear"]["x"]),  # type: ignore[index]
                "linear_y": float(sample["linear"]["y"]),  # type: ignore[index]
                "angular_z": float(sample["angular"])  # type: ignore[index]
            }
        except Exception as e:
            logger.error(f"Failed to parse twist: {e}")

    def _on_imu_quat(self, sample):
        """Handle IMU quaternion samples; keep full rotation + yaw convenience."""
        try:
            rot = tide_quat_to_rr_rotation(sample)
            # Derive yaw from quaternion (wxyz) for 2D alignment
            w, x, y, z = float(sample["w"]), float(sample["x"]), float(sample["y"]), float(sample["z"])  # type: ignore[index]
            # yaw from quaternion
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self._imu_rotation = rot
            self._imu_yaw = normalize_angle(yaw)
        except Exception as e:
            logger.error(f"Failed to parse IMU quaternion: {e}")

    def _on_target_pose(self, sample):
        try:
            if isinstance(sample, Mapping) and all(k in sample for k in ("x", "y", "theta")):
                x, y, yaw = float(sample["x"]), float(sample["y"]), float(sample["theta"])  # type: ignore[index]
                self._target_pose = np.array([x, y, yaw], dtype=float)
        except Exception as e:
            logger.error(f"Failed to parse target pose: {e}")

    def _on_arm_position(self, sample):
        """Handle arm joint position Vector2: x=shoulder revs, y=elbow revs."""
        try:
            # Accept Tide Vector2 or Mapping
            shoulder, elbow = tide_vec2_to_tuple(sample)
            self._arm_pos = (float(shoulder), float(elbow))
        except Exception as e:
            logger.error(f"Failed to parse arm position: {e}")

    def _on_arm_target(self, sample):
        """Handle arm joint target Vector3/Vector2: x=shoulder, y=elbow (radians)."""
        try:
            # Prefer Vector3 (x,y,z) but accept Vector2
            try:
                sx, sy, _ = tide_vec3_to_tuple(sample)
                shoulder, elbow = sx, sy
            except Exception:
                shoulder, elbow = tide_vec2_to_tuple(sample)
            self._arm_target = (float(shoulder), float(elbow))
        except Exception as e:
            logger.error(f"Failed to parse arm target: {e}")

    # ---- Main loop ----
    def step(self):
        # Use odometry pose exclusively for transform & alignment
        x, y, yaw = self._last_pose.tolist()

        # Log transform & geometry for the robot
        robot_path = f"world/{self.robot_id}"
        rr.log(robot_path, pose2d_to_transform(x, y, yaw, z=0.0))

        # Update and draw trajectory in world frame as a 3D linestrip at z=0
        self._trail_points.append((float(x), float(y), 0.0))
        if len(self._trail_points) >= 2:
            rr.log(f"world/trajectory/{self.robot_id}", rr.LineStrips3D([list(self._trail_points)]))

        # Draw a short heading line in robot-local coordinates so it aligns with the box
        # Place slightly above the top surface to avoid z-fighting
        top_z_local = self.robot_size[2] + 0.005
        start_local = (0.0, 0.0, float(top_z_local))
        end_local = (float(self.heading_line_length), 0.0, float(top_z_local))
        rr.log(robot_path + "/heading", rr.LineStrips3D([[start_local, end_local]]))

        # Draw target pose if available
        if self._target_pose is not None:
            tx, ty, tyaw = self._target_pose.tolist()
            # Point marker at z=0
            rr.log("world/target/point", rr.Points3D(positions=[(float(tx), float(ty), 0.0)], radii=[0.02], colors=[[255, 0, 0]]))
            # Heading arrow as short line
            arrow_len = max(self.heading_line_length, 0.15)
            hx = tx + math.cos(tyaw) * arrow_len
            hy = ty + math.sin(tyaw) * arrow_len
            rr.log("world/target/heading", rr.LineStrips3D([[(float(tx), float(ty), 0.01), (float(hx), float(hy), 0.01)]]))

        # Plot arm joint positions as time series scalars (revolutions)
        if self._arm_pos is not None:
            shoulder_rev, elbow_rev = self._arm_pos
            base = f"plots/{self.robot_id}/arm"
            rr.log(base + "/shoulder_rev", rr.Scalars(float(shoulder_rev)))
            rr.log(base + "/elbow_rev", rr.Scalars(float(elbow_rev)))
        # Plot arm joint targets as time series scalars (radians)
        if self._arm_target is not None:
            t_shoulder, t_elbow = self._arm_target
            base = f"plots/{self.robot_id}/arm"
            rr.log(base + "/shoulder_target", rr.Scalars(float(t_shoulder)))
            rr.log(base + "/elbow_target", rr.Scalars(float(t_elbow)))

    def cleanup(self):
        logger.info("RerunVizNode shutting down")
        super().cleanup()
