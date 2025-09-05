#!/usr/bin/env python3
"""
Simple teleop node for PS5 controller input to robot movement.
"""

import pygame
import os
import time
import logging
import math
import numpy as np

from tide.core.node import BaseNode
from tide.namespaces import robot_topic
from tide import CmdTopic
from tide.models import Twist2D, Vector2, Pose2D
from tide.core.geometry import SE2, SO2
from tide.models.serialization import to_zenoh_value


class TeleopNode(BaseNode):
    """
    A simple teleop node for controlling robot movement with PS5 controller.
    
    Controls:
    - Left stick: Linear movement (forward/backward, left/right)
    - Right stick X: Angular rotation (left/right)
    """
    
    def __init__(self, *, config=None):
        super().__init__(config=config)
        
        # Default configuration
        self.robot_id = "cash"
        self.update_rate = 30.0  # Hz
        self.max_linear_speed = 1.0  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.deadzone = 0.05
        self.field_relative = True  # Convert joystick to field-relative driving

        # Target pose UI (for Go-To node)
        self.target_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.has_target = False
        
        # Override from config
        if config:
            self.robot_id = config.get("robot_id", self.robot_id)
            self.update_rate = config.get("update_rate", self.update_rate)
            self.max_linear_speed = config.get("max_linear_speed", self.max_linear_speed)
            self.max_angular_speed = config.get("max_angular_speed", self.max_angular_speed)
            self.deadzone = config.get("deadzone", self.deadzone)
            self.field_relative = config.get("field_relative", self.field_relative)
            # No PD gains here; TeleopNode only handles manual control and UI
        
        # Set update rate
        self.hz = self.update_rate
        
        # Topic names
        # Publish manual teleop twist to dedicated topic for Mux
        self.twist_topic = robot_topic(self.robot_id, "cmd/teleop")
        self.odom_topic = robot_topic(self.robot_id, "state/pose2d")
        self.target_topic = robot_topic(self.robot_id, "ui/target_pose2d")
        # Go-To control events
        self.goto_start_topic = robot_topic(self.robot_id, "ui/goto/start")
        self.goto_cancel_topic = robot_topic(self.robot_id, "ui/goto/cancel")

        # Odometry subscription for field-relative driving
        self.current_theta = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self._last_pose = None  # (x,y,theta)
        self._last_pose_ts = None
        self._prev_pose = None
        self._prev_pose_ts = None
        self.subscribe(self.odom_topic, self._on_odometry)
        
        # Initialize logging
        self.logger = logging.getLogger(f"Teleop_{self.robot_id}")
        
        # Initialize PS5 controller
        self._init_controller()
        
        # State management
        self.seq = 0
        self._last_time = None
        
        self.logger.info(f"Teleop Node started for robot {self.robot_id}")
        self.logger.info("Controls:")
        self.logger.info("  Left stick: Linear movement (X/Y)")
        self.logger.info("  Right stick X: Angular rotation")
        self.logger.info("Go-To-Position controls:")
        self.logger.info("  Triangle: Set target to current pose")
        self.logger.info("  D-Pad: Move target (field X/Y)")
        self.logger.info("  L1/R1: Rotate target heading")
        self.logger.info("  Square: Start go-to-position")
        self.logger.info("  Cross: Cancel go-to-position")

    def _on_odometry(self, sample):
        """Update the latest robot yaw from odometry (radians)."""
        try:
            # Support common shapes: {x,y,theta} or nested {pose:{x,y,theta}}
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
            # Normalize angle to [-pi, pi] for stability
            self.current_theta = math.atan2(math.sin(theta), math.cos(theta))
            self.current_x = x
            self.current_y = y
            # Shift last -> prev for derivative estimates
            if self._last_pose is not None:
                self._prev_pose = self._last_pose
                self._prev_pose_ts = self._last_pose_ts
            self._last_pose = (x, y, self.current_theta)
            # Capture timestamp if present for better velocity estimation
            ts = None
            try:
                ts = float(sample.get("timestamp")) if isinstance(sample, dict) else None
            except Exception:
                ts = None
            self._last_pose_ts = ts or time.time()
        except Exception as e:
            self.logger.debug(f"Failed to parse odometry sample: {e}")
        
    def _init_controller(self):
        """Initialize PS5 controller."""
        try:
            os.environ['SDL_VIDEODRIVER'] = 'dummy'
            pygame.init()
            pygame.joystick.init()
            
            if pygame.joystick.get_count() == 0:
                raise RuntimeError("No gamepad detected")
            
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            
            self.logger.info(f"Controller connected: {self.joystick.get_name()}")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize controller: {e}")
            self.joystick = None
    

    
    def _read_controller_inputs(self):
        """Read PS5 controller inputs."""
        if not self.joystick:
            return {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        
        try:
            pygame.event.pump()
            
            # Read analog sticks
            left_x = -self.joystick.get_axis(1) if self.joystick.get_numaxes() > 0 else 0.0
            left_y = -self.joystick.get_axis(0) if self.joystick.get_numaxes() > 1 else 0.0
            right_x = self.joystick.get_axis(2) if self.joystick.get_numaxes() > 2 else 0.0
            
            # Apply deadzone
            left_x = self._apply_deadzone(left_x)
            left_y = self._apply_deadzone(left_y)
            right_x = self._apply_deadzone(right_x)
            
            return {
                'linear_x': left_x * self.max_linear_speed,
                'linear_y': left_y * self.max_linear_speed,
                'angular_z': -right_x * self.max_angular_speed,
            }
            
        except Exception as e:
            self.logger.error(f"Error reading controller: {e}")
            return {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
    
    def _apply_deadzone(self, value):
        """Apply deadzone to joystick input."""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * min(scaled, 1.0)


    def _read_buttons(self):
        """Read digital buttons and hat for target manipulation and mode control."""
        if not self.joystick:
            return {}
        try:
            pygame.event.pump()
            buttons = {i: self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())}
            hat = self.joystick.get_hat(0) if self.joystick.get_numhats() > 0 else (0, 0)
            return {
                'square': bool(buttons.get(0, 0)),
                'cross': bool(buttons.get(1, 0)),
                'circle': bool(buttons.get(2, 0)),
                'triangle': bool(buttons.get(3, 0)),
                'l1': bool(buttons.get(4, 0)),
                'r1': bool(buttons.get(5, 0)),
                'hat_x': hat[0],
                'hat_y': hat[1],
            }
        except Exception as e:
            self.logger.debug(f"Button read error: {e}")
            return {}

    def _publish_target(self):
        if not self.has_target:
            return
        try:
            msg = Pose2D(timestamp=time.time(), x=float(self.target_pose[0]), y=float(self.target_pose[1]), theta=float(self.target_pose[2]))
            self.put(self.target_topic, to_zenoh_value(msg))
        except Exception as e:
            self.logger.debug(f"Failed to publish target pose: {e}")

    def _update_target_from_inputs(self, dt: float):
        """Adjust target pose using D-pad and L1/R1; Triangle seeds from current pose."""
        btn = self._read_buttons()
        if not btn:
            return
        # Seed target to current pose
        if btn.get('triangle'):
            self.target_pose = np.array([self.current_x, self.current_y, self.current_theta])
            self.has_target = True
        # Move target in field frame using hat (held = continuous)
        move_speed = 0.3  # m/s when holding hat
        dx = float(btn.get('hat_y', 0)) * move_speed * dt
        dy = float(btn.get('hat_x', 0)) * move_speed * dt
        if abs(dx) > 0 or abs(dy) > 0:
            # Note: hat_y is +1 up; we treat +y up in field
            self.target_pose[0] += dx
            self.target_pose[1] += dy
            self.has_target = True
        # Rotate target using L1/R1 hold
        rot_speed = math.radians(45.0)  # deg/s
        if btn.get('l1'):
            self.target_pose[2] -= rot_speed * dt
            self.has_target = True
        if btn.get('r1'):
            self.target_pose[2] += rot_speed * dt
            self.has_target = True
        # Start/Cancel goto: publish UI events for GoToPositionNode
        if btn.get('square') and self.has_target:
            try:
                self.put(self.goto_start_topic, to_zenoh_value(True))
            except Exception as e:
                self.logger.debug(f"Failed to publish goto start: {e}")
        if btn.get('cross'):
            try:
                self.put(self.goto_cancel_topic, to_zenoh_value(True))
            except Exception as e:
                self.logger.debug(f"Failed to publish goto cancel: {e}")
        # Always publish target if available
        self._publish_target()

    # TeleopNode no longer computes go-to twists; that logic moved to GoToPositionNode

    
    def _create_twist_message(self, linear_x, linear_y, angular_z):
        """Create a twist message."""
        self.seq += 1
        
        twist_msg = Twist2D(
            linear=Vector2(x=linear_x, y=linear_y),
            angular=angular_z
        )
        
        return twist_msg
    
    def step(self):
        """Main processing loop."""
        try:
            now = time.time()
            # Read controller inputs
            inputs = self._read_controller_inputs()
            
            # Get twist commands from controller (joystick frame)
            linear_x = inputs['linear_x']
            linear_y = -inputs['linear_y']
            angular_z = -inputs['angular_z']
            # Compute dt
            if self._last_time is None:
                dt = 1.0 / max(self.update_rate, 1e-6)
            else:
                dt = max(now - self._last_time, 1e-6)
            self._last_time = now

            # Update target editing from buttons (and publish)
            self._update_target_from_inputs(dt)

            # Teleop mode only: optionally convert to robot-frame using SE2 exp/log mapping
            if self.field_relative:
                try:
                    twist_vec = np.array([linear_x, linear_y, angular_z], dtype=float)
                    t_cmd = SE2.exp(twist_vec)
                    r_inv = SE2.exp(np.array([0.0, 0.0, -self.current_theta], dtype=float)).inverse()
                    t_body = r_inv * t_cmd
                    # Preserve commanded angular rate explicitly
                    t_body = SE2(translation=t_body.translation, rotation=SO2(theta=angular_z))
                    t_twist = t_body.log()
                    linear_x = t_twist[0]
                    linear_y = t_twist[1]
                    angular_z = t_twist[2]
                except Exception as map_e:
                    self.logger.debug(f"Field-relative mapping failed; using raw inputs: {map_e}")

            # Create and publish twist message (robot-frame)
            twist_msg = self._create_twist_message(linear_x, linear_y, angular_z)
            self.logger.info(f"twist_msg: {twist_msg}")
            self.put(self.twist_topic, to_zenoh_value(twist_msg))
            
            # Log status periodically
            if self.seq % 90 == 0:  # Every 3 seconds at 30Hz
                self.logger.info(f"Teleop active - Linear: ({linear_x:.2f}, {linear_y:.2f}), Angular: {angular_z:.2f}")
            
        except Exception as e:
            self.logger.error(f"Error in step: {e}")
            # Send zero twist on error
            try:
                zero_twist = self._create_twist_message(0.0, 0.0, 0.0)
                self.put(self.twist_topic, to_zenoh_value(zero_twist))
            except:
                pass
    
    def stop(self):
        """Clean up and stop."""
        try:
            # Send stop command
            zero_twist = self._create_twist_message(0.0, 0.0, 0.0)
            self.put(self.twist_topic, to_zenoh_value(zero_twist))
            self.logger.info("Sent stop command")
        except Exception as e:
            self.logger.error(f"Error during stop: {e}")
        
        # Clean up pygame
        try:
            if self.joystick:
                self.joystick.quit()
            pygame.quit()
        except:
            pass
        
        self.logger.info("Teleop Node stopped")
        super().stop() 

def clipped_cos(x):
    x = max(min(x, np.pi/2), -np.pi/2)
    return math.cos(x)
