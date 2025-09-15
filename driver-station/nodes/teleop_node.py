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
from tide.core.utils import add_project_root_to_path

add_project_root_to_path(__file__)
# Use the acceleration limiter from GoTo node

from .goto_node import apply_acceleration_limit, Twist2dVelocity

from tide.core.node import BaseNode
from tide.namespaces import robot_topic
from tide import CmdTopic
from tide.models import Twist2D, Vector2, Vector3, Pose2D
from tide.core.geometry import SE2, SO2
from tide.models.serialization import to_zenoh_value


CLAW_CLOSED = "CLOSED"
CLAW_OPEN = "OPEN"
CLAW_SPEAR = "SPEAR"
_CLAW_STATES = {CLAW_CLOSED, CLAW_OPEN, CLAW_SPEAR}


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
            # Arm position proximity tolerance (radians) for state transitions
            # If not provided, default to ~3 degrees.
            self.arm_pos_tol_rad = float(config.get("arm_position_tolerance_rad", math.radians(3.0)))
        else:
            self.arm_pos_tol_rad = math.radians(3.0)
        
        # Set update rate
        self.hz = self.update_rate
        
        # Topic names
        # Publish manual teleop twist to dedicated topic for Mux
        self.twist_topic = robot_topic(self.robot_id, "cmd/teleop")
        self.odom_topic = robot_topic(self.robot_id, "state/pose2d")
        self.target_topic = robot_topic(self.robot_id, "ui/target_pose2d")
        # Arm target topic (Vector2: x=shoulder revs, y=elbow revs)
        # Use absolute topic to match the rest of the stack
        self.arm_cmd_topic = robot_topic(self.robot_id, "cmd/arm/target")
        # Arm state topic (Vector2: x=shoulder rad, y=elbow rad)
        self.arm_pos_topic = robot_topic(self.robot_id, "state/arm/position")
        # Wrist/claw command topics (native Tide nodes use full path)
        self.wrist_angle_topic = robot_topic(self.robot_id, "cmd/wrist/angle")
        self.claw_cmd_topic = robot_topic(self.robot_id, "cmd/wrist/claw")
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
        # Arm joint position subscription
        self._arm_pos: tuple[float, float] | None = None  # (shoulder_rad, elbow_rad)
        self._arm_pos_ts: float | None = None
        self.subscribe(self.arm_pos_topic, self._on_arm_position)
        
        # Initialize logging
        self.logger = logging.getLogger(f"Teleop_{self.robot_id}")
        
        # Initialize PS5 controller
        self._init_controller()
        
        # State management
        self.seq = 0
        self._last_time = None
        # Teleop publish gating: only publish when inputs are active
        self._teleop_active = False
        
        # Arm control state (driver station side - manual increments)
        self.arm_target = np.array([0.0, 0.0], dtype=float)  # [shoulder, elbow] in joint revs
        self.arm_step = (config or {}).get('arm_step', 0.02)  # revs per tick
        self.arm_repeat_interval = (config or {}).get('arm_repeat_interval', 0.3)  # seconds
        self._arm_last_hat = (0, 0)
        self._arm_last_emit = 0.0
        # Arm intermediate end-velocity fraction (0..1) used for travel waypoints
        self.arm_intermediate_end_velocity_frac = float((config or {}).get('arm_intermediate_end_velocity_frac', -0.1))

        # Arm state machine
        self._arm_state = 'STOW'  # INIT -> STOW immediately at start
        self._last_transition_ts = 0.0
        self._debounce_s = float((config or {}).get('arm_button_debounce_s', 0.2))
        self._btn_prev: dict[str, bool] = {}

        # Intermediate travel fallback timer (position-based transition preferred)
        # Increase to 3.0s so that a fallback is obvious if it triggers.
        self._wrist_travel_timer = ElapsedTimer(0.35)
        # Intermediate crossing detection state
        self._intermediate_target: np.ndarray | None = None
        self._intermediate_prev_err: tuple[float, float] | None = None
        # Named poses (placeholders; will be tuned later) shoulder, elbow
        # arm straight out is (0.0, pi)
        UP = np.pi/2.0


        self._pose_map = {
            "UP": np.array([UP, UP], dtype=float),
            'STOW':        np.array([UP - np.deg2rad(35.0), np.deg2rad(270.0)], dtype=float),
            'PLACE_HEAD':  np.array([UP + np.deg2rad(12.0), UP + np.deg2rad(47.0)], dtype=float),
            'PLACE_HEAD_WRIST_TRAVEL': np.array([UP + np.deg2rad(0.0), np.deg2rad(200.0)], dtype=float),
            'PLACE_SIDE':  np.array([UP + np.deg2rad(-10.0), UP + np.deg2rad(10.0)], dtype=float),
            'PLACE_SIDE_WRIST_TRAVEL': np.array([UP + np.deg2rad(15.0), np.deg2rad(230.0)], dtype=float),
            'RELEASE':     np.array([UP + np.deg2rad(15.0), np.deg2rad(200.0)], dtype=float),
            'GROUND_IN_PARALLEL':     np.array([UP - np.deg2rad(55.0), np.deg2rad(280.0)], dtype=float),
            'GROUND_IN_PERPENDICULAR':   np.array([UP - np.deg2rad(32.0), np.deg2rad(260.0)], dtype=float),
        }

        self._wrist_angle_map = {
            'UP': np.pi/2.0,
            'STOW': np.pi,
            'PLACE_HEAD': np.pi/2.0,
            'PLACE_HEAD_WRIST_TRAVEL': np.pi,
            'PLACE_SIDE': np.pi/2.0,
            'PLACE_SIDE_WRIST_TRAVEL': np.pi,
            'RELEASE': np.pi,
            'GROUND_IN_PARALLEL': np.deg2rad(90 + 55.0),
            'GROUND_IN_PERPENDICULAR': np.deg2rad(90 - 10.0),
        }

        # Claw state (decoupled from arm state machine)
        self._claw_state = CLAW_CLOSED
        self._rb_prev = False  # right bumper previous state for edge detection
        self._lb_prev = False  # left bumper previous state for edge detection
        # Trigger press latches
        self._l2_active = False
        self._r2_active = False
        
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
        self.logger.info("Arm control:")
        self.logger.info("  D-Pad Left/Right: Shoulder -/+ small step")
        self.logger.info("  D-Pad Up/Down: Elbow +/− small step")
        self.logger.info("  Triangle: Place (head-on)")
        self.logger.info("  Square: Place (side)")
        self.logger.info("  Left bumper: Spear claw (toggle spear/closed)")
        self.logger.info("  Right bumper: Toggle claw open/close")
        self.logger.info("  Circle: Toggle intakes")
        self.logger.info("  Cross: Advance (open->silo) or Stow from intakes")
        self.logger.info("  L2>0.5: GoTo SILO pose,  R2>0.5: GoTo GOAL pose")
        self.logger.info(f"  Arm step: {self.arm_step:.3f} rev, repeat every {self.arm_repeat_interval:.2f}s")
        self.logger.info(f"  Arm cmd topic: {self.arm_cmd_topic}")

        self.is_first_arm_update = True

    def _on_arm_position(self, sample):
        """Handle arm joint position Vector2: x=shoulder rad, y=elbow rad."""
        try:
            print("case 2")
            shoulder = float(sample["x"])  # type: ignore[index]
            elbow = float(sample["y"])    # type: ignore[index]
            self._arm_pos = (shoulder, elbow)
            self._arm_pos_ts = time.time()
        except Exception as e:
            self.logger.debug(f"Failed to parse arm position: {e}")

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
            # Triggers as axes -> normalize from [-1,1] to [0,1]
            l2 = 0.0
            r2 = 0.0
            try:
                axes = self.joystick.get_numaxes()
                # Common PS5 layouts put L2/R2 on axes 4/5 (or 3/4). Try safely.
                if axes > 4:
                    l2_raw = float(self.joystick.get_axis(4))
                    r2_raw = float(self.joystick.get_axis(5))
                elif axes > 3:
                    l2_raw = float(self.joystick.get_axis(3))
                    r2_raw = 0.0
                else:
                    l2_raw = 0.0
                    r2_raw = 0.0
                l2 = max(0.0, min(1.0, (l2_raw + 1.0) * 0.5))
                r2 = max(0.0, min(1.0, (r2_raw + 1.0) * 0.5))
            except Exception:
                l2, r2 = 0.0, 0.0
            return {
                'square': bool(buttons.get(2, 0)),
                'cross': bool(buttons.get(0, 0)),
                'circle': bool(buttons.get(1, 0)),
                'triangle': bool(buttons.get(3, 0)),
                'l1': bool(buttons.get(4, 0)),
                'r1': bool(buttons.get(5, 0)),
                'hat_x': hat[0],
                'hat_y': hat[1],
                'l2': l2,
                'r2': r2,
                'direction_down': bool(buttons.get(12, 0)),
                'direction_up': bool(buttons.get(11, 0)),
                'direction_left': bool(buttons.get(13, 0)),
                'direction_right': bool(buttons.get(14, 0)),
                'left_bumper': bool(buttons.get(9, 0)),
                'right_bumper': bool(buttons.get(10, 0)),
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
        # if btn.get('square') and self.has_target:
        #     try:
        #         self.put(self.goto_start_topic, to_zenoh_value(True))
        #     except Exception as e:
        #         self.logger.debug(f"Failed to publish goto start: {e}")
        # if btn.get('cross'):
        #     try:
        #         self.put(self.goto_cancel_topic, to_zenoh_value(True))
        #     except Exception as e:
        #         self.logger.debug(f"Failed to publish goto cancel: {e}")
        # Always publish target if available
        self._publish_target()

    # TeleopNode no longer computes go-to twists; that logic moved to GoToPositionNode

    def _update_arm_from_hat(self, now: float) -> None:
        """Use D-pad to increment arm joint targets with debouncing and slow repeat."""
        btn = self._read_buttons()
        if not btn:
            return
        hat = (int(btn.get('hat_x', 0)), int(btn.get('hat_y', 0)))
        # Reset debounce when hat released
        if hat == (0, 0):
            self._arm_last_hat = (0, 0)
            # Debug: hat released
            self.logger.debug("Arm D-pad: released (no command)")
            return
        # Emit on edge or at slow repeat interval
        should_emit = False
        if hat != self._arm_last_hat:
            should_emit = True
        elif (now - self._arm_last_emit) >= float(self.arm_repeat_interval):
            should_emit = True
        if not should_emit:
            remaining = max(0.0, float(self.arm_repeat_interval) - (now - self._arm_last_emit))
            self.logger.debug(f"Arm D-pad: debounced hat={hat}, next send in {remaining:.2f}s")
            return
        # Map hat to deltas: left/right -> shoulder, up/down -> elbow
        d_shoulder = float(hat[0]) * self.arm_step  # right=+1, left=-1
        d_elbow = float(hat[1]) * self.arm_step    # up=+1, down=-1
        # Update and publish
        self.arm_target[0] += d_shoulder
        self.arm_target[1] += d_elbow
        try:
            # Publish as Vector3 with zero end-velocity for manual nudges.
            self.put(
                self.arm_cmd_topic,
                to_zenoh_value(Vector3(x=float(self.arm_target[0]), y=float(self.arm_target[1]), z=0.0)),
            )
        except Exception as e:
            self.logger.debug(f"Failed to publish arm target: {e}")
        self._arm_last_hat = hat
        self._arm_last_emit = now

    def _arm_publish_targets(self, shoulder: float, elbow: float, wrist_angle: float, claw_state: str, *, end_velocity_frac: float = 0.0):
        # Arm joints
        try:
            # Publish desired end-velocity fraction in z for intermediate waypoints
            self.put(
                self.arm_cmd_topic,
                to_zenoh_value(Vector3(x=float(shoulder), y=float(elbow), z=float(end_velocity_frac))),
            )
        except Exception as e:
            self.logger.info(f"Failed to publish arm target: {e}")
        # Wrist angle
        try:
            self.put(self.wrist_angle_topic, to_zenoh_value(float(wrist_angle)))
        except Exception as e:
            self.logger.info(f"Failed to publish wrist angle: {e}")
        # Claw command (driven elsewhere via toggle)
        self._publish_claw_state(claw_state)

    def _publish_claw_state(self, state: str, *, log: bool = False) -> None:
        if state not in _CLAW_STATES:
            self.logger.debug(f"Ignoring unknown claw state '{state}'")
            return
        try:
            self.put(self.claw_cmd_topic, to_zenoh_value(state))
            if log:
                self.logger.info(f"Claw set to {state}")
        except Exception as e:
            self.logger.info(f"Failed to publish claw state '{state}': {e}")

    def _set_claw_state(self, state: str, *, log: bool = True) -> None:
        if state not in _CLAW_STATES:
            self.logger.debug(f"Ignoring attempt to set unknown claw state '{state}'")
            return
        if self._claw_state == state:
            return
        self._claw_state = state
        self._publish_claw_state(state, log=log)

    def _apply_arm_state(self):
        pose = self._pose_map.get(self._arm_state, np.array([0.0, 0.0], dtype=float))
        wrist = float(self._wrist_angle_map.get(self._arm_state, 0.0))
        # Use independent claw toggle state rather than state machine map
        claw_state = self._claw_state
        self.arm_target[:] = pose
        # Use non-zero end velocity on travel waypoints to smooth transitions
        # Include RELEASE so it can flow-through to GROUND_IN_PARALLEL automatically
        if self._arm_state in ("PLACE_HEAD_WRIST_TRAVEL", "PLACE_SIDE_WRIST_TRAVEL", "RELEASE"):
            evf = self.arm_intermediate_end_velocity_frac
        else:
            evf = 0.0
        self._arm_publish_targets(float(pose[0]), float(pose[1]), wrist, claw_state, end_velocity_frac=evf)

    def _begin_intermediate_monitor(self, state_name: str):
        """Initialize crossing detection for an intermediate waypoint state."""
        tgt = self._pose_map.get(state_name)
        self._intermediate_target = tgt.copy() if tgt is not None else None
        if self._arm_pos is not None and self._intermediate_target is not None:
            s_err = float(self._intermediate_target[0]) - float(self._arm_pos[0])
            e_err = float(self._intermediate_target[1]) - float(self._arm_pos[1])
            self._intermediate_prev_err = (s_err, e_err)
            self.logger.debug(
                f"Intermediate monitor start: target=({self._intermediate_target[0]:.3f}, {self._intermediate_target[1]:.3f}), "
                f"err=({s_err:.3f}, {e_err:.3f})"
            )
        else:
            self._intermediate_prev_err = None

    def _clear_intermediate_monitor(self):
        self._intermediate_target = None
        self._intermediate_prev_err = None

    def _arm_crossed_intermediate(self) -> bool:
        """Return True if each joint error changed sign relative to previous reading.

        Uses zero-crossing of error (target - measured). Requires at least one
        previous reading; updates the stored previous error each call.
        """
        if self._arm_pos is None or self._intermediate_target is None:
            return False
        s_err = float(self._intermediate_target[0]) - float(self._arm_pos[0])
        e_err = float(self._intermediate_target[1]) - float(self._arm_pos[1])
        crossed = False
        if self._intermediate_prev_err is not None:
            ps, pe = self._intermediate_prev_err
            # Consider zero or sign flip as crossing
            s_cross = (ps > 0 and s_err <= 0) or (ps < 0 and s_err >= 0) or s_err == 0.0
            e_cross = (pe > 0 and e_err <= 0) or (pe < 0 and e_err >= 0) or e_err == 0.0
            crossed = bool(s_cross or e_cross)
            if crossed:
                self.logger.info(
                    f"Arm crossed intermediate: prev_err=({ps:.3f},{pe:.3f}) curr_err=({s_err:.3f},{e_err:.3f})"
                )
        # Update previous for next iteration
        self._intermediate_prev_err = (s_err, e_err)
        return crossed

    def _arm_near_target(self, target: np.ndarray) -> bool:
        """Return True if measured arm joints are within tolerance of target (radians)."""
        if self._arm_pos is None:
            return False
        shoulder_err = abs(float(self._arm_pos[0]) - float(target[0]))
        elbow_err = abs(float(self._arm_pos[1]) - float(target[1]))
        return shoulder_err <= self.arm_pos_tol_rad and elbow_err <= self.arm_pos_tol_rad

    def _update_arm_state_machine(self, now: float):
        btn = self._read_buttons() or {}

        # Debounce window
        if (now - self._last_transition_ts) < self._debounce_s:
            self._btn_prev = {k: bool(btn.get(k, False)) for k in ('square','cross','circle','triangle')}
            # Still handle triggers in parallel even if debouncing buttons
            self._handle_triggers(btn)
            return
        prev = self._btn_prev
        def pressed(name: str) -> bool:
            return bool(btn.get(name, False)) and not bool(prev.get(name, False))

        # State transitions
        s = self._arm_state
        transitioned = False 
        if self.is_first_arm_update:
            self.is_first_arm_update = False
            transitioned = True
            return     
        # INIT -> STOW is implicit at startup (we start in STOW)
        if s == 'STOW':
            if pressed('triangle'):
                self._arm_state = 'PLACE_HEAD_WRIST_TRAVEL'; transitioned = True; self._wrist_travel_timer.reset(); self._begin_intermediate_monitor(self._arm_state)
            elif pressed('square'):
                self._arm_state = 'PLACE_SIDE_WRIST_TRAVEL'; transitioned = True; self._wrist_travel_timer.reset(); self._begin_intermediate_monitor(self._arm_state)
            elif pressed('circle'):
                self._arm_state = 'GROUND_IN_PERPENDICULAR'; transitioned = True
            elif pressed('cross'):
                self._arm_state = 'GROUND_IN_PARALLEL'; transitioned = True
        elif s == 'PLACE_HEAD_WRIST_TRAVEL' or s == 'PLACE_SIDE_WRIST_TRAVEL':
            # Prefer crossing-based transition; use 3s timer as obvious fallback
            if self._arm_crossed_intermediate():
                self.logger.info("Arm crossed intermediate target; advancing to PLACE state")
                self._arm_state = 'PLACE_HEAD' if s == 'PLACE_HEAD_WRIST_TRAVEL' else 'PLACE_SIDE'; transitioned = True; self._clear_intermediate_monitor()
            elif self._wrist_travel_timer.elapsed():
                self.logger.warning("Fallback: wrist travel timer elapsed (3s); advancing state")
                self._arm_state = 'PLACE_HEAD' if s == 'PLACE_HEAD_WRIST_TRAVEL' else 'PLACE_SIDE'; transitioned = True; self._clear_intermediate_monitor()
        elif s in ('PLACE_HEAD', 'PLACE_SIDE'):
            if pressed('circle'):
                self._arm_state = 'RELEASE'; transitioned = True; self._wrist_travel_timer.reset(); self._begin_intermediate_monitor(self._arm_state)
        elif s == 'RELEASE':
            # Prefer crossing-based transition; use timer as fallback; allow manual advance on Cross
            if self._arm_crossed_intermediate():
                self.logger.info("Arm crossed RELEASE; advancing to STOW")
                self._arm_state = 'STOW'; transitioned = True; self._clear_intermediate_monitor()
            elif self._wrist_travel_timer.elapsed():
                self.logger.warning("Fallback: RELEASE timer elapsed; advancing to STOW")
                self._arm_state = 'STOW'; transitioned = True; self._clear_intermediate_monitor()
            elif pressed('cross'):
                self._arm_state = 'STOW'; transitioned = True; self._clear_intermediate_monitor()
        elif s == 'GROUND_IN_PARALLEL':
            if pressed('circle'):
                self._arm_state = 'GROUND_IN_PERPENDICULAR'; transitioned = True
            elif pressed('cross'):
                self._arm_state = 'STOW'; transitioned = True
            elif pressed('triangle'):
                self._arm_state = 'PLACE_HEAD_WRIST_TRAVEL'; transitioned = True; self._wrist_travel_timer.reset(); self._begin_intermediate_monitor(self._arm_state)

        elif s == 'GROUND_IN_PERPENDICULAR':
            if pressed('circle'):
                self._arm_state = 'GROUND_IN_PARALLEL'; transitioned = True
            elif pressed('cross'):
                self._arm_state = 'STOW'; transitioned = True
            elif pressed('triangle'):
                self._arm_state = 'PLACE_SIDE_WRIST_TRAVEL'; transitioned = True; self._wrist_travel_timer.reset(); self._begin_intermediate_monitor(self._arm_state)
        elif s == 'UP':
            if pressed('direction_down'):
                self._arm_state = 'STOW'; transitioned = True

        if pressed('direction_up'):
            self._arm_state = 'UP'; transitioned = True

        if transitioned:
            self._last_transition_ts = now
        self._apply_arm_state()

        # Save for edge detection next time
        self._btn_prev = {k: bool(btn.get(k, False)) for k in ('square','cross','circle','triangle')}
        # Also handle triggers (GoTo) each cycle
        self._handle_triggers(btn)

    def _handle_triggers(self, btn: dict):
        # Thresholding with edge detection
        l2 = float(btn.get('l2', 0.0))
        r2 = float(btn.get('r2', 0.0))
        l2_now = l2 > 0.5
        r2_now = r2 > 0.5
        if l2_now and not self._l2_active:
            # SILO position (placeholder zeros)
            try:
                pose = Pose2D(timestamp=time.time(), x=0.0, y=0.0, theta=0.0)
                self.put(self.target_topic, to_zenoh_value(pose))
                self.put(self.goto_start_topic, to_zenoh_value(True))
            except Exception:
                pass
        if r2_now and not self._r2_active:
            # GOAL position (placeholder zeros)
            try:
                pose = Pose2D(timestamp=time.time(), x=0.0, y=0.0, theta=0.0)
                self.put(self.target_topic, to_zenoh_value(pose))
                self.put(self.goto_start_topic, to_zenoh_value(True))
            except Exception:
                pass
        self._l2_active = l2_now
        self._r2_active = r2_now

    
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
            self.logger.info(f"Arm State: {self._arm_state}")
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

            # Claw toggle (edge-triggered)
            try:
                btn = self._read_buttons() or {}
                rb_now = bool(btn.get('right_bumper', False))
                lb_now = bool(btn.get('left_bumper', False))
                if rb_now and not self._rb_prev:
                    if self._claw_state == CLAW_SPEAR:
                        target = CLAW_CLOSED
                    elif self._claw_state == CLAW_OPEN:
                        target = CLAW_CLOSED
                    else:
                        target = CLAW_OPEN
                    self._set_claw_state(target)
                if lb_now and not self._lb_prev:
                    target = CLAW_SPEAR if self._claw_state != CLAW_SPEAR else CLAW_CLOSED
                    self._set_claw_state(target)
                self._rb_prev = rb_now
                self._lb_prev = lb_now
            except Exception as e:
                self.logger.debug(f"Claw toggle handling error: {e}")

            # Update target editing from buttons (and publish)
            self._update_target_from_inputs(dt)
            # Update arm control from D-pad (debounced small steps)
            self._update_arm_from_hat(now)
            # Update arm state machine + triggers (debounced)
            self._update_arm_state_machine(now)

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

            # Determine if inputs are effectively idle (inside deadband)
            inputs_idle = (linear_x == 0.0 and linear_y == 0.0 and angular_z == 0.0)

            if inputs_idle:
                # If we were active previously, send one zero then stop publishing
                if self._teleop_active:
                    zero = self._create_twist_message(0.0, 0.0, 0.0)
                    try:
                        self.put(self.twist_topic, to_zenoh_value(zero))
                    except Exception:
                        pass
                    self._teleop_active = False
                # Do not continuously publish zeros; allow mux to select autonomy
            else:
                # Inputs active: apply acceleration limiting and publish
                self._teleop_active = True
                limited = apply_acceleration_limit(Twist2dVelocity(linear_x, linear_y, angular_z), dt)
                twist_msg = self._create_twist_message(limited.vx, limited.vy, limited.w)
                self.put(self.twist_topic, to_zenoh_value(twist_msg))
            
            # Log status periodically
            if self.seq % 90 == 0:  # Every 3 seconds at 30Hz
                self.logger.info(
                    f"Teleop {'ACTIVE' if self._teleop_active else 'IDLE'} - "
                    f"Linear: ({linear_x:.2f}, {linear_y:.2f}), Angular: {angular_z:.2f}"
                )
            
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


class ElapsedTimer:
    def __init__(self, duration_s: float):
        self.duration_s = duration_s
        self.start_ts = time.time()

    def elapsed(self) -> bool:
        return time.time() - self.start_ts >= self.duration_s
    
    def reset(self):
        self.start_ts = time.time()
