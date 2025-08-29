#!/usr/bin/env python3
"""
Robot Odometry Node for Tide Framework
Subscribes to robot velocity twists and IMU data to compute odometry using exponential map
"""

import time
import logging
import math
import numpy as np
from typing import Optional, Tuple

from tide.core.node import BaseNode
from tide.core.geometry import SE2, SO2
from tide.models import Pose2D
from tide.models.serialization import to_zenoh_value


def robot_topic(robot_id: str, topic: str) -> str:
    """Create a robot-specific topic name."""
    return f"/{robot_id}/{topic}"

logger = logging.getLogger(__name__)


class RobotOdometryNode(BaseNode):
    """
    Tide node that computes robot odometry by subscribing to velocity twists and IMU data.
    Uses IMU yaw measurements for more accurate angular velocity and SE2 exponential map for pose integration.
    """
    
    def __init__(self, *, config=None):
        super().__init__(config=config)
        
        # Default configuration
        self.robot_id = "cash"
        self.update_rate = 30.0  # Hz
        
        # Override from config
        if config:
            self.robot_id = config.get("robot_id", self.robot_id)
            self.update_rate = config.get("update_rate", self.update_rate)
        
        # Set update rate
        self.hz = self.update_rate
        
        # Topic names following Tide namespacing
        self.state_twist_topic = robot_topic(self.robot_id, "state/twist")
        self.imu_quat_topic = robot_topic(self.robot_id, "sensor/imu/quat")
        self.gyro_topic = robot_topic(self.robot_id, "sensor/gyro/vel")
        self.pose_topic = robot_topic(self.robot_id, "state/pose2d")
        
        # Subscribe to topics
        self.subscribe(self.state_twist_topic, self.on_state_twist)
        self.subscribe(self.imu_quat_topic, self.on_imu_quaternion)
        self.subscribe(self.gyro_topic, self.on_gyro_velocity)
        
        # State variables
        self.current_pose = SE2.exp(np.array([0.0, 0.0, 0.0]))  # Start at origin
        self.last_twist = None
        self.last_timestamp = None
        
        # IMU state
        self.current_quaternion = None
        self.last_quaternion = None
        self.current_gyro = None
        self.last_yaw = None
        self.yaw_initialized = False
        
        # Yaw rezeroing state
        self.initial_yaw_offset = None
        self.yaw_measurement_count = 0
        self.yaw_rezeroed = False
        
        logger.info(f"Robot Odometry Node initialized for robot {self.robot_id}")
        logger.info(f"Subscribed to state twist: {self.state_twist_topic}")
        logger.info(f"Subscribed to IMU quaternion: {self.imu_quat_topic}")
        logger.info(f"Subscribed to gyro velocity: {self.gyro_topic}")
        logger.info(f"Update rate: {self.update_rate} Hz")
    
    def on_state_twist(self, sample):
        """
        Handle incoming state twist messages from the drivetrain.
        """
        try:
            # Sample is already parsed by Tide into a dictionary
            # Store the latest twist
            self.last_twist = {
                'linear_x': sample['linear']['x'],
                'linear_y': sample['linear']['y'],
                'angular_z': sample['angular']  # This will be replaced with IMU data
            }
            
            # Update timestamp
            current_time = time.time()
            if self.last_timestamp is not None:
                dt = current_time - self.last_timestamp
                self._update_odometry(dt)
            
            self.last_timestamp = current_time
            
        except Exception as e:
            logger.error(f"Error parsing state twist: {e}")
    
    def on_imu_quaternion(self, sample):
        """
        Handle incoming IMU quaternion data.
        """
        try:
            # Sample is already parsed by Tide into a dictionary
            # Store the quaternion
            self.last_quaternion = self.current_quaternion
            self.current_quaternion = {
                'w': sample['w'],
                'x': sample['x'],
                'y': sample['y'],
                'z': sample['z']
            }
            
            # Convert quaternion to yaw angle
            raw_yaw = self._quaternion_to_yaw(sample['w'], sample['x'], sample['y'], sample['z'])
            
            # Handle yaw rezeroing on first/second measurements
            self.yaw_measurement_count += 1
            
            if not self.yaw_rezeroed:
                if self.yaw_measurement_count == 1:
                    # Store first measurement as potential offset
                    self.initial_yaw_offset = raw_yaw
                    logger.info(f"First yaw measurement captured: {math.degrees(raw_yaw):.1f}°")
                elif self.yaw_measurement_count == 2:
                    # Use second measurement to confirm offset (assuming robot is stationary)
                    # Average the first two measurements for better accuracy
                    self.initial_yaw_offset = (self.initial_yaw_offset + raw_yaw) / 2.0
                    self.yaw_rezeroed = True
                    logger.info(f"Yaw rezeroed! Initial offset: {math.degrees(self.initial_yaw_offset):.1f}°")
            
            # Apply yaw offset if rezeroing is complete
            if self.yaw_rezeroed:
                yaw = self._normalize_angle(raw_yaw - self.initial_yaw_offset)
            else:
                yaw = raw_yaw
            
            if not self.yaw_initialized:
                self.last_yaw = yaw
                self.yaw_initialized = True
            else:
                self.last_yaw = yaw
            
        except Exception as e:
            logger.error(f"Error parsing IMU quaternion: {e}")
    
    def on_gyro_velocity(self, sample):
        """
        Handle incoming gyroscope velocity data.
        """
        try:
            # Sample is already parsed by Tide into a dictionary
            # Store the gyro data (primarily interested in z-axis)
            self.current_gyro = {
                'x': sample['x'],
                'y': sample['y'],
                'z': sample['z']
            }
            
        except Exception as e:
            logger.error(f"Error parsing gyro velocity: {e}")
    
    def _quaternion_to_yaw(self, w: float, x: float, y: float, z: float) -> float:
        """
        Convert quaternion to yaw angle (rotation around z-axis).
        
        Args:
            w, x, y, z: Quaternion components
            
        Returns:
            Yaw angle in radians
        """
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def _normalize_angle(self, angle: float) -> float:
        """
        Normalize angle to [-pi, pi] range.
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _update_odometry(self, dt: float):
        """
        Update robot pose using the exponential map.
        
        Args:
            dt: Time delta in seconds
        """
        if self.last_twist is None:
            return
        
        try:
            # Get linear velocities from drivetrain
            linear_x = self.last_twist['linear_x']
            linear_y = self.last_twist['linear_y']
            
            # Use IMU gyro for angular velocity if available, otherwise fall back to drivetrain
            if self.current_gyro is not None:
                angular_z = self.current_gyro['z']
            else:
                angular_z = self.last_twist['angular_z']
            
            # Create twist vector for SE2 exponential map
            # SE2 twist vector is [v_x, v_y, omega] * dt
            twist_vector = np.array([
                linear_x * dt,
                linear_y * dt,
                angular_z * dt
            ])
            
            # Compute the exponential map to get the pose update
            pose_delta = SE2.exp(twist_vector)
            
            # Update the pose by multiplication (composition)
            self.current_pose = self.current_pose * pose_delta
            if self.yaw_initialized and self.yaw_rezeroed:
                self.current_pose = SE2(SO2(self.last_yaw), self.current_pose.translation)

            # Extract pose components for logging
            pose_matrix = self.current_pose.as_matrix()
            x = pose_matrix[0, 2]
            y = pose_matrix[1, 2]
            theta = math.atan2(pose_matrix[1, 0], pose_matrix[0, 0])

            # Publish Pose2D with timestamp
            ts = time.time()
            pose_msg = Pose2D(timestamp=ts, x=x, y=y, theta=theta)
            try:
                self.put(self.pose_topic, to_zenoh_value(pose_msg))
            except Exception as pub_e:
                logger.error(f"Failed to publish Pose2D: {pub_e}")

            # Log the current pose
            logger.info(
                f"Robot Pose - x: {x:.3f}m, y: {y:.3f}m, theta: {theta:.3f}rad ({math.degrees(theta):.1f}°) -> published {self.pose_topic}"
            )
            
            # Also log the twist components used
            logger.debug(f"Twist used - linear: ({linear_x:.3f}, {linear_y:.3f}), angular: {angular_z:.3f}, dt: {dt:.3f}")
            
        except Exception as e:
            logger.error(f"Error updating odometry: {e}")
    
    def get_current_pose(self) -> Tuple[float, float, float]:
        """
        Get the current robot pose.
        
        Returns:
            Tuple of (x, y, theta) where x,y are in meters and theta is in radians
        """
        try:
            pose_matrix = self.current_pose.as_matrix()
            x = pose_matrix[0, 2]
            y = pose_matrix[1, 2]
            theta = math.atan2(pose_matrix[1, 0], pose_matrix[0, 0])
            return (x, y, theta)
        except Exception:
            return (0.0, 0.0, 0.0)
    
    def reset_pose(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        """
        Reset the robot pose to a specific position.
        
        Args:
            x: X position in meters
            y: Y position in meters  
            theta: Orientation in radians
        """
        try:
            # Create new pose using SE2 exponential map
            self.current_pose = SE2.exp(np.array([x, y, theta]))
            logger.info(f"Pose reset to x: {x:.3f}m, y: {y:.3f}m, theta: {theta:.3f}rad")
        except Exception as e:
            logger.error(f"Error resetting pose: {e}")
    
    def rezero_yaw(self):
        """
        Manually trigger yaw rezeroing. This will reset the yaw offset using the next IMU measurement.
        Useful if the robot has been moved while stationary.
        """
        self.yaw_measurement_count = 0
        self.yaw_rezeroed = False
        self.initial_yaw_offset = None
        logger.info("Yaw rezeroing triggered - will rezero on next two IMU measurements")
    
    def step(self):
        """
        Main step function - called periodically.
        Currently just handles status logging.
        """
        # Initialize step counter
        if not hasattr(self, '_step_count'):
            self._step_count = 0
        self._step_count += 1
        
        # Log status periodically (every 3 seconds)
        if self._step_count % (int(self.update_rate) * 3) == 0:
            x, y, theta = self.get_current_pose()
            
            # Status of data availability
            twist_status = "Available" if self.last_twist else "N/A"
            imu_status = "Available" if self.current_quaternion else "N/A"
            gyro_status = "Available" if self.current_gyro else "N/A"
            yaw_status = "Rezeroed" if self.yaw_rezeroed else f"Calibrating ({self.yaw_measurement_count}/2)"
            
            logger.info(f"Odometry Status - Pose: ({x:.3f}, {y:.3f}, {math.degrees(theta):.1f}°), "
                       f"Twist: {twist_status}, IMU: {imu_status}, Gyro: {gyro_status}, Yaw: {yaw_status}")
    
    def cleanup(self):
        """
        Cleanup when node is shutting down.
        """
        logger.info("Robot Odometry Node shutting down")
        
        # Log final pose
        x, y, theta = self.get_current_pose()
        logger.info(f"Final pose - x: {x:.3f}m, y: {y:.3f}m, theta: {theta:.3f}rad ({math.degrees(theta):.1f}°)")
        
        super().cleanup()
