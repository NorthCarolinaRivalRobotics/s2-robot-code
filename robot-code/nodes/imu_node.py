#!/usr/bin/env python3
"""
IMU Node for Tide Framework
Publishes IMU quaternion and gyroscope data using BNO055 sensor
"""

import time
import logging
import asyncio
from typing import Optional, Tuple

import board
import adafruit_bno055

from tide.core.node import BaseNode
from tide.namespaces import robot_topic
from tide.models import Quaternion, Vector3
from tide.models.serialization import to_zenoh_value

logger = logging.getLogger(__name__)


class IMUNode(BaseNode):
    """
    Tide node that publishes IMU quaternion and gyroscope data from BNO055 sensor.
    Publishes to sensor/imu/quat and sensor/gyro/vel topics following Tide namespacing conventions.
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
        self.quat_topic = robot_topic(self.robot_id, "sensor/imu/quat")
        self.gyro_topic = robot_topic(self.robot_id, "sensor/gyro/vel")
        
        # Initialize sensor to None - will be initialized lazily
        self.sensor = None
        self._initialization_started = False
        self._initialization_lock = asyncio.Lock()
        self._last_successful_read = 0
        
        logger.info(f"IMU Node initialized for robot {self.robot_id}")
        logger.info(f"Publishing quaternion data to: {self.quat_topic}")
        logger.info(f"Publishing gyro velocity data to: {self.gyro_topic}")
        logger.info(f"Update rate: {self.update_rate} Hz")
    
    async def _ensure_initialized(self):
        """Ensure IMU sensor is initialized, initializing if necessary."""
        if self.sensor is not None:
            return True
            
        async with self._initialization_lock:
            # Double-check after acquiring lock
            if self.sensor is not None:
                return True
                
            if self._initialization_started:
                return False
                
            self._initialization_started = True
            try:
                logger.info("Initializing BNO055 IMU sensor...")
                
                # Initialize I2C and sensor
                i2c = board.I2C()  # uses board.SCL and board.SDA
                self.sensor = adafruit_bno055.BNO055_I2C(i2c)
                
                # Test sensor connection by reading temperature
                _ = self.sensor.temperature
                
                logger.info("BNO055 IMU sensor initialized successfully")
                return True
                
            except Exception as e:
                logger.error(f"Failed to initialize BNO055 IMU sensor: {e}")
                self.sensor = None
                self._initialization_started = False
                return False
    
    def _read_quaternion(self) -> Optional[Tuple[float, float, float, float]]:
        """
        Read quaternion data from the IMU sensor.
        
        Returns:
            Tuple of (w, x, y, z) quaternion components, or None if read fails
        """
        if self.sensor is None:
            return None
            
        try:
            # Read quaternion from sensor
            quat = self.sensor.quaternion
            
            if quat is None or None in quat:
                logger.warning("IMU sensor returned None quaternion")
                return None
            
            # BNO055 returns quaternion as (w, x, y, z)
            w, x, y, z = quat
            
            # Validate quaternion magnitude (should be close to 1.0)
            magnitude = (w*w + x*x + y*y + z*z) ** 0.5
            if abs(magnitude - 1.0) > 0.1:  # Allow some tolerance
                logger.warning(f"Quaternion magnitude unusual: {magnitude:.3f}")
            
            self._last_successful_read = time.time()
            return (w, x, y, z)
            
        except Exception as e:
            logger.error(f"Error reading quaternion from IMU: {e}")
            return None
    
    def _read_gyroscope(self) -> Optional[Tuple[float, float, float]]:
        """
        Read gyroscope data from the IMU sensor.
        
        Returns:
            Tuple of (x, y, z) angular velocities in rad/sec, or None if read fails
        """
        if self.sensor is None:
            return None
            
        try:
            # Read gyroscope from sensor
            gyro = self.sensor.gyro
            
            if gyro is None or None in gyro:
                logger.warning("IMU sensor returned None gyroscope data")
                return None
            
            # BNO055 returns gyroscope as (x, y, z) in rad/sec
            x, y, z = gyro
            
            self._last_successful_read = time.time()
            return (x, y, z)
            
        except Exception as e:
            logger.error(f"Error reading gyroscope from IMU: {e}")
            return None
    
    def _create_quaternion_message(self, w: float, x: float, y: float, z: float) -> Quaternion:
        """Create a Tide Quaternion message."""
        return Quaternion(w=w, x=x, y=y, z=z)
    
    def _create_gyro_message(self, x: float, y: float, z: float) -> Vector3:
        """Create a Tide Vector3 message for gyroscope data."""
        return Vector3(x=x, y=y, z=z)
    
    def step(self):
        """
        Main step function - called periodically to read and publish IMU data.
        """
        try:
            # Ensure sensor is initialized
            if not asyncio.run(self._ensure_initialized()):
                logger.error("IMU sensor not available, skipping step")
                return
            
            # Read quaternion data
            quat_data = self._read_quaternion()
            gyro_data = self._read_gyroscope()
            
            # Check if we've had any successful reads recently
            if quat_data is None and gyro_data is None:
                time_since_last_read = time.time() - self._last_successful_read
                if time_since_last_read > 5.0:  # 5 seconds timeout
                    logger.warning("No successful IMU reads for 5 seconds")
                return
            
            # Publish quaternion data if available
            if quat_data is not None:
                w, x, y, z = quat_data
                quat_msg = self._create_quaternion_message(w, x, y, z)
                self.put(self.quat_topic, to_zenoh_value(quat_msg))
            
            # Publish gyroscope data if available
            if gyro_data is not None:
                gx, gy, gz = gyro_data
                gyro_msg = self._create_gyro_message(gx, gy, gz)
                self.put(self.gyro_topic, to_zenoh_value(gyro_msg))
            
            # Log status periodically (every 3 seconds)
            if hasattr(self, '_step_count'):
                self._step_count += 1
            else:
                self._step_count = 1
                
            if self._step_count % (int(self.update_rate) * 3) == 0:
                quat_str = f"Quat: w={w:.3f}, x={x:.3f}, y={y:.3f}, z={z:.3f}" if quat_data else "Quat: N/A"
                gyro_str = f"Gyro: x={gx:.3f}, y={gy:.3f}, z={gz:.3f}" if gyro_data else "Gyro: N/A"
                logger.info(f"IMU active - {quat_str}, {gyro_str}")
            
        except Exception as e:
            logger.error(f"Error in IMU step: {e}")
    
    def cleanup(self):
        """
        Cleanup when node is shutting down.
        """
        logger.info("IMU Node shutting down")
        
        # Reset sensor
        self.sensor = None
        
        super().cleanup()

