#!/usr/bin/env python3
"""
Drivebase Node for Tide Framework
Integrates the existing DriveBase class with Tide's node system
"""


from tide.core.node import BaseNode
from tide import CmdTopic
from tide.models import Twist2D
from tide.namespaces import robot_topic
import logging
import asyncio
from tide import CmdTopic

# Import the existing drivebase module
import sys

import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanum_drive import MecanumDrive

logger = logging.getLogger(__name__)

class DrivebaseNode(BaseNode):
    """
    Tide node that controls the robot's drivebase using PCA9685 PWM controller
    """
    ROBOT_ID = "cash"
    GROUP = "drive"
    
    def __init__(self, *, config=None):
        super().__init__(config=config)
        
        # Initialize drivebase to None - will be initialized lazily
        self.drivebase = None
        self._initialization_started = False
        self._initialization_lock = asyncio.Lock()
        
        # Subscribe to twist commands using proper Tide namespacing
        twist_topic = robot_topic(self.ROBOT_ID, CmdTopic.TWIST.value)
        self.subscribe(twist_topic, self.on_cmd_vel)
        logger.info(f"Subscribed to {twist_topic}")
    
    async def _ensure_initialized(self):
        """Ensure drivebase is initialized, initializing if necessary."""
        if self.drivebase is not None:
            return
            
        async with self._initialization_lock:
            # Double-check after acquiring lock
            if self.drivebase is not None:
                return
                
            if self._initialization_started:
                return
                
            self._initialization_started = True
            try:
                logger.info("Initializing drivebase...")
                self.drivebase = MecanumDrive()
                await self.drivebase.set_stops()
                logger.info("Drivebase initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize Drivebase: {e}")
                self.drivebase = None
                self._initialization_started = False
    
    def on_cmd_vel(self, sample):
        """
        Handle incoming velocity commands
        """
        # Parse the twist command
        twist = sample
        
        try:
            # Extract linear velocities
            linear_x = twist["linear"]["x"]
            linear_y = twist["linear"]["y"]
            
            # Handle angular velocity - it might be a float or an object with z field
            angular_data = twist["angular"]
            angular_z = float(angular_data)
                
        except (KeyError, TypeError, ValueError) as e:
            logger.error(f"Error parsing twist command: {e}, sample: {sample}")
            return
        
        # Send to drivebase asynchronously (fire and forget)
        # Use asyncio.ensure_future instead of create_task for better compatibility
        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                asyncio.ensure_future(self._drive_async(linear_x, linear_y, angular_z))
                logger.debug(f"Drive command: linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular={angular_z:.2f}")
            else:
                logger.warning("Event loop not running, cannot send drive command")
        except RuntimeError:
            logger.warning("No event loop available, cannot send drive command")
    
    async def _drive_async(self, linear_x, linear_y, angular_z):
        """Helper method to handle async drive commands."""
        try:
            # Ensure drivebase is initialized before using it
            await self._ensure_initialized()
            if self.drivebase is not None:
                await self.drivebase.drive(linear_x, linear_y, angular_z)
        except Exception as e:
            logger.error(f"Error in drive command: {e}")
                
    
    def step(self):
        """
        Main step function - called periodically
        For the drivebase, we don't need to do anything here as it's reactive
        """
        pass
    
    def cleanup(self):
        """
        Cleanup when node is shutting down
        """
        if self.drivebase:
            # Stop all motors asynchronously
            try:
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    asyncio.ensure_future(self._cleanup_async())
                else:
                    logger.warning("Event loop not running, cannot perform async cleanup")
            except RuntimeError:
                logger.warning("No event loop available, cannot perform async cleanup")
        
        super().cleanup()
    
    async def _cleanup_async(self):
        """Helper method to handle async cleanup operations."""
        try:
            if self.drivebase is not None:
                await self.drivebase.drive(0.0, 0.0, 0.0)
                logger.info("DriveBase stopped")
        except Exception as e:
            logger.error(f"Error stopping DriveBase: {e}") 