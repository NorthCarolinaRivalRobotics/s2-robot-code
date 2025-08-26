#!/usr/bin/env python3
"""
Drivebase Node for Tide Framework
Integrates the existing DriveBase class with Tide's node system
"""

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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
        
        # Initialize drivebase to None - will be initialized asynchronously
        self.drivebase = None
        self._initialization_task = None
        
        # Subscribe to twist commands using proper Tide namespacing
        twist_topic = robot_topic(self.ROBOT_ID, CmdTopic.TWIST.value)
        self.subscribe(twist_topic, self.on_cmd_vel)
        logger.info(f"Subscribed to {twist_topic}")
        
        # Start async initialization
        self._initialization_task = asyncio.create_task(self._async_init())
    
    async def _async_init(self):
        """Asynchronously initialize the drivebase."""
        try:
            self.drivebase = MecanumDrive()
            await self.drivebase.set_stops()
            logger.info(f"Drivebase initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize Drivebase: {e}")
            self.drivebase = None
    
    def on_cmd_vel(self, sample):
        """
        Handle incoming velocity commands
        """
        if self.drivebase is None:
            logger.warning("Drivebase not yet initialized, ignoring command")
            return
            
        # Parse the twist command
        twist = sample
        
        # Extract linear and angular velocities
        linear_x = twist["linear"]["x"]
        linear_y = twist["linear"]["y"]
        angular_z = twist["angular"]["z"]
        
        # Send to drivebase asynchronously (fire and forget)
        asyncio.create_task(self._drive_async(linear_x, linear_y, angular_z))
        logger.debug(f"Drive command: linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular={angular_z:.2f}")
    
    async def _drive_async(self, linear_x, linear_y, angular_z):
        """Helper method to handle async drive commands."""
        try:
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
        # Cancel initialization if still running
        if self._initialization_task and not self._initialization_task.done():
            self._initialization_task.cancel()
            
        if self.drivebase:
            # Stop all motors asynchronously
            try:
                # Create a cleanup task that will be handled by the event loop
                asyncio.create_task(self._cleanup_async())
            except Exception as e:
                logger.error(f"Error scheduling cleanup: {e}")
        
        super().cleanup()
    
    async def _cleanup_async(self):
        """Helper method to handle async cleanup operations."""
        try:
            if self.drivebase is not None:
                await self.drivebase.drive(0.0, 0.0, 0.0)
                logger.info("DriveBase stopped")
        except Exception as e:
            logger.error(f"Error stopping DriveBase: {e}") 