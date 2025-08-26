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
from tide import CmdTopic

# Import the existing drivebase module
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from drivebase import DriveBase

logger = logging.getLogger(__name__)

class DrivebaseNode(BaseNode):
    """
    Tide node that controls the robot's drivebase using PCA9685 PWM controller
    """
    ROBOT_ID = "frogbot"
    GROUP = "drive"
    
    def __init__(self, *, config=None):
        super().__init__(config=config)
        
        # Get configuration parameters
        self.address = self.config.get('address', 0x40)
        self.busnum = self.config.get('busnum', 1)
        
        # Initialize the drivebase
        try:
            self.drivebase = DriveBase(address=self.address, busnum=self.busnum)
            logger.info(f"DriveBase initialized with address=0x{self.address:02x}, busnum={self.busnum}")
        except Exception as e:
            logger.error(f"Failed to initialize DriveBase: {e}")
            self.drivebase = None
        
        # Subscribe to twist commands using proper Tide namespacing
        twist_topic = robot_topic(self.ROBOT_ID, CmdTopic.TWIST.value)
        self.subscribe(twist_topic, self.on_cmd_vel)
        logger.info(f"Subscribed to {twist_topic}")
    
    def on_cmd_vel(self, sample):
        """
        Handle incoming velocity commands
        """
        try:
            # Parse the twist command
            twist = sample
            
            # Extract linear and angular velocities
            linear_x = twist["linear"]["x"]
            angular_z = twist["angular"]
            
            # Send to drivebase if available
            if self.drivebase:
                self.drivebase.drive(linear_x, angular_z)
                logger.debug(f"Drive command: linear={linear_x:.2f}, angular={angular_z:.2f}")
            else:
                logger.warning("DriveBase not available - skipping command")
                
        except Exception as e:
            logger.error(f"Error processing velocity command: {e}")
    
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
            # Stop all motors
            try:
                self.drivebase.drive(0.0, 0.0)
                logger.info("DriveBase stopped")
            except Exception as e:
                logger.error(f"Error stopping DriveBase: {e}")
        
        super().cleanup() 