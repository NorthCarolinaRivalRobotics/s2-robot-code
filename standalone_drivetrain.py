#!/usr/bin/env python3
"""
Standalone Drivetrain Controller for Rival S2 Robot
Uses zenoh directly for communication without Tide framework
"""

import asyncio
import logging
import time
import math
import sys
import os

# Import Tide models and serialization
try:
    from tide.models import Twist2D, Vector2, Vector3
    from tide.models.serialization import to_zenoh_value, from_zenoh_value
except ImportError:
    print("Error: tide-sdk package not found. Install with: pip install tide-sdk")
    sys.exit(1)

# Add robot-code directory to path to import mecanum_drive
sys.path.append(os.path.join(os.path.dirname(__file__), 'robot-code'))

try:
    import zenoh
except ImportError:
    print("Error: zenoh package not found. Install with: pip install zenoh")
    sys.exit(1)

from mecanum_drive import MecanumDrive

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class StandaloneDriveController:
    """Standalone mecanum drive controller using zenoh for communication."""
    
    def __init__(self, robot_id="cash"):
        self.robot_id = robot_id
        self.drive = None
        self._initialization_lock = asyncio.Lock()
        self._initialization_started = False
        
        # Topic names
        self.cmd_twist_topic = f"{robot_id}/cmd/twist"
        self.state_twist_topic = f"{robot_id}/state/twist"
        self.gyro_topic = f"{robot_id}/sensor/gyro/vel"
        
        # State tracking - use Tide models
        self.current_twist = Twist2D(
            linear=Vector2(x=0.0, y=0.0),
            angular=0.0
        )
        self.imu_angular_velocity = None
        self.last_command_time = 0.0
        self._pending_command = None
        
        # Zenoh session
        self.session = None
        self.cmd_subscriber = None
        self.gyro_subscriber = None
        self.state_publisher = None
        
        # Control loop state
        self._running = False
        
    async def initialize(self):
        """Initialize the drive controller and zenoh session."""
        logger.info("Initializing standalone drive controller...")
        
        # Initialize zenoh session
        try:
            config = zenoh.Config()
            self.session = zenoh.open(config)
            logger.info("Zenoh session opened")
        except Exception as e:
            logger.error(f"Failed to open zenoh session: {e}")
            raise
        
        # Initialize mecanum drive
        await self._ensure_drive_initialized()
        
        # Setup zenoh subscriptions and publishers
        await self._setup_zenoh_communication()
        
        logger.info("Standalone drive controller initialized successfully")
    
    async def _ensure_drive_initialized(self):
        """Ensure mecanum drive is initialized."""
        if self.drive is not None:
            return
            
        async with self._initialization_lock:
            if self.drive is not None:
                return
                
            if self._initialization_started:
                return
                
            self._initialization_started = True
            try:
                logger.info("Initializing mecanum drive...")
                self.drive = MecanumDrive()
                await self.drive.set_stops()
                logger.info("Mecanum drive initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize mecanum drive: {e}")
                self.drive = None
                self._initialization_started = False
                raise
    
    async def _setup_zenoh_communication(self):
        """Setup zenoh subscribers and publishers."""
        try:
            # Subscribe to cmd/twist commands
            self.cmd_subscriber = self.session.declare_subscriber(
                self.cmd_twist_topic, 
                self._on_cmd_twist
            )
            logger.info(f"Subscribed to {self.cmd_twist_topic}")
            
            # Subscribe to IMU gyro data
            self.gyro_subscriber = self.session.declare_subscriber(
                self.gyro_topic,
                self._on_imu_angular_velocity
            )
            logger.info(f"Subscribed to {self.gyro_topic}")
            
            # Publisher for state/twist
            self.state_publisher = self.session.declare_publisher(self.state_twist_topic)
            logger.info(f"Publishing state twist to {self.state_twist_topic}")
            
        except Exception as e:
            logger.error(f"Failed to setup zenoh communication: {e}")
            raise
    
    def _on_cmd_twist(self, sample):
        """Handle incoming twist commands."""
        try:
            # Parse Zenoh payload using Tide serialization
            twist_msg = from_zenoh_value(sample.payload, Twist2D)
            
            # Extract velocities with anti-tip feedback
            pitch_velocity = 0.0
            if self.imu_angular_velocity:
                pitch_velocity = self.imu_angular_velocity.y
            
            anti_tip_feedback = pitch_velocity * 0.00  # Disabled for now
            
            linear_x = twist_msg.linear.x + anti_tip_feedback
            linear_y = twist_msg.linear.y
            angular_z = twist_msg.angular
            
            logger.debug(f"Received cmd_twist: linear=({linear_x:.3f}, {linear_y:.3f}), angular={angular_z:.3f}")
            
            # Update command time
            self.last_command_time = time.time()
            
            # Execute drive command directly in callback thread
            # We'll handle this synchronously to avoid task scheduling issues
            self._schedule_drive_command(linear_x, linear_y, angular_z)
            
        except (Exception) as e:
            logger.error(f"Error parsing twist command: {e}, payload type: {type(sample.payload)}")
    
    def _on_imu_angular_velocity(self, sample):
        """Handle incoming IMU angular velocity data."""
        try:
            # Parse Zenoh payload using Tide serialization
            self.imu_angular_velocity = from_zenoh_value(sample.payload, Vector3)
            logger.debug(f"IMU angular velocity: x={self.imu_angular_velocity.x:.3f}, "
                        f"y={self.imu_angular_velocity.y:.3f}, z={self.imu_angular_velocity.z:.3f}")
        except (Exception) as e:
            logger.error(f"Error parsing IMU data: {e}, payload type: {type(sample.payload)}")
    
    def _schedule_drive_command(self, linear_x, linear_y, angular_z):
        """Schedule drive command to be executed in the main loop."""
        self._pending_command = {
            'linear_x': linear_x,
            'linear_y': linear_y,
            'angular_z': angular_z,
            'timestamp': time.time()
        }
    
    async def _execute_drive_command(self, linear_x, linear_y, angular_z):
        """Execute drive command and update state."""
        try:
            await self._ensure_drive_initialized()
            if self.drive is None:
                logger.warning("Drive not initialized, skipping command")
                return
            
            # Execute drive command with velocity query
            wheel_velocities = await self.drive.drive(
                linear_x, linear_y, angular_z, query_velocities=True
            )
            
            # Update and publish state if we got velocity feedback
            if wheel_velocities:
                await self._update_and_publish_state(wheel_velocities)
            
        except Exception as e:
            logger.error(f"Error executing drive command: {e}")
    
    async def _update_and_publish_state(self, wheel_velocities):
        """Update current twist state from wheel velocities and publish."""
        try:
            # Extract wheel speeds
            fl = wheel_velocities.get('front_left', 0.0)
            fr = wheel_velocities.get('front_right', 0.0)
            bl = wheel_velocities.get('back_left', 0.0)
            br = wheel_velocities.get('back_right', 0.0)
            
            # Calculate robot velocities using mecanum kinematics
            linear_x = (fl + fr + bl + br) / 4.0
            linear_y = (-fl + fr + bl - br) / 4.0
            
            # Use IMU angular velocity if available, otherwise estimate from wheels
            if self.imu_angular_velocity:
                angular_z = self.imu_angular_velocity.z
            else:
                # Estimate angular velocity from wheels (robot width ~30cm)
                robot_width = 0.3
                angular_z = (-fl - fr + bl + br) / (4.0 * robot_width)
            
            # Update current state using Tide models
            self.current_twist = Twist2D(
                linear=Vector2(x=linear_x, y=linear_y),
                angular=angular_z
            )
            
            # Publish state
            await self._publish_state()
            
        except Exception as e:
            logger.error(f"Error updating state from wheels: {e}")
    
    async def _publish_state(self):
        """Publish current twist state."""
        try:
            if self.state_publisher:
                payload = to_zenoh_value(self.current_twist)
                self.state_publisher.put(payload)
                logger.debug(f"Published state: linear=({self.current_twist.linear.x:.3f}, "
                           f"{self.current_twist.linear.y:.3f}), angular={self.current_twist.angular:.3f}")
        except Exception as e:
            logger.error(f"Error publishing state: {e}")
    
    async def _control_loop(self):
        """Main control loop for periodic tasks."""
        logger.info("Starting control loop")
        step_count = 0
        
        while self._running:
            try:
                step_count += 1
                current_time = time.time()
                
                # Process any pending drive commands
                if self._pending_command is not None:
                    cmd = self._pending_command
                    self._pending_command = None  # Clear it first
                    await self._execute_drive_command(
                        cmd['linear_x'], cmd['linear_y'], cmd['angular_z']
                    )
                
                # Check for idle state and query velocities
                elif current_time - self.last_command_time > 0.5:  # 500ms timeout
                    if step_count % 10 == 0:  # Every ~3.33 seconds at 30Hz
                        await self._query_idle_velocities()
                
                # Log status periodically
                if step_count % 900 == 0:  # Every 30 seconds at 30Hz
                    logger.info(f"Control loop running - step {step_count}")
                    if self.imu_angular_velocity:
                        logger.info(f"IMU angular velocity: x={self.imu_angular_velocity.x:.3f}, "
                                   f"y={self.imu_angular_velocity.y:.3f}, z={self.imu_angular_velocity.z:.3f}")
                    logger.info(f"Current twist: linear=({self.current_twist.linear.x:.3f}, "
                               f"{self.current_twist.linear.y:.3f}), angular={self.current_twist.angular:.3f}")
                
                await asyncio.sleep(1.0 / 30.0)  # 30Hz control loop
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                await asyncio.sleep(1.0)
    
    async def _query_idle_velocities(self):
        """Query current velocities when robot is idle."""
        try:
            await self._ensure_drive_initialized()
            if self.drive is None:
                return
            
            # Send zero command with query to get current state
            wheel_velocities = await self.drive.drive(0.0, 0.0, 0.0, query_velocities=True)
            if wheel_velocities:
                await self._update_and_publish_state(wheel_velocities)
                
        except Exception as e:
            logger.error(f"Error querying idle velocities: {e}")
    
    async def start(self):
        """Start the drive controller."""
        logger.info("Starting standalone drive controller...")
        
        await self.initialize()
        
        self._running = True
        # Don't use create_task to avoid async issues - run control loop directly
        # We'll handle this differently in main()
        
        logger.info("Standalone drive controller started successfully")
    
    async def stop(self):
        """Stop the drive controller."""
        logger.info("Stopping standalone drive controller...")
        
        self._running = False
        
        # Control loop will stop on next iteration when _running becomes False
        
        # Stop motors
        if self.drive:
            try:
                await self.drive.stop_all_motors()
                logger.info("Motors stopped")
            except Exception as e:
                logger.error(f"Error stopping motors: {e}")
        
        # Close zenoh session
        if self.session:
            try:
                self.session.close()
                logger.info("Zenoh session closed")
            except Exception as e:
                logger.error(f"Error closing zenoh session: {e}")
        
        logger.info("Standalone drive controller stopped")


async def main():
    """Main function to run the standalone drive controller."""
    controller = StandaloneDriveController("cash")
    
    try:
        await controller.start()
        
        # Run the control loop directly - no create_task needed
        logger.info("Drive controller running. Press Ctrl+C to stop.")
        await controller._control_loop()
            
    except KeyboardInterrupt:
        logger.info("Received interrupt signal")
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
    finally:
        await controller.stop()


if __name__ == "__main__":
    asyncio.run(main())
