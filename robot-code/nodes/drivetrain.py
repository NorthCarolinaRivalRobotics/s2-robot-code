# #!/usr/bin/env python3
# """
# Drivebase Node for Tide Framework
# Integrates the existing DriveBase class with Tide's node system
# """


# from tide.core.node import BaseNode
# from tide import CmdTopic
# from tide.models import Twist2D, Vector3, Vector2
# from tide.models.serialization import to_zenoh_value
# from tide.namespaces import robot_topic
# import logging
# import asyncio
# import time

# # Import the existing drivebase module
# import sys

# import os
# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# from mecanum_drive import MecanumDrive

# logger = logging.getLogger(__name__)

# class DrivebaseNode(BaseNode):
#     """
#     Tide node that controls the robot's drivebase using PCA9685 PWM controller
#     """
#     ROBOT_ID = "cash"
#     GROUP = "drive"
    
#     def __init__(self, *, config=None):
#         super().__init__(config=config)
        
#         # Initialize drivebase to None - will be initialized lazily
#         self.drivebase = None
#         self._initialization_started = False
#         self._initialization_lock = asyncio.Lock()
        
#         # Store latest IMU angular velocity
#         self.imu_angular_velocity = None
        
#         # Store current twist state for publishing
#         self.current_twist = Twist2D(linear=Vector2(x=0.0, y=0.0), angular=0.0)
#         self.last_command_time = 0.0
        
#         # Subscribe to twist commands using proper Tide namespacing
#         twist_topic = robot_topic(self.ROBOT_ID, CmdTopic.TWIST.value)
#         self.subscribe(twist_topic, self.on_cmd_vel)
#         logger.info(f"Subscribed to {twist_topic}")
        
#         # Subscribe to IMU gyroscope data
#         gyro_topic = robot_topic(self.ROBOT_ID, "sensor/gyro/vel")
#         self.subscribe(gyro_topic, self.on_imu_angular_velocity)
#         logger.info(f"Subscribed to IMU gyro data: {gyro_topic}")
        
#         # State twist topic for publishing current velocity
#         self.state_twist_topic = robot_topic(self.ROBOT_ID, "state/twist")
#         logger.info(f"Publishing state twist to: {self.state_twist_topic}")
    
#     async def _ensure_initialized(self):
#         """Ensure drivebase is initialized, initializing if necessary."""
#         if self.drivebase is not None:
#             return
            
#         async with self._initialization_lock:
#             # Double-check after acquiring lock
#             if self.drivebase is not None:
#                 return
                
#             if self._initialization_started:
#                 return
                
#             self._initialization_started = True
#             try:
#                 logger.info("Initializing drivebase...")
#                 self.drivebase = MecanumDrive()
#                 await self.drivebase.set_stops()
#                 logger.info("Drivebase initialized successfully")
#             except Exception as e:
#                 logger.error(f"Failed to initialize Drivebase: {e}")
#                 self.drivebase = None
#                 self._initialization_started = False
    
#     def on_imu_angular_velocity(self, sample):
#         """
#         Handle incoming IMU angular velocity data
#         """
#         try:
#             # Parse the Vector3 angular velocity data
#             angular_vel = sample
            
#             # Store the latest angular velocity (we're primarily interested in z-axis rotation)
#             self.imu_angular_velocity = {
#                 'x': angular_vel.get('x', 0.0),
#                 'y': angular_vel.get('y', 0.0),  
#                 'z': angular_vel.get('z', 0.0)
#             }
            
#             logger.debug(f"IMU angular velocity: x={self.imu_angular_velocity['x']:.3f}, "
#                         f"y={self.imu_angular_velocity['y']:.3f}, z={self.imu_angular_velocity['z']:.3f}")
                        
#         except (KeyError, TypeError, ValueError) as e:
#             logger.error(f"Error parsing IMU angular velocity: {e}, sample: {sample}")
    
#     def on_cmd_vel(self, sample):
#         """
#         Handle incoming velocity commands
#         """
#         # Parse the twist command
#         twist = sample
        
#         try:

#             pitch_velocity = self.get_imu_angular_velocity()['y']
#             anti_tip_feedback = pitch_velocity * 0.00
#             print(f"IMU angular velocity: {pitch_velocity}")
#             # Extract linear velocities
#             linear_x = twist["linear"]["x"] + anti_tip_feedback
#             linear_y = twist["linear"]["y"]
            
#             # Handle angular velocity - it might be a float or an object with z field
#             angular_data = twist["angular"]
#             angular_z = float(angular_data)
                
#         except (KeyError, TypeError, ValueError) as e:
#             logger.error(f"Error parsing twist command: {e}, sample: {sample}")
#             return
        
#         # Update last command time
#         self.last_command_time = time.time()
        
#         # Send to drivebase asynchronously 
#         try:
#             # Try to get existing event loop first
#             try:
#                 loop = asyncio.get_running_loop()
#                 # If we have a running loop, schedule the task
#                 asyncio.create_task(self._drive_async(linear_x, linear_y, angular_z))
#                 logger.debug(f"Drive command scheduled: linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular={angular_z:.2f}")
#             except RuntimeError:
#                 # No running loop, create a new thread to run the async operation
#                 import threading
#                 def run_async():
#                     asyncio.run(self._drive_async(linear_x, linear_y, angular_z))
                
#                 thread = threading.Thread(target=run_async, daemon=True)
#                 thread.start()
#                 logger.debug(f"Drive command (threaded): linear_x={linear_x:.2f}, linear_y={linear_y:.2f}, angular={angular_z:.2f}")
                
#         except Exception as e:
#             logger.error(f"Error scheduling drive command: {e}")
    
#     def get_imu_angular_velocity(self):
#         """
#         Get the latest IMU angular velocity data.
        
#         Returns:
#             dict: Angular velocity with 'x', 'y', 'z' components, or None if no data available
#         """
#         return self.imu_angular_velocity
    
#     def get_current_twist(self):
#         """
#         Get the latest twist state.
        
#         Returns:
#             Twist2D: Current robot twist based on wheel velocities and IMU
#         """
#         return self.current_twist
    

    
#     async def _drive_async(self, linear_x, linear_y, angular_z):
#         """Helper method to handle async drive commands and read velocities simultaneously."""
#         try:
#             # Ensure drivebase is initialized before using it
#             await self._ensure_initialized()
#             if self.drivebase is not None:
#                 # Use combined command+query for better performance
#                 wheel_velocities = await self.drivebase.drive(
#                     linear_x, linear_y, angular_z, query_velocities=True
#                 )
                
#                 # If we got velocity data, update twist state and publish
#                 if wheel_velocities:
#                     await self._update_twist_from_wheels(wheel_velocities)
                
#         except Exception as e:
#             logger.error(f"Error in drive command: {e}")
    
#     async def _update_twist_from_wheels(self, wheel_velocities):
#         """Update twist state from wheel velocities and publish."""
#         try:
#             # Convert wheel velocities to robot twist
#             fl = wheel_velocities.get('front_left', 0.0)
#             fr = wheel_velocities.get('front_right', 0.0)
#             bl = wheel_velocities.get('back_left', 0.0)
#             br = wheel_velocities.get('back_right', 0.0)
            
#             # Calculate linear velocities from wheel speeds (mecanum kinematics)
#             linear_x = (fl + fr + bl + br) / 4.0
#             linear_y = (-fl + fr + bl - br) / 4.0
            
#             # Use IMU angular velocity if available, otherwise estimate from wheels
#             if self.imu_angular_velocity and 'z' in self.imu_angular_velocity:
#                 angular_z = self.imu_angular_velocity['z']
#             else:
#                 # Estimate angular velocity from wheels (requires robot width)
#                 robot_width_estimate = 0.3  # 30cm, adjust based on actual robot
#                 angular_z = (-fl - fr + bl + br) / (4.0 * robot_width_estimate)
            
#             # Create and update current twist
#             twist_msg = Twist2D(
#                 linear=Vector2(x=linear_x, y=linear_y),
#                 angular=angular_z
#             )
            
#             self.current_twist = twist_msg
            
#             # Publish to state twist topic
#             self.put(self.state_twist_topic, to_zenoh_value(twist_msg))
            
#         except Exception as e:
#             logger.error(f"Error updating twist from wheels: {e}")
                
    
#     def step(self):
#         """
#         Main step function - called periodically
#         Twist state is now published automatically during drive commands for better performance
#         """
#         # Initialize step counter
#         if hasattr(self, '_step_count'):
#             self._step_count += 1
#         else:
#             self._step_count = 1
        
#         # If we haven't received a command in a while, query velocities to update state
#         current_time = time.time()
#         if current_time - self.last_command_time > 0.5:  # 500ms timeout
#             # Query velocities less frequently when idle to reduce overhead
#             if self._step_count % 10 == 0:  # Every ~333ms at 30Hz
#                 try:
#                     loop = asyncio.get_running_loop()
#                     asyncio.create_task(self._query_idle_velocities())
#                 except RuntimeError:
#                     # No running loop, skip idle query to avoid threading overhead
#                     pass
            
#         # Log IMU angular velocity periodically for monitoring
#         if self._step_count % 90 == 0 and self.imu_angular_velocity is not None:
#             logger.info(f"IMU angular velocity available - "
#                        f"x: {self.imu_angular_velocity['x']:.3f}, "
#                        f"y: {self.imu_angular_velocity['y']:.3f}, "
#                        f"z: {self.imu_angular_velocity['z']:.3f} rad/s")
        
#         # Log current twist state periodically
#         if self._step_count % 90 == 0:
#             logger.info(f"Current twist state - "
#                        f"linear: ({self.current_twist.linear.x:.3f}, {self.current_twist.linear.y:.3f}), "
#                        f"angular: {self.current_twist.angular:.3f}")
    
#     async def _query_idle_velocities(self):
#         """Query velocities when robot is idle to maintain state accuracy."""
#         try:
#             await self._ensure_initialized()
#             if self.drivebase is not None:
#                 # Send zero command with query to get current velocities
#                 wheel_velocities = await self.drivebase.drive(0.0, 0.0, 0.0, query_velocities=True)
#                 if wheel_velocities:
#                     await self._update_twist_from_wheels(wheel_velocities)
#         except Exception as e:
#             logger.error(f"Error querying idle velocities: {e}")
    

    
#     def cleanup(self):
#         """
#         Cleanup when node is shutting down
#         """
#         if self.drivebase:
#             # Stop all motors asynchronously
#             try:
#                 # Try to get existing event loop first
#                 try:
#                     loop = asyncio.get_running_loop()
#                     asyncio.create_task(self._cleanup_async())
#                 except RuntimeError:
#                     # No running loop, run cleanup synchronously
#                     import threading
#                     def run_cleanup():
#                         asyncio.run(self._cleanup_async())
                    
#                     thread = threading.Thread(target=run_cleanup, daemon=True)
#                     thread.start()
#                     # Wait a bit for cleanup to complete
#                     thread.join(timeout=2.0)
                    
#             except Exception as e:
#                 logger.error(f"Error during cleanup: {e}")
        
#         super().cleanup()
    
#     async def _cleanup_async(self):
#         """Helper method to handle async cleanup operations."""
#         try:
#             if self.drivebase is not None:
#                 await self.drivebase.drive(0.0, 0.0, 0.0)
#                 logger.info("DriveBase stopped")
#         except Exception as e:
#             logger.error(f"Error stopping DriveBase: {e}") 