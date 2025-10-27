#!/usr/bin/python3 -B

import asyncio
import math
import moteus
import moteus_pi3hat
import time 

class MecanumDrive:
    """
    Simple mecanum drive class for controlling 4 wheel mecanum robot.
    
    Wheel configuration:
    - Motor 4: Front Left
    - Motor 1: Front Right  
    - Motor 3: Back Left
    - Motor 2: Back Right
    """
    
    def __init__(self, motor_ids=[4, 1, 3, 2], servo_bus_map=None, transport=None):
        """
        Initialize the mecanum drive.
        
        Args:
            motor_ids: List of motor IDs [front_left, front_right, back_left, back_right]
            servo_bus_map: Optional servo bus mapping for pi3hat
        """
        # Wheel constants for 120mm diameter, 6:1 gear ratio
        self.WHEEL_DIAMETER = 0.104  # 109mm in meters
        self.GEAR_RATIO = 100.0/29.0
        self.WHEEL_CIRCUMFERENCE = self.WHEEL_DIAMETER * math.pi
        
        # Motor configuration
        self.motor_ids = motor_ids
        self.front_left_id = motor_ids[0]
        self.front_right_id = motor_ids[1] 
        self.back_left_id = motor_ids[2]
        self.back_right_id = motor_ids[3]


        self.motor_directons = {
            1: 1,
            2: 1,
            3: -1,
            4: -1
        }
        
        # Store previous motor velocities for fallback
        self.previous_motor_velocities = {motor_id: 0.0 for motor_id in motor_ids}

        # Setup transport
        # Allow an external transport to be provided so multiple controllers can share it.
        if transport is not None:
            self.transport = transport
        else:
            raise ValueError("Transport must be provided")
        # Create motor controllers
        self.motors = {
            motor_id: moteus.Controller(id=motor_id, transport=self.transport) 
            for motor_id in motor_ids
        }




        
    def wheel_speed_to_motor_speed(self, wheel_speed_ms):
        """
        Convert wheel speed in m/s to motor speed in rev/s.
        
        Args:
            wheel_speed_ms: Wheel speed in meters per second
            
        Returns:
            Motor speed in revolutions per second
        """
        return (wheel_speed_ms / self.WHEEL_CIRCUMFERENCE) * self.GEAR_RATIO
    
    def motor_speed_to_wheel_speed(self, motor_speed_revs, motor_id):
        """
        Convert motor speed in rev/s to wheel speed in m/s.
        
        Args:
            motor_speed_revs: Motor speed in revolutions per second
            motor_id: Motor ID to apply correct direction
            
        Returns:
            Wheel speed in meters per second
        """
        direction = self.motor_directons.get(motor_id, 1)
        return (motor_speed_revs / self.GEAR_RATIO) * self.WHEEL_CIRCUMFERENCE * direction
        
    async def set_front_left_velocity(self, velocity_ms):
        """Set front left motor velocity in m/s."""
        motor_speed = self.wheel_speed_to_motor_speed(velocity_ms)
        command = self.motors[self.front_left_id].make_position(
            position=math.nan,  # Position mode disabled
            velocity=motor_speed * self.motor_directons[self.front_left_id],
            maximum_torque=1.0,
            query=False
        )
        await self.transport.cycle([command])
    
    async def set_front_right_velocity(self, velocity_ms):
        """Set front right motor velocity in m/s."""
        motor_speed = self.wheel_speed_to_motor_speed(velocity_ms)
        command = self.motors[self.front_right_id].make_position(
            position=math.nan,
            velocity=motor_speed * self.motor_directons[self.front_right_id],
            maximum_torque=1.0,
            query=False
        )
        await self.transport.cycle([command])
    
    async def set_back_left_velocity(self, velocity_ms):
        """Set back left motor velocity in m/s."""
        motor_speed = self.wheel_speed_to_motor_speed(velocity_ms)
        command = self.motors[self.back_left_id].make_position(
            position=math.nan,
            velocity=motor_speed * self.motor_directons[self.back_left_id],
            maximum_torque=1.0,
            query=False
        )
        await self.transport.cycle([command])
    
    async def set_back_right_velocity(self, velocity_ms):
        """Set back right motor velocity in m/s.""" 
        motor_speed = self.wheel_speed_to_motor_speed(velocity_ms)
        command = self.motors[self.back_right_id].make_position(
            position=math.nan,
            velocity=motor_speed * self.motor_directons[self.back_right_id] ,
            maximum_torque=1.0,
            query=False
        )
        await self.transport.cycle([command])
    
    async def set_all_velocities(self, front_left_ms, front_right_ms, back_left_ms, back_right_ms, query_velocities=False):
        """
        Set all motor velocities simultaneously, optionally querying current velocities.
        
        Args:
            front_left_ms: Front left wheel velocity in m/s
            front_right_ms: Front right wheel velocity in m/s  
            back_left_ms: Back left wheel velocity in m/s
            back_right_ms: Back right wheel velocity in m/s
            query_velocities: If True, also query current velocities and return them
            
        Returns:
            dict or None: If query_velocities=True, returns dict with wheel velocities in m/s
        """
        commands = [
            self.motors[self.front_left_id].make_position(
                position=math.nan,
                velocity=self.wheel_speed_to_motor_speed(front_left_ms) * self.motor_directons[self.front_left_id],
                maximum_torque=1.0,
                accel_limit=200.0,
                query=query_velocities
            ),
            self.motors[self.front_right_id].make_position(
                position=math.nan,
                velocity=self.wheel_speed_to_motor_speed(front_right_ms) * self.motor_directons[self.front_right_id],
                maximum_torque=1.0,
                accel_limit=200.0,
                query=query_velocities
            ),
            self.motors[self.back_left_id].make_position(
                position=math.nan,
                velocity=self.wheel_speed_to_motor_speed(back_left_ms) * self.motor_directons[self.back_left_id],
                maximum_torque=1.0,
                accel_limit=200.0,
                query=query_velocities
            ),
            self.motors[self.back_right_id].make_position(
                position=math.nan,
                velocity=self.wheel_speed_to_motor_speed(back_right_ms) * self.motor_directons[self.back_right_id],
                maximum_torque=1.0,
                accel_limit=200.0,
                query=query_velocities
            )
        ]
        
        results = await self.transport.cycle(commands)
        
        # If querying velocities, parse and return them
        if query_velocities and results:
            wheel_velocities = {}
            for result in results:
                rid = getattr(result, "id", None)
                if rid in self.motor_ids:
                    try:
                        motor_speed = result.values[moteus.Register.VELOCITY]
                        wheel_speed = self.motor_speed_to_wheel_speed(motor_speed, rid)
                        wheel_velocities[rid] = wheel_speed
                        # Update previous velocity for fallback
                        self.previous_motor_velocities[rid] = motor_speed
                    except (KeyError, AttributeError):
                        # Use previous velocity if current read fails
                        if rid in self.previous_motor_velocities:
                            wheel_speed = self.motor_speed_to_wheel_speed(
                                self.previous_motor_velocities[rid], rid
                            )
                            wheel_velocities[rid] = wheel_speed
            
            # Convert to named format
            return {
                'front_left': wheel_velocities.get(self.front_left_id, 0.0),
                'front_right': wheel_velocities.get(self.front_right_id, 0.0),
                'back_left': wheel_velocities.get(self.back_left_id, 0.0),
                'back_right': wheel_velocities.get(self.back_right_id, 0.0)
            }
        
        return None
    
    async def stop_all_motors(self):
        """Stop all motors."""
        await self.set_all_velocities(0.0, 0.0, 0.0, 0.0)
    
    async def set_stops(self):
        for motor in self.motors.values():
            await motor.set_stop()
    
    async def get_wheel_velocities(self):
        """
        Read current wheel velocities from all motors.
        
        Returns:
            dict: Dictionary with motor IDs as keys and wheel speeds in m/s as values
                 Format: {front_left_id: speed, front_right_id: speed, back_left_id: speed, back_right_id: speed}
        """
        # Create query commands for all motors
        commands = [
            self.motors[motor_id].make_position(
                position=math.nan,
                velocity=0.0,  # No movement, just query
                maximum_torque=0.0,  # No torque while querying
                query=True  # Enable query to get feedback
            ) for motor_id in self.motor_ids
        ]
        
        # Execute the commands and get results
        results = await self.transport.cycle(commands)
        
        # Parse the results and convert to wheel speeds
        wheel_velocities = {}
        for result in results:
            rid = getattr(result, "id", None)
            if rid in self.motor_ids:
                try:
                    motor_speed = result.values[moteus.Register.VELOCITY]
                    wheel_speed = self.motor_speed_to_wheel_speed(motor_speed, rid)
                    wheel_velocities[rid] = wheel_speed
                    # Update previous velocity for fallback
                    self.previous_motor_velocities[rid] = motor_speed
                except (KeyError, AttributeError):
                    # Use previous velocity if current read fails
                    if rid in self.previous_motor_velocities:
                        print(f"Failed to read velocity for motor {rid}, using previous value")
                        wheel_speed = self.motor_speed_to_wheel_speed(
                            self.previous_motor_velocities[rid], rid
                        )
                        wheel_velocities[rid] = wheel_speed

        return wheel_velocities
    
    async def get_wheel_velocities_named(self):
        """
        Read current wheel velocities with named keys.
        
        Returns:
            dict: Dictionary with named keys and wheel speeds in m/s
                 Format: {'front_left': speed, 'front_right': speed, 'back_left': speed, 'back_right': speed}
        """
        velocities = await self.get_wheel_velocities()
        return {
            'front_left': velocities.get(self.front_left_id, 0.0),
            'front_right': velocities.get(self.front_right_id, 0.0),
            'back_left': velocities.get(self.back_left_id, 0.0),
            'back_right': velocities.get(self.back_right_id, 0.0)
        }
            
    async def drive(self, linear_x, linear_y, angular_z, query_velocities=False):
        """
        Drive the robot with mecanum kinematics.
        
        Args:
            linear_x: Linear velocity in x direction (m/s)
            linear_y: Linear velocity in y direction (m/s)
            angular_z: Angular velocity around z axis (rad/s)
            query_velocities: If True, also query and return current velocities
            
        Returns:
            dict or None: If query_velocities=True, returns wheel velocities
        """
        front_left_speed = linear_x + linear_y + angular_z
        front_right_speed = linear_x - linear_y - angular_z
        back_left_speed = linear_x - linear_y + angular_z
        back_right_speed = linear_x + linear_y - angular_z
        
        return await self.set_all_velocities(
            front_left_speed, front_right_speed, back_left_speed, back_right_speed,
            query_velocities=query_velocities
        )
# Example usage
async def main():
    """Example of how to use the MecanumDrive class."""
    
    # Create mecanum drive with default motor IDs [4, 1, 3, 2] (front_left, front_right, back_left, back_right)
    drive = MecanumDrive()
    await drive.set_stops()
    
    try:
        # Example: Move forward at 0.5 m/s and read velocities
        start_time = time.time()
        while time.time() - start_time < 2.0:
            print("Moving forward...")
            await drive.set_all_velocities(0.5, 0.5, 0.5, 0.5)
            
            # Read and display actual wheel velocities
            velocities = await drive.get_wheel_velocities_named()
            print(f"Wheel velocities (m/s): FL={velocities['front_left']:.3f}, "
                  f"FR={velocities['front_right']:.3f}, "
                  f"BL={velocities['back_left']:.3f}, "
                  f"BR={velocities['back_right']:.3f}")
            
            await asyncio.sleep(0.1)  # Slower loop for readable output

        # Example: Strafe right and read velocities
        print("\nStrafing right...")
        start_time = time.time()
        while time.time() - start_time < 2.0:
            await drive.set_all_velocities(-0.3, 0.3, 0.3, -0.3)  # Strafe right pattern
            
            velocities = await drive.get_wheel_velocities_named()
            print(f"Strafe velocities (m/s): FL={velocities['front_left']:.3f}, "
                  f"FR={velocities['front_right']:.3f}, "
                  f"BL={velocities['back_left']:.3f}, "
                  f"BR={velocities['back_right']:.3f}")
            
            await asyncio.sleep(0.1)
        
        # Stop and verify all wheels stop
        print("\nStopping...")
        await drive.stop_all_motors()
        
        # Check that wheels have stopped
        await asyncio.sleep(0.5)  # Wait for motors to stop
        final_velocities = await drive.get_wheel_velocities_named()
        print(f"Final velocities (m/s): FL={final_velocities['front_left']:.3f}, "
              f"FR={final_velocities['front_right']:.3f}, "
              f"BL={final_velocities['back_left']:.3f}, "
              f"BR={final_velocities['back_right']:.3f}")
        
    except KeyboardInterrupt:
        print("Emergency stop!")
        await drive.stop_all_motors()


if __name__ == "__main__":
    asyncio.run(main())
