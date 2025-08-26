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
    - Motor 1: Front Left
    - Motor 2: Front Right  
    - Motor 3: Back Left
    - Motor 4: Back Right
    """
    
    def __init__(self, motor_ids=[1, 2, 3, 4], servo_bus_map=None):
        """
        Initialize the mecanum drive.
        
        Args:
            motor_ids: List of motor IDs [front_left, front_right, back_left, back_right]
            servo_bus_map: Optional servo bus mapping for pi3hat
        """
        # Wheel constants for 120mm diameter, 6:1 gear ratio
        self.WHEEL_DIAMETER = 0.109  # 109mm in meters
        self.GEAR_RATIO = 100.0/29.0
        self.WHEEL_CIRCUMFERENCE = self.WHEEL_DIAMETER * math.pi
        
        # Motor configuration
        self.motor_ids = motor_ids
        self.front_left_id = motor_ids[0]
        self.front_right_id = motor_ids[1] 
        self.back_left_id = motor_ids[2]
        self.back_right_id = motor_ids[3]
        
        # Setup transport
        if servo_bus_map is None:
            servo_bus_map = {1: motor_ids}  # Default: all motors on bus 1
        
        self.transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=servo_bus_map)
        
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
    
    async def set_front_left_velocity(self, velocity_ms):
        """Set front left motor velocity in m/s."""
        motor_speed = self.wheel_speed_to_motor_speed(velocity_ms)
        command = self.motors[self.front_left_id].make_position(
            position=math.nan,  # Position mode disabled
            velocity=motor_speed,
            maximum_torque=1.0,
            query=False
        )
        await self.transport.cycle([command])
    
    async def set_front_right_velocity(self, velocity_ms):
        """Set front right motor velocity in m/s."""
        motor_speed = self.wheel_speed_to_motor_speed(velocity_ms)
        command = self.motors[self.front_right_id].make_position(
            position=math.nan,
            velocity=motor_speed,
            maximum_torque=1.0,
            query=False
        )
        await self.transport.cycle([command])
    
    async def set_back_left_velocity(self, velocity_ms):
        """Set back left motor velocity in m/s."""
        motor_speed = self.wheel_speed_to_motor_speed(velocity_ms)
        command = self.motors[self.back_left_id].make_position(
            position=math.nan,
            velocity=motor_speed,
            maximum_torque=1.0,
            query=False
        )
        await self.transport.cycle([command])
    
    async def set_back_right_velocity(self, velocity_ms):
        """Set back right motor velocity in m/s.""" 
        motor_speed = self.wheel_speed_to_motor_speed(velocity_ms)
        command = self.motors[self.back_right_id].make_position(
            position=math.nan,
            velocity=motor_speed,
            maximum_torque=1.0,
            query=False
        )
        await self.transport.cycle([command])
    
    async def set_all_velocities(self, front_left_ms, front_right_ms, back_left_ms, back_right_ms):
        """
        Set all motor velocities simultaneously.
        
        Args:
            front_left_ms: Front left wheel velocity in m/s
            front_right_ms: Front right wheel velocity in m/s  
            back_left_ms: Back left wheel velocity in m/s
            back_right_ms: Back right wheel velocity in m/s
        """
        print(self.wheel_speed_to_motor_speed(front_left_ms))
        commands = [
            self.motors[self.front_left_id].make_position(
                position=math.nan,
                velocity=self.wheel_speed_to_motor_speed(front_left_ms),
                maximum_torque=1.0,
                query=False
            ),
            self.motors[self.front_right_id].make_position(
                position=math.nan,
                velocity=self.wheel_speed_to_motor_speed(front_right_ms),
                maximum_torque=1.0,
                query=False
            ),
            self.motors[self.back_left_id].make_position(
                position=math.nan,
                velocity=self.wheel_speed_to_motor_speed(back_left_ms),
                maximum_torque=1.0,
                query=False
            ),
            self.motors[self.back_right_id].make_position(
                position=math.nan,
                velocity=self.wheel_speed_to_motor_speed(back_right_ms),
                maximum_torque=1.0,
                query=False
            )
        ]
        await self.transport.cycle(commands)
    
    async def stop_all_motors(self):
        """Stop all motors."""
        await self.set_all_velocities(0.0, 0.0, 0.0, 0.0)
    


# Example usage
async def main():
    """Example of how to use the MecanumDrive class."""
    
    # Create mecanum drive with default motor IDs [1, 2, 3, 4]
    drive = MecanumDrive()
    
    try:
        # Example: Move forward at 0.5 m/s
        start_time = time.time()
        while time.time() - start_time < 1.0:
            print("Moving forward...")
            await drive.set_all_velocities(0.5, 0.5, 0.5, 0.5)
            await asyncio.sleep(0.005)

        
        # Stop
        print("Stopping...")
        await drive.stop_all_motors()
        
    except KeyboardInterrupt:
        print("Emergency stop!")
        await drive.stop_all_motors()


if __name__ == "__main__":
    asyncio.run(main())
