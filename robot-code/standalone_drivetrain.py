#!/usr/bin/env python3
"""
Simplified Standalone Drivetrain Controller for Rival S2 Robot
Uses zenoh directly for communication, adopting patterns from working example
"""

import asyncio
import time
import math
import sys
import os
import json

from tide import CmdTopic, StateTopic, robot_topic

# Import Tide models and serialization
try:
    from tide.models import Twist2D, Vector2, Vector3
    from tide.models.serialization import to_zenoh_value, from_zenoh_value
except ImportError:
    print("Error: tide-sdk package not found. Install with: pip install tide-sdk")
    sys.exit(1)

# Add current directory to path to import mecanum_drive
sys.path.append(os.path.dirname(__file__))

try:
    import zenoh
except ImportError:
    print("Error: zenoh package not found. Install with: pip install zenoh")
    sys.exit(1)

from mecanum_drive import MecanumDrive
from moteus_hardware import MoteusHardware

# Global state variables (following example_moteus_swerve.py pattern)
last_recv = time.monotonic()
reference_vx = 0.0  # m/s
reference_vy = 0.0  # m/s
reference_w = 0.0   # rad/s
WATCHDOG_TIMEOUT = 0.5

# Robot ID and Zenoh topics using Tide conventions
ROBOT_ID = "cash"
VELOCITY_KEY = robot_topic(ROBOT_ID, CmdTopic.TWIST.value).strip('/')
STATE_TWIST_KEY = robot_topic(ROBOT_ID, StateTopic.TWIST.value).strip('/')
GYRO_KEY = robot_topic(ROBOT_ID, "sensor/imu/gyro").strip('/')

# Hardware (shared transport for drivetrain + arm)
hardware: MoteusHardware | None = None
drive: MecanumDrive | None = None

# Arm control state
arm_target = Vector2(x=0.0, y=0.0)  # shoulder, elbow in joint rotations
ARM_CMD_TIMEOUT = 2.0
last_arm_recv = 0.0

def zenoh_velocity_listener(sample):
    """Callback for incoming velocity commands using Tide serialization."""
    global reference_vx, reference_vy, reference_w, last_recv
    try:
        # Parse Zenoh payload using Tide serialization
        twist_msg = from_zenoh_value(sample.payload, Twist2D)
        reference_vx = twist_msg.linear.x
        reference_vy = twist_msg.linear.y
        reference_w = twist_msg.angular
        last_recv = time.monotonic()
        print(f"Received cmd: vx={reference_vx:.3f}, vy={reference_vy:.3f}, w={reference_w:.3f}")
    except Exception as e:
        print(f"Failed to parse velocity command: {e}")

# Global variable to store IMU data for potential anti-tip functionality
imu_angular_velocity = None

def zenoh_gyro_listener(sample):
    """Callback for IMU gyro data using Tide serialization."""
    global imu_angular_velocity
    try:
        # Parse Zenoh payload using Tide serialization
        imu_angular_velocity = from_zenoh_value(sample.payload, Vector3)
        # For now, just store it - could be used for anti-tip feedback later
        print(f"Received gyro: x={imu_angular_velocity.x:.3f}, y={imu_angular_velocity.y:.3f}, z={imu_angular_velocity.z:.3f}")
    except Exception as e:
        print(f"Failed to parse gyro data: {e}")

async def initialize_drive():
    """Initialize the mecanum drive system."""
    global hardware, drive
    if hardware is None:
        print("Initializing shared moteus hardware (drive + arm)...")
        hardware = MoteusHardware(robot_id=ROBOT_ID)
        await hardware.initialize()
        drive = hardware.drive
        print("Hardware initialized successfully")
    return drive  # type: ignore[return-value]

async def main():
    """Main function - simple control loop following example_moteus_swerve.py pattern."""
    global drive, reference_vx, reference_vy, reference_w, last_recv
    
    print("Starting simplified standalone drivetrain controller...")
    
    # Initialize Zenoh session
    z_conf = zenoh.Config()
    session = zenoh.open(z_conf)
    
    # Subscribe to velocity commands and gyro data
    _ = session.declare_subscriber(VELOCITY_KEY, zenoh_velocity_listener)
    _ = session.declare_subscriber(GYRO_KEY, zenoh_gyro_listener)
    
    # Arm command subscription (Vector2: x=shoulder, y=elbow in joint revolutions)
    ARM_CMD_KEY = robot_topic(ROBOT_ID, "cmd/arm/target")
    print(f"DEBUG: ARM_CMD_KEY = '{ARM_CMD_KEY}'")
    def _arm_cmd_listener(sample):
        global last_arm_recv, arm_target
        print(f"DEBUG: Received arm command on topic '{sample.key_expr}'")
        try:
            vec = from_zenoh_value(sample.payload, Vector2)
            arm_target = Vector2(x=float(vec.x), y=float(vec.y))
            last_arm_recv = time.monotonic()
            print(f"Arm cmd: shoulder={arm_target.x:.3f} rev, elbow={arm_target.y:.3f} rev")
        except Exception as e:
            print(f"Failed to parse arm command: {e}")
    arm_subscriber = session.declare_subscriber(ARM_CMD_KEY, _arm_cmd_listener)
    print(f"DEBUG: Declared subscriber for '{ARM_CMD_KEY}'")
    
    # Publisher for state twist
    state_twist_pub = session.declare_publisher(STATE_TWIST_KEY)
    # Publisher for arm state (Vector2: joint rotations)
    ARM_STATE_KEY = robot_topic(ROBOT_ID, "state/arm/position")
    arm_state_pub = session.declare_publisher(ARM_STATE_KEY)
    
    print(f"Subscribed to {VELOCITY_KEY}")
    print(f"Subscribed to {GYRO_KEY}")
    print(f"Publishing state to {STATE_TWIST_KEY}")
    print(f"Subscribed to {ARM_CMD_KEY}")
    print(f"Publishing arm state to {ARM_STATE_KEY}")
    
    # Initialize the drive
    await initialize_drive()
    
    print("Drive controller running. Press Ctrl+C to stop.")
    
    try:
        loop_start = time.monotonic()
        step_count = 0
        
        while True:
            dt = time.monotonic() - loop_start
            loop_start = time.monotonic()
            step_count += 1
            
            # Watchdog - stop if no commands received recently
            if loop_start - last_recv > WATCHDOG_TIMEOUT:
                reference_vx = 0.0
                reference_vy = 0.0
                reference_w = 0.0
            
            # Execute drive command if we have velocities
            try:
                # Execute drive command with velocity query
                wheel_velocities = await drive.drive(
                    reference_vx, reference_vy, reference_w, query_velocities=True
                )

                # Command arm position and query state in same cycle (avoid query-only packet)
                arm_state = None
                if hardware is not None:
                    print(f"Arm target: {arm_target.x}, {arm_target.y}")
                    arm_state = await hardware.arm.set_targets(arm_target.x, arm_target.y, query=True)
                
                # Publish state if we got wheel velocities back
                if wheel_velocities:
                    # Calculate robot velocities from wheel speeds (basic mecanum kinematics)
                    fl = wheel_velocities.get('front_left', 0.0)
                    fr = wheel_velocities.get('front_right', 0.0)
                    bl = wheel_velocities.get('back_left', 0.0)
                    br = wheel_velocities.get('back_right', 0.0)
                    
                    # Basic mecanum math
                    actual_vx = (fl + fr + bl + br) / 4.0
                    actual_vy = (-fl + fr + bl - br) / 4.0
                    
                    # Use IMU angular velocity if available, otherwise estimate from wheels
                    if imu_angular_velocity:
                        actual_w = imu_angular_velocity.z
                    else:
                        actual_w = (-fl - fr + bl + br) / (4.0 * 0.3)  # Assume 30cm wheelbase
                    
                    # Publish state using Tide serialization
                    current_twist = Twist2D(
                        linear=Vector2(x=actual_vx, y=actual_vy),
                        angular=actual_w
                    )
                    payload = to_zenoh_value(current_twist)
                    state_twist_pub.put(payload)

                # Publish arm joint positions from same cycle result
                if hardware is not None and arm_state is not None:
                    s, e = arm_state
                    arm_state_pub.put(to_zenoh_value(Vector2(x=s, y=e)))
                    
                    if step_count % 60 == 0:  # Log every 2 seconds at 30Hz
                        print(f"Driving: cmd=({reference_vx:.2f}, {reference_vy:.2f}, {reference_w:.2f}), "
                                f"actual=({actual_vx:.2f}, {actual_vy:.2f}, {actual_w:.2f})")
            
            except Exception as e:
                print(f"Error executing drive command: {e}")
        
            # Periodic status logging
            if step_count % 900 == 0:  # Every 30 seconds
                print(f"Controller running - step {step_count}, last_cmd: {time.monotonic() - last_recv:.1f}s ago")
            
            await asyncio.sleep(1.0 / 30.0)  # 30Hz control loop
            
    except KeyboardInterrupt:
        print("\nStopping drivetrain controller...")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Stop all motors
        if hardware and hardware.drive:
            try:
                await hardware.stop_all()
                print("Motors stopped")
            except Exception as e:
                print(f"Error stopping motors: {e}")
        
        # Close zenoh session
        session.close()
        print("Drivetrain controller stopped")

if __name__ == "__main__":
    asyncio.run(main())
