#!/usr/bin/python3 -B

import asyncio
import math
import json
import time

import numpy as np
import moteus
import moteus_pi3hat

from kinematics import measured_positions_to_module_angles, robot_relative_velocity_to_twist, twist_to_wheel_speeds, WheelSpeeds, ModuleAngles, wheel_speeds_to_twist
from geometry2d import Transform2d, Twist2d, Twist2dVelocity

import zenoh

from utils import angle_wrap

AZIMUTH_RATIO = 12.0 / 75.0
DRIVE_REDUCTION = (25.0/21.0) * (15.0/45.0)
DRIVE_DIAMETER = 0.075  # 75 mm
DRIVE_CIRCUMFERENCE = DRIVE_DIAMETER * math.pi
WATCHDOG_TIMEOUT = 0.5

# GLOBAL STATE
last_recv = time.monotonic()
reference_vx = 0.0  # m/s
reference_vy = 0.0  # m/s
reference_w = 0.0   # rad/s
offset = 0.0        # rad
is_initial_angle = True
reference_heading = 0.0
gain = 0.1
angular_velocity_constant = 0.0
yaw_bias_integral = 0.0

# The Zenoh keys we will subscribe to:
VELOCITY_KEY = "robot/control/velocity"
ZERO_HEADING_KEY = "robot/control/zero_heading"
MEASURED_TWIST_KEY = "robot/observed/twist"
ODOMETRY_KEY = "robot/odom"
WHEEL_VELOCITIES_KEY = "robot/observed/wheel_velocities"
MODULE_ANGLES_KEY = "robot/observed/module_angles"





def calculate_swerve_angle(position: float) -> float:
    return angle_wrap(position * 2 * math.pi * AZIMUTH_RATIO)

def wheel_speed_to_motor_speed(wheel_speed: float) -> float:
    return wheel_speed / (DRIVE_CIRCUMFERENCE * DRIVE_REDUCTION)

def calculate_target_position_delta(reference_azimuth_angle, estimated_angle):
    angle_difference = angle_wrap(reference_azimuth_angle - estimated_angle)
    return angle_difference / (2 * math.pi * AZIMUTH_RATIO)

def zenoh_velocity_listener(sample):
    # This callback is invoked when new velocity data arrives.
    # Payload is assumed JSON with fields vx, vy, omega (deg/s)
    global reference_vx, reference_vy, reference_w, last_recv
    data_str = sample.payload.to_string()
    try:
        data = json.loads(data_str)
        # vx, vy in m/s, omega in deg/s as per original code
        # We'll store omega in radians as before.
        reference_vx = data["vx"]
        reference_vy = data["vy"]
        reference_w = math.radians(data["omega"])
        last_recv = time.monotonic()
    except Exception as e:
        print(f"Failed to parse velocity command: {e}")

def zenoh_zero_heading_listener(sample):
    # This callback is invoked when zero-heading command arrives.
    # Payload might be ignored if unnecessary. Just zero the heading.
    global is_initial_angle
    is_initial_angle = True

def motor_speed_to_wheel_speed(motor_speed: float, drive_direction: float) -> float:
    """
    Convert motor speed (rev/s) to wheel speed (m/s).
    drive_direction is 1 or -1 based on the drive_directions map.
    """
    return motor_speed * DRIVE_CIRCUMFERENCE * DRIVE_REDUCTION * drive_direction

async def main():
    global reference_vx, reference_vy, reference_w, offset, is_initial_angle, reference_heading
    global yaw_bias_integral, angular_velocity_constant
    pose = Transform2d(0.0, 0.0, 0.0)
    # Start Zenoh session and subscribe
    z_conf = zenoh.Config()
    session = zenoh.open(z_conf)
    _ = session.declare_subscriber(VELOCITY_KEY, zenoh_velocity_listener)
    _ = session.declare_subscriber(ZERO_HEADING_KEY, zenoh_zero_heading_listener)

    measured_twist_pub = session.declare_publisher(MEASURED_TWIST_KEY)
    odom_pub = session.declare_publisher(ODOMETRY_KEY)
    wheel_velocities_pub = session.declare_publisher(WHEEL_VELOCITIES_KEY)
    module_angles_pub = session.declare_publisher(MODULE_ANGLES_KEY)

    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={1: [1, 2, 4], 2: [5, 6], 3: [3, 7, 8]},
    )

    azimuth_ids = [2, 4, 6, 8]
    drive_ids = [1, 3, 5, 7]

    servos = {servo_id: moteus.Controller(id=servo_id, transport=transport) for servo_id in azimuth_ids + drive_ids}
    drive_directions = {1: 1, 3: 1, 5: -1, 7: -1}

    results = await transport.cycle([x.make_stop(query=True) for x in servos.values()])
    initial_module_positions = {
        result.id: result.values[moteus.Register.POSITION] for result in results if result.id in azimuth_ids
    }

    offset = 0.0
    yaw = 0.0
    measured_module_positions = {2: 0.0, 4: 0.0, 6: 0.0, 8: 0.0}
    module_scaling = {2: 1.0, 4: 1.0, 6: 1.0, 8: 1.0}
    module_inversions = {2: False, 4: False, 6: False, 8: False}

    # Store previous motor velocities
    previous_motor_velocities = {1: 0.0, 3: 0.0, 5: 0.0, 7: 0.0}

    loop_start = time.monotonic()
    dt = 0.005

    try:
        while True:
            dt = time.monotonic() - loop_start
            loop_start = time.monotonic()

            # Watchdog
            if loop_start - last_recv > WATCHDOG_TIMEOUT:
                reference_vx = 0.0
                reference_vy = 0.0
                reference_w = 0.0

            # Calculate a reference twist to drive the wheels
            reference_heading = reference_heading + reference_w * dt
            heading_error = angle_wrap(reference_heading - -yaw)
            heading_gain = 0.0
            if np.abs(reference_w) > 0.1:
                reference_heading = -yaw
                heading_gain = 0.0
            reference = Twist2dVelocity(reference_vx, reference_vy, reference_w + heading_error * heading_gain)

            # Convert reference twist to wheel speeds + angles
            wheel_speeds, module_angles = robot_relative_velocity_to_twist(reference, dt, -(yaw + np.pi/2))

            # Build servo commands
            commands = []
            # 1) Azimuth commands
            for servo_id in azimuth_ids:
                current_angle = calculate_swerve_angle(measured_module_positions[servo_id]) - \
                                calculate_swerve_angle(initial_module_positions[servo_id])
                current_angle = angle_wrap(current_angle)

                target_angle = -angle_wrap(module_angles.from_id(servo_id))
                error = angle_wrap(target_angle - current_angle)
                module_scaling[servo_id] = np.cos(np.clip(error, -np.pi/2, np.pi/2))

                if abs(error) > np.pi / 2:
                    module_inversions[servo_id] = not module_inversions[servo_id]

                target_position_delta = calculate_target_position_delta(target_angle, current_angle)
                commands.append(servos[servo_id].make_position(
                    position=measured_module_positions[servo_id] + target_position_delta,
                    velocity=0.0,
                    maximum_torque=1.7,
                    velocity_limit=230.0,
                    accel_limit=120.0,
                    query=True,
                ))

            # 2) Drive commands
            for servo_id in drive_ids:
                commands.append(servos[servo_id].make_position(
                    position=math.nan,
                    velocity=module_scaling[servo_id + 1] *
                              wheel_speed_to_motor_speed(wheel_speeds.from_id(servo_id)) *
                              drive_directions[servo_id],
                    maximum_torque=1.0 * 0.25,
                    query=True,
                ))

            # Cycle to get servo results + IMU
            results = await transport.cycle(commands, request_attitude=True)
            imu_result = [x for x in results if x.id == -1 and isinstance(x, moteus_pi3hat.CanAttitudeWrapper)][0]

            # Gyro-based yaw offset logic
            if is_initial_angle:
                offset = imu_result.euler_rad.yaw
                yaw_bias_integral = 0.0
                angular_velocity_constant = np.deg2rad(imu_result.rate_dps.z)
                pose = Transform2d(0.0, 0.0, 0.0)
                is_initial_angle = False

            yaw_bias_integral += angular_velocity_constant * dt
            yaw = angle_wrap(imu_result.euler_rad.yaw - (offset - yaw_bias_integral))

            # Update measured positions
            measured_module_positions = {
                r.id: r.values[moteus.Register.POSITION] for r in results if r.id in azimuth_ids
            }
            # Save measured motor velocities
            measured_wheel_speeds_map = {
                r.id: r.values[moteus.Register.VELOCITY] for r in results if r.id in drive_ids
            }

            # Build a measured angles object from the actual module azimuth
            module_angles = measured_positions_to_module_angles(measured_module_positions, initial_module_positions)

            # Build dictionary of measured wheel speeds (in m/s)
            actual_wheel_speeds_dict = {}
            for drive_id in drive_ids:
                if drive_id not in measured_wheel_speeds_map:
                    print(f"NOT FOUND; SKIPPING servo: {drive_id}")
                    motor_speed = previous_motor_velocities[drive_id]
                else:
                    motor_speed = measured_wheel_speeds_map[drive_id]
                    previous_motor_velocities[drive_id] = motor_speed

                actual_wheel_speeds_dict[drive_id] = motor_speed_to_wheel_speed(
                    motor_speed, drive_directions[drive_id]
                )

            # Construct a WheelSpeeds object with actual measured wheel speeds
            measured_wheel_speeds_struct = WheelSpeeds(
                front_left=actual_wheel_speeds_dict[1],
                front_right=actual_wheel_speeds_dict[3],
                back_left=actual_wheel_speeds_dict[5],
                back_right=actual_wheel_speeds_dict[7],
            )

            # Compute linear components from wheels, but override angular with IMU
            raw_twist = wheel_speeds_to_twist(measured_wheel_speeds_struct, module_angles, dt)

            # Overwrite raw_twist.w with the gyro-based angular velocity (minus drift offset).
            # We can read imu_result.rate_dps.z, convert to rad/s, and then
            # subtract (or add) any offset as needed:
            # Ex: actual_wz = np.deg2rad(imu_result.rate_dps.z) - drift_term
            actual_wz = np.deg2rad(imu_result.rate_dps.z)  # or adjust if needed
            actual_twist = Twist2dVelocity(raw_twist.vx, -raw_twist.vy, actual_wz)

            print("Actual Twist (gyro-based ω):", actual_twist.vx, actual_twist.vy, actual_twist.w)

            # Publish measured twist
            measured_twist_msg = json.dumps({
                "vx": actual_twist.vx,
                "vy": actual_twist.vy,
                "omega": actual_twist.w
            })
            measured_twist_pub.put(measured_twist_msg)

            twist = Twist2d.from_twist2dvelocity(actual_twist, dt)
            # the most beautiful operation in all of robotics.
            pose = pose * twist.exp()
            pose.theta = -yaw # set to best estimate of yaw.

            # Publish odometry instead
            odom_msg = json.dumps({
                "x": pose.x,
                "y": pose.y,
                "theta": pose.theta
            })
            odom_pub.put(odom_msg)

            wheel_velocities_msg = json.dumps({
                "front_left": actual_wheel_speeds_dict[1],
                "front_right": actual_wheel_speeds_dict[3],
                "back_left": actual_wheel_speeds_dict[5],
                "back_right": actual_wheel_speeds_dict[7]
            })
            wheel_velocities_pub.put(wheel_velocities_msg)

            module_angles_msg = json.dumps({
                "front_left": module_angles.front_left_angle,
                "front_right": module_angles.front_right_angle,
                "back_left": module_angles.back_left_angle,
                "back_right": module_angles.back_right_angle
            })
            module_angles_pub.put(module_angles_msg)


            await asyncio.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopping all servos...")
        await transport.cycle([x.make_stop() for x in servos.values()])
    finally:
        session.close()

if __name__ == "__main__":
    asyncio.run(main())