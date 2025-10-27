#!/usr/bin/env python3
"""
Simple async test script for a double-jointed arm (shoulder + elbow) using moteus.

- Shoulder motor ID: 6 (bus 2)
- Elbow motor ID:    5 (bus 4)

Moves each joint slowly back and forth with conservative limits, printing positions.
This uses relative moves from the current positions (no persistent rezero).
"""

import asyncio
import math
import time

import numpy as np

import moteus
import moteus_pi3hat


# Port/bus mapping from arm_code_idea.md
SHOULDER_ID = 6  # bus 2
ELBOW_ID = 5     # bus 4

# Basic joint parameters
GEAR_REDUCTION = 10.0  # motor revs per 1 joint rev

# Conservative limits (per arm_code_idea.md)
MAX_TORQUE_NM = 1.0
MAX_VELOCITY_RPS = np.nan  # motor rev/s

# Motion parameters
JOINT_DELTA_REV = -0.25  # move ~18 degrees at the joint
JOINT_DELTA_REV_2 = -0.5
DWELL_S = 0.5
CYCLES = 3


def motor_to_joint_revs(motor_revs: float, zero_offset: float) -> float:
    return (motor_revs - zero_offset) / GEAR_REDUCTION


async def main():
    # Configure the Pi3Hat transport with only the arm motors
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map={
            2: [SHOULDER_ID],
            4: [ELBOW_ID],
        }
    )

    # Create controllers
    shoulder = moteus.Controller(id=SHOULDER_ID, transport=transport)
    elbow = moteus.Controller(id=ELBOW_ID, transport=transport)

    # Query current positions and treat them as zero references (in code)
    results = await transport.cycle([
        shoulder.make_stop(query=True),
        elbow.make_stop(query=True),
    ])

    zero = {SHOULDER_ID: 0.0, ELBOW_ID: 0.0}
    for r in results:
        if getattr(r, "id", None) in zero and moteus.Register.POSITION in r.values:
            zero[r.id] = float(r.values[moteus.Register.POSITION])

    print("Starting arm test with conservative limits…")
    print(f" - Shoulder ID {SHOULDER_ID} on bus 2")
    print(f" - Elbow    ID {ELBOW_ID} on bus 4")
    print(f" - Max torque {MAX_TORQUE_NM} Nm, max velocity {MAX_VELOCITY_RPS} rps (motor)")

    # Helper to command absolute motor target from a desired joint delta
    def joint_target_to_motor_abs(joint_revs_target: float, motor_id: int) -> float:
        return zero[motor_id] + joint_revs_target * GEAR_REDUCTION

    # Simple back-and-forth pattern for both joints
    pattern = [0.0, +JOINT_DELTA_REV, 0.0, +JOINT_DELTA_REV_2, 0.0]

    try:
        for cycle in range(CYCLES):
            print(f"\nCycle {cycle + 1}/{CYCLES}")
            for joint_target in pattern:
                # Command both joints to the same relative target for simplicity
                cmds = [
                    shoulder.make_position(
                        position=joint_target_to_motor_abs(joint_target, SHOULDER_ID),
                        velocity=0.0,
                        maximum_torque=MAX_TORQUE_NM,
                        velocity_limit=MAX_VELOCITY_RPS,
                        accel_limit=np.nan,
                        query=True,
                    ),
                    # elbow.make_position(
                    #     position=joint_target_to_motor_abs(joint_target, ELBOW_ID),
                    #     velocity=0.0,
                    #     maximum_torque=MAX_TORQUE_NM,
                    #     velocity_limit=MAX_VELOCITY_RPS,
                    #     accel_limit=2.0,
                    #     query=True,
                    # ),
                ]

                # Continuously send commands for the dwell duration
                end_time = time.time() + DWELL_S
                while time.time() < end_time:
                    res = await transport.cycle(cmds)

                    # Extract and print joint positions
                    s_joint = None
                    e_joint = None
                    for rr in res:
                        if moteus.Register.POSITION not in rr.values:
                            continue
                        if rr.id == SHOULDER_ID:
                            s_joint = motor_to_joint_revs(float(rr.values[moteus.Register.POSITION]), zero[SHOULDER_ID])
                        elif rr.id == ELBOW_ID:
                            e_joint = motor_to_joint_revs(float(rr.values[moteus.Register.POSITION]), zero[ELBOW_ID])

                    if s_joint is not None and e_joint is not None:
                        print(
                            f"Target joint: {joint_target:+.3f} rev  |  "
                            f"Shoulder: {s_joint:+.3f} rev ({s_joint*360:+6.1f} deg)  "
                            f"Elbow: {e_joint:+.3f} rev ({e_joint*360:+6.1f} deg)"
                        )

        print("\nArm test complete.")

    except KeyboardInterrupt:
        print("\nInterrupted; stopping servos…")
    finally:
        # Stop both servos
        await transport.cycle([shoulder.make_stop(), elbow.make_stop()])


if __name__ == "__main__":
    asyncio.run(main())
