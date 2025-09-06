#!/usr/bin/env python3

"""
Shared Moteus hardware layer combining drivetrain and arm on a single transport.

Bus/ID mapping (per arm_code_idea.md):
- Motors 1,2 on bus 1
- Motor 6 on bus 2
- Motors 5,3,4 on bus 4

Drivetrain motors: 1,2,3,4 (mecanum)
Arm motors: 5 (elbow), 6 (shoulder)
"""

from typing import Optional, Dict, Any, Tuple

import moteus_pi3hat

from mecanum_drive import MecanumDrive
from arm_controller import ArmController


class MoteusHardware:
    def __init__(self, *, robot_id: str = "cash") -> None:
        self.robot_id = robot_id

        # Configure transport with desired servo bus map
        # Note: Ensure physical buses match this configuration on the Pi3Hat.
        self.servo_bus_map = {
            1: [1, 2],
            2: [6],
            4: [5, 3, 4],
        }

        self.transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=self.servo_bus_map)

        # Drivetrain (wheel order: [front_left, front_right, back_left, back_right])
        self.drive = MecanumDrive(motor_ids=[4, 1, 3, 2], transport=self.transport)

        # Arm
        self.arm = ArmController(shoulder_id=6, elbow_id=5, transport=self.transport)

    async def initialize(self) -> None:
        # Initialize drivetrain stops
        await self.drive.set_stops()
        # Initialize arm zero offsets
        await self.arm.initialize()

    async def stop_all(self) -> None:
        await self.drive.stop_all_motors()
        await self.arm.stop()

