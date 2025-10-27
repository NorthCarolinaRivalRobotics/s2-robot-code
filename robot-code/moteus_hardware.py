#!/usr/bin/env python3

"""Shared Moteus hardware layer combining drivetrain, arm, and power telemetry.

Bus/ID mapping (per arm_code_idea.md with power-dist diagnostics):
- Motors 1,2 on bus 1
- Motor 6 on bus 2
- Power distribution board on bus 3 (id 32)
- Motors 5,3,4 on bus 4

Drivetrain motors: 1,2,3,4 (mecanum)
Arm motors: 5 (elbow), 6 (shoulder)
Power distribution: diagnostic stream at id 32
"""

from typing import Optional, Dict, Any

import time

import moteus
import moteus_pi3hat

from mecanum_drive import MecanumDrive
from arm_controller import ArmController


class MoteusHardware:
    def __init__(self, *, robot_id: str = "cash", power_id: int = 32) -> None:
        self.robot_id = robot_id
        self.power_id = power_id

        # Configure transport with desired servo bus map
        # Note: Ensure physical buses match this configuration on the Pi3Hat.
        self.servo_bus_map = {
            1: [1, 2],
            2: [6],
            3: [power_id],
            4: [5, 3, 4],
        }

        self.transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map=self.servo_bus_map)

        # Drivetrain (wheel order: [front_left, front_right, back_left, back_right])
        self.drive = MecanumDrive(motor_ids=[4, 1, 3, 2], transport=self.transport)

        # Arm
        self.arm = ArmController(shoulder_id=6, elbow_id=5, transport=self.transport)

        # Power distribution diagnostic stream (optional)
        self._power_controller: Optional[moteus.Controller] = None
        self._power_stream: Optional[moteus.Stream] = None
        self._power_fields = [
            "input_voltage_V",
            "output_voltage_V",
            "input_current_A",
            "output_current_A",
            "energy_uW_hr",
            "temperature_C",
            "uptime_s",
        ]
        self._init_power_stream()

    async def initialize(self) -> None:
        # Initialize drivetrain stops
        await self.drive.set_stops()
        # Initialize arm zero offsets
        await self.arm.initialize()

    async def stop_all(self) -> None:
        await self.drive.stop_all_motors()
        await self.arm.stop()

    # ------------------------------------------------------------------
    def _init_power_stream(self) -> None:
        try:
            self._power_controller = moteus.Controller(id=self.power_id, transport=self.transport)
            self._power_stream = moteus.Stream(self._power_controller)
        except Exception:
            self._power_controller = None
            self._power_stream = None

    async def read_power_diagnostics(self) -> Optional[Dict[str, Any]]:
        if self._power_stream is None:
            return None
        try:
            power = await self._power_stream.read_data("power")
        except Exception:
            # Attempt one-time reinitialization if the stream fails
            self._power_stream = None
            self._init_power_stream()
            return None
        if power is None:
            return None
        return self._power_to_dict(power)

    def _power_to_dict(self, power: Any) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"timestamp_s": time.time()}
        for field in self._power_fields:
            value = getattr(power, field, None)
            if value is None:
                continue
            try:
                payload[field] = float(value)
            except Exception:
                payload[field] = value

        voltage = payload.get("input_voltage_V") or payload.get("output_voltage_V")
        if isinstance(voltage, (int, float)):
            current_out = payload.get("output_current_A")
            current_in = payload.get("input_current_A")
            if isinstance(current_out, (int, float)):
                payload["output_power_W"] = float(voltage) * float(current_out)
            if isinstance(current_in, (int, float)):
                payload["input_power_W"] = float(voltage) * float(current_in)

        return payload

    @property
    def power_stream_ready(self) -> bool:
        return self._power_stream is not None
