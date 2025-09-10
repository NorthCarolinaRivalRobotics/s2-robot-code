#!/usr/bin/env python3

import math
import moteus
import moteus_pi3hat
from typing import Optional, Tuple
import numpy as np


class ArmController:
    """
    Double-jointed arm controller (shoulder + elbow) using moteus position control.

    - Shoulder motor ID default: 6 (bus 2)
    - Elbow motor ID default: 5 (bus 4)
    - Gear reduction: 10:1 (motor:joint)

    Public API uses joint angles in radians (at the joint output).
    Internally converts to motor revolutions using gear_ratio.
    """

    def __init__(
        self,
        *,
        shoulder_id: int = 6,
        elbow_id: int = 5,
        transport: Optional[moteus_pi3hat.Pi3HatRouter] = None,
        servo_bus_map: Optional[dict] = None,
        gear_ratio: float = 10.0,
        max_torque_nm: float = 1.0,
        max_velocity_rps: float = 60.0,
        max_acceleration_rps2: float = 70.0,
    ) -> None:
        # Transport can be shared; create if not provided
        if transport is not None:
            self.transport = transport
        else:
            raise ValueError("Transport must be provided")

        self.shoulder_id = shoulder_id
        self.elbow_id = elbow_id
        self.gear_ratio = gear_ratio
        self.max_torque_nm = max_torque_nm
        self.max_velocity_rps = max_velocity_rps
        self.max_acceleration_rps2 = max_acceleration_rps2
        # Controllers
        self.servos = {
            shoulder_id: moteus.Controller(id=shoulder_id, transport=self.transport),
            elbow_id: moteus.Controller(id=elbow_id, transport=self.transport),
        }

        # Zero offsets: motor revolutions for angle mapping
        self._motor_zero = {shoulder_id: 0.0, elbow_id: 0.0}

        # Target joint angles (radians) - persist last command
        self._target_angle = {shoulder_id: 0.0, elbow_id: 0.0}

    async def initialize(self) -> None:
        """Query current motor positions and set mapping so current joints read +pi/2 rad.

        This assumes both arm segments are started vertical (up), corresponding to +pi/2 radians.
        """
        # Query both servos
        results = await self.transport.cycle(
            [self.servos[self.shoulder_id].make_stop(query=True),
             self.servos[self.elbow_id].make_stop(query=True)]
        )
        for r in results:
            rid = getattr(r, "id", None)
            if rid in self._motor_zero and moteus.Register.POSITION in r.values:
                mpos = float(r.values[moteus.Register.POSITION])
                # Choose motor_zero so that current motor position maps to +pi/2 at the joint
                # angle = ((mpos - motor_zero)/gear_ratio) * 2*pi = +pi/2
                # => motor_zero = mpos - gear_ratio/4
                self._motor_zero[rid] = mpos - (self.gear_ratio / 4.0)
        
    def _angle_to_motor(self, angle_rad: float, motor_id: int) -> float:
        # Convert joint angle (rad) to motor revolutions
        joint_revs = angle_rad / (2.0 * math.pi)
        return self._motor_zero[motor_id] + joint_revs * self.gear_ratio

    def _motor_to_angle(self, motor_revs: float, motor_id: int) -> float:
        # Convert motor revolutions to joint angle (rad)
        joint_revs = (motor_revs - self._motor_zero[motor_id]) / self.gear_ratio
        return joint_revs * (2.0 * math.pi)

    async def set_targets(
        self,
        shoulder_angle_rad: float,
        elbow_angle_rad: float,
        *,
        end_velocity_frac: float | None = None,
        query: bool = False,
    ) -> Tuple[float, float] | None:
        """Command both joints to target joint angles (radians).

        If query=True, returns a tuple (shoulder_angle_rad, elbow_angle_rad) from the same cycle results.
        """
        self._target_angle[self.shoulder_id] = shoulder_angle_rad
        self._target_angle[self.elbow_id] = elbow_angle_rad

        # Determine desired endpoint velocity for the trajectory in motor rev/s.
        # If unspecified, default to 0.0 for a stop at the target.
        if end_velocity_frac is None:
            endpoint_vel_rps = 0.0
        else:
            # Clamp to [0, 1] and scale by configured max velocity.
            f = float(end_velocity_frac)
            endpoint_vel_rps = f * float(self.max_velocity_rps)

        cmds = []
        cmds.append(
            self.servos[self.shoulder_id].make_position(
                position=self._angle_to_motor(shoulder_angle_rad, self.shoulder_id),
                velocity=-endpoint_vel_rps,
                maximum_torque=self.max_torque_nm,
                velocity_limit=np.nan,
                accel_limit=self.max_acceleration_rps2,
                query=query,
            )
        )
        cmds.append(
            self.servos[self.elbow_id].make_position(
                position=self._angle_to_motor(elbow_angle_rad, self.elbow_id),
                velocity=endpoint_vel_rps,
                maximum_torque=self.max_torque_nm,
                velocity_limit=np.nan,
                accel_limit=self.max_acceleration_rps2,
                query=query,
            )
        )
        results = await self.transport.cycle(cmds)
        if not query:
            return None
        # Parse joint positions from results
        joint_s = self._target_angle[self.shoulder_id]
        joint_e = self._target_angle[self.elbow_id]
        for r in results:
            if r.id == self.shoulder_id and hasattr(r, 'values') and r.values is not None:
                if moteus.Register.POSITION in r.values:
                    joint_s = self._motor_to_angle(float(r.values[moteus.Register.POSITION]), self.shoulder_id)
            elif r.id == self.elbow_id and hasattr(r, 'values') and r.values is not None:
                if moteus.Register.POSITION in r.values:
                    joint_e = self._motor_to_angle(float(r.values[moteus.Register.POSITION]), self.elbow_id)
        return (joint_s, joint_e)

    async def increment(self, d_shoulder: float, d_elbow: float) -> Tuple[float, float]:
        """Increment current targets by the provided deltas (radians)."""
        s = self._target_angle[self.shoulder_id] + d_shoulder
        e = self._target_angle[self.elbow_id] + d_elbow
        await self.set_targets(s, e)
        return (s, e)

    async def stop(self) -> None:
        await self.transport.cycle([self.servos[self.shoulder_id].make_stop(), self.servos[self.elbow_id].make_stop()])
