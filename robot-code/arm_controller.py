#!/usr/bin/env python3

import math
import moteus
import moteus_pi3hat
from typing import Optional, Tuple


class ArmController:
    """
    Double-jointed arm controller (shoulder + elbow) using moteus position control.

    - Shoulder motor ID default: 6 (bus 2)
    - Elbow motor ID default: 5 (bus 4)
    - Gear reduction: 10:1 (motor:joint)

    Public API uses joint rotations (revolutions at the joint output).
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
        max_velocity_rps: float = 0.5,
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

        # Controllers
        self.servos = {
            shoulder_id: moteus.Controller(id=shoulder_id, transport=self.transport),
            elbow_id: moteus.Controller(id=elbow_id, transport=self.transport),
        }

        # # Zero offsets: motor revolutions to treat as joint=0.0
        # self._motor_zero = {shoulder_id: 0.0, elbow_id: 0.0}

        # Target positions in joint revolutions (persist last command)
        self._target_joint = {shoulder_id: 0.0, elbow_id: 0.0}

    async def initialize(self) -> None:
        """Query current motor positions and treat them as joint zero."""
        # Query both servos
        results = await self.transport.cycle(
            [self.servos[self.shoulder_id].make_stop(query=True),
             self.servos[self.elbow_id].make_stop(query=True)]
        )
        for r in results:
            rid = getattr(r, "id", None)
            if rid in self._motor_zero and moteus.Register.POSITION in r.values:
                self._motor_zero[rid] = float(r.values[moteus.Register.POSITION])
        print(f"DEBUG: Motor zero: {self._motor_zero}")

    def _joint_to_motor(self, joint_revs: float, motor_id: int) -> float:
        return joint_revs * self.gear_ratio

    def _motor_to_joint(self, motor_revs: float, motor_id: int) -> float:
        return (motor_revs) / self.gear_ratio

    async def set_targets(self, shoulder_joint_revs: float, elbow_joint_revs: float, *, query: bool = False) -> Tuple[float, float] | None:
        """Command both joints to target joint rotations (revs).

        If query=True, returns a tuple (shoulder_rev, elbow_rev) from the same cycle results.
        """
        self._target_joint[self.shoulder_id] = shoulder_joint_revs
        self._target_joint[self.elbow_id] = elbow_joint_revs

        cmds = []
        cmds.append(
            self.servos[self.shoulder_id].make_position(
                position=self._joint_to_motor(shoulder_joint_revs, self.shoulder_id),
                velocity=0.0,
                maximum_torque=self.max_torque_nm,
                velocity_limit=self.max_velocity_rps,
                query=query,
            )
        )
        cmds.append(
            self.servos[self.elbow_id].make_position(
                position=self._joint_to_motor(elbow_joint_revs, self.elbow_id),
                velocity=0.0,
                maximum_torque=self.max_torque_nm,
                velocity_limit=self.max_velocity_rps,
                query=query,
            )
        )
        results = await self.transport.cycle(cmds)
        if not query:
            return None
        # Parse joint positions from results
        joint_s = self._target_joint[self.shoulder_id]
        joint_e = self._target_joint[self.elbow_id]
        for r in results:
            if r.id == self.shoulder_id and hasattr(r, 'values') and r.values is not None:
                if moteus.Register.POSITION in r.values:
                    joint_s = self._motor_to_joint(float(r.values[moteus.Register.POSITION]), self.shoulder_id)
            elif r.id == self.elbow_id and hasattr(r, 'values') and r.values is not None:
                if moteus.Register.POSITION in r.values:
                    joint_e = self._motor_to_joint(float(r.values[moteus.Register.POSITION]), self.elbow_id)
        return (joint_s, joint_e)

    async def increment(self, d_shoulder: float, d_elbow: float) -> Tuple[float, float]:
        """Increment current targets by the provided deltas (joint revolutions)."""
        s = self._target_joint[self.shoulder_id] + d_shoulder
        e = self._target_joint[self.elbow_id] + d_elbow
        await self.set_targets(s, e)
        return (s, e)

    async def stop(self) -> None:
        await self.transport.cycle([self.servos[self.shoulder_id].make_stop(), self.servos[self.elbow_id].make_stop()])
