#!/usr/bin/env python3
"""
Wrist/Claw Node (placeholder)

Subscribes to simple wrist angle and claw open/close commands and publishes
corresponding state with a basic time-to-arrive estimate. Designed to be
swapped with real servo control later.
"""

import time
import logging
from dataclasses import dataclass

from tide.core.node import BaseNode
from tide.namespaces import robot_topic
from tide.models.serialization import to_zenoh_value


logger = logging.getLogger(__name__)


@dataclass
class WristState:
    angle: float = 0.0      # radians (placeholder units)
    claw_open: bool = False
    busy: bool = False
    eta_ts: float = 0.0     # wall-clock when we expect arrival


class WristNode(BaseNode):
    """
    Minimal wrist+claw node with time-based arrival estimate.
    Topics:
      - cmd/wrist/angle: float (radians)
      - cmd/wrist/claw: bool (True=open)

      - state/wrist/angle: float
      - state/wrist/claw: bool
      - state/wrist/arrived: bool
    """

    def __init__(self, *, config=None):
        super().__init__(config=config)

        self.robot_id = (config or {}).get("robot_id", "cash")
        self.update_rate = float((config or {}).get("update_rate", 30.0))
        # simple timing: 1.0 rad/s equivalent travel speed for arrival estimate
        self.travel_speed = float((config or {}).get("travel_speed", 1.0))
        self.hz = self.update_rate

        # Topics
        self.cmd_angle_topic = robot_topic(self.robot_id, "cmd/wrist/angle")
        self.cmd_claw_topic = robot_topic(self.robot_id, "cmd/wrist/claw")
        self.state_angle_topic = robot_topic(self.robot_id, "state/wrist/angle")
        self.state_claw_topic = robot_topic(self.robot_id, "state/wrist/claw")
        self.state_arrived_topic = robot_topic(self.robot_id, "state/wrist/arrived")

        # Subscribe to commands
        self.subscribe(self.cmd_angle_topic, self._on_cmd_angle)
        self.subscribe(self.cmd_claw_topic, self._on_cmd_claw)

        self._state = WristState()
        self._target_angle = 0.0

        logger.info(
            f"WristNode ready: cmd=({self.cmd_angle_topic}, {self.cmd_claw_topic})"
            f" state=({self.state_angle_topic}, {self.state_claw_topic}, {self.state_arrived_topic})"
        )

    def _on_cmd_angle(self, sample):
        try:
            target = float(sample)
        except Exception:
            logger.debug(f"Invalid wrist angle sample: {sample}")
            return
        # Estimate travel time
        now = time.time()
        distance = abs(target - self._state.angle)
        travel = distance / max(self.travel_speed, 1e-6)
        self._target_angle = target
        self._state.busy = True
        self._state.eta_ts = now + travel
        logger.info(f"Wrist angle cmd: {target:.3f} rad, eta {travel:.2f}s")

    def _on_cmd_claw(self, sample):
        try:
            # Accept bool or truthy string/number
            val = bool(sample if isinstance(sample, bool) else (float(sample) != 0.0))
        except Exception:
            logger.debug(f"Invalid claw sample: {sample}")
            return
        self._state.claw_open = val
        logger.info(f"Claw {'OPEN' if val else 'CLOSED'}")
        # Publish immediately
        try:
            self.put(self.state_claw_topic, to_zenoh_value(val))
        except Exception:
            pass

    def step(self):
        # Update arrival simulation and publish state periodically
        now = time.time()
        if self._state.busy and now >= self._state.eta_ts:
            self._state.busy = False
            self._state.angle = float(self._target_angle)
        # Publish angle and arrived status
        try:
            self.put(self.state_angle_topic, to_zenoh_value(float(self._state.angle)))
            self.put(self.state_arrived_topic, to_zenoh_value(not self._state.busy))
        except Exception:
            pass

    def cleanup(self):
        logger.info("WristNode shutting down")
        super().cleanup()

