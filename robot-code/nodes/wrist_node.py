#!/usr/bin/env python3
"""
Wrist/Claw Node (PCA9685 servos)

Controls two hobby position servos (wrist angle + claw open/close)
via an Adafruit PCA9685 16-channel PWM driver. Uses angle-to-pulse
mapping for the wrist and two discrete positions for the claw.
"""

import time
import logging
from dataclasses import dataclass

import numpy as np
from tide.core.node import BaseNode
from tide.namespaces import robot_topic
from tide.models.serialization import to_zenoh_value


logger = logging.getLogger(__name__)

try:
    import Adafruit_PCA9685  # Matches usage in adafruit_Pca_example.py
except Exception:  # ImportError on dev machines without hardware
    Adafruit_PCA9685 = None  # type: ignore[assignment]


@dataclass
class WristState:
    angle: float = 0.0
    claw_open: bool = False
    busy: bool = False
    eta_ts: float = 0.0


class WristNode(BaseNode):
    """
    Topics:
      - cmd/wrist/angle: float (radians)
      - cmd/wrist/claw: bool (True=open)

      - state/wrist/angle: float
      - state/wrist/claw: bool
      - state/wrist/arrived: bool
    """

    def __init__(self, *, config=None):
        super().__init__(config=config)

        cfg = config or {}
        self.robot_id = cfg.get("robot_id", "cash")
        self.update_rate = float(cfg.get("update_rate", 30.0))
        self.travel_speed = float(cfg.get("travel_speed", 1.0))  # rad/s estimate
        self.hz = self.update_rate

        # PCA9685 configuration (matches example API)
        self.pca_addr = int(cfg.get("pca9685_address", 0x40))
        self.pca_busnum = int(cfg.get("i2c_busnum", 1))
        self.pwm_freq = int(cfg.get("pwm_freq", 60))

        # Channels
        self.wrist_channel = int(cfg.get("wrist_channel", 0))
        self.claw_channel = int(cfg.get("claw_channel", 1))

        # Servo pulse bounds (PCA9685 tick counts, 0..4095). Typical ~150..600 at 60Hz.
        self.servo_min = int(cfg.get("servo_min", 150))
        self.servo_max = int(cfg.get("servo_max", 600))

        # Wrist mechanical range in radians mapped linearly to [servo_min, servo_max]
        self.wrist_min_rad = float(cfg.get("wrist_min_rad", 0.0))
        self.wrist_max_rad = float(cfg.get("wrist_max_rad", np.pi))

        # Claw open/close positions as normalized [0..1] in the servo travel
        self.claw_open_norm = float(cfg.get("claw_open_norm", 0.600))
        self.claw_closed_norm = float(cfg.get("claw_closed_norm", 0.250))

        # Topics
        self.cmd_angle_topic = robot_topic(self.robot_id, "cmd/wrist/angle")
        self.cmd_claw_topic = robot_topic(self.robot_id, "cmd/wrist/claw")
        self.state_angle_topic = robot_topic(self.robot_id, "state/wrist/angle")
        self.state_claw_topic = robot_topic(self.robot_id, "state/wrist/claw")
        self.state_arrived_topic = robot_topic(self.robot_id, "state/wrist/arrived")

        # Subscribe to commands
        self.subscribe(self.cmd_angle_topic, self._on_cmd_angle)
        self.subscribe(self.cmd_claw_topic, self._on_cmd_claw)

        # Runtime state
        self._state = WristState()
        self._target_angle = 0.0

        # Hardware init
        self.pwm = None
        if Adafruit_PCA9685 is not None:
            try:
                self.pwm = Adafruit_PCA9685.PCA9685(address=self.pca_addr, busnum=self.pca_busnum)
                self.pwm.set_pwm_freq(self.pwm_freq)
                logger.info(f"PCA9685 ready addr=0x{self.pca_addr:02X} bus={self.pca_busnum} freq={self.pwm_freq}Hz")
                if self.servo_min > 1000 or self.servo_max > 1000:
                    logger.warning(
                        "servo_min/servo_max look high. Values are PCA9685 ticks (0..4095). "
                        "Typical hobby range is ~150..600 at 60Hz."
                    )
            except Exception as e:
                logger.warning(f"PCA9685 init failed; running without hardware control: {e}")
                self.pwm = None
        else:
            logger.warning("Adafruit_PCA9685 not available; running without hardware control")

        logger.info(
            f"WristNode ready: cmd=({self.cmd_angle_topic}, {self.cmd_claw_topic})"
            f" state=({self.state_angle_topic}, {self.state_claw_topic}, {self.state_arrived_topic})"
        )

    # --- Helpers ---
    def _clamp(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def _angle_to_pulse(self, angle_rad: float) -> int:
        a = self._clamp(angle_rad, self.wrist_min_rad, self.wrist_max_rad)
        span_rad = max(self.wrist_max_rad - self.wrist_min_rad, 1e-6)
        t = (a - self.wrist_min_rad) / span_rad  # 0..1
        pulse = int(round(self.servo_min + t * (self.servo_max - self.servo_min)))
        return int(self._clamp(pulse, self.servo_min, self.servo_max))

    def _norm_to_pulse(self, n: float) -> int:
        t = self._clamp(n, 0.0, 1.0)
        pulse = int(round(self.servo_min + t * (self.servo_max - self.servo_min)))
        return int(self._clamp(pulse, self.servo_min, self.servo_max))

    def _set_servo(self, channel: int, pulse: int):
        if self.pwm is None:
            return
        try:
            self.pwm.set_pwm(int(channel), 0, int(pulse))  # API per adafruit_Pca_example.py
        except Exception as e:
            logger.warning(f"Failed to set PWM on ch{channel}: {e}")

    # --- Handlers ---
    def _on_cmd_angle(self, sample):
        try:
            target = float(sample)
        except Exception:
            logger.debug(f"Invalid wrist angle sample: {sample}")
            return

        # Send to hardware immediately (position servo)
        pulse = self._angle_to_pulse(target)
        self._set_servo(self.wrist_channel, pulse)

        # Estimate travel time and publish progress
        now = time.time()
        distance = abs(target - self._state.angle)
        travel = distance / max(self.travel_speed, 1e-6)
        self._target_angle = float(self._clamp(target, self.wrist_min_rad, self.wrist_max_rad))
        self._state.busy = True
        self._state.eta_ts = now + travel
        logger.info(f"Wrist angle cmd: {target:.3f} rad (pulse={pulse}), eta {travel:.2f}s")

    def _on_cmd_claw(self, sample):
        try:
            val = bool(sample if isinstance(sample, bool) else (float(sample) != 0.0))
        except Exception:
            logger.debug(f"Invalid claw sample: {sample}")
            return

        self._state.claw_open = val
        norm = self.claw_open_norm if val else self.claw_closed_norm
        pulse = self._norm_to_pulse(norm)
        self._set_servo(self.claw_channel, pulse)
        logger.info(f"Claw {'OPEN' if val else 'CLOSED'} (pulse={pulse})")

        # Publish immediately
        try:
            self.put(self.state_claw_topic, to_zenoh_value(val))
        except Exception:
            pass

    # --- Node loop ---
    def step(self):
        now = time.time()
        if self._state.busy and now >= self._state.eta_ts:
            self._state.busy = False
            self._state.angle = float(self._target_angle)
        try:
            self.put(self.state_angle_topic, to_zenoh_value(float(self._state.angle)))
            self.put(self.state_arrived_topic, to_zenoh_value(not self._state.busy))
        except Exception:
            pass

    def cleanup(self):
        logger.info("WristNode shutting down")
        # Leave servos at last commanded position; optionally move claw to closed
        super().cleanup()
