#!/usr/bin/env python3
"""
Wrist/Claw Node (PCA9685 servos)

Controls two hobby position servos (wrist angle + claw open/close)
and a pair of intake ESCs via an Adafruit PCA9685 16-channel PWM driver.
Uses angle-to-pulse mapping for the wrist, discrete positions for the claw,
and symmetric -1..1 power mapping for the intakes.
"""

import time
import logging
from dataclasses import dataclass

import numpy as np
from tide.core.node import BaseNode
from tide.namespaces import robot_topic
from tide.models.serialization import to_zenoh_value


CLAW_MODE_CLOSED = "CLOSED"
CLAW_MODE_OPEN = "OPEN"
CLAW_MODE_SPEAR = "SPEAR"
_CLAW_MODES = {CLAW_MODE_CLOSED, CLAW_MODE_OPEN, CLAW_MODE_SPEAR}

_INDICATOR_COLOR_US = {
    "red": 1150.0,
    "orange": 1200.0,
    "yellow": 1300.0,
    "sage": 1400.0,
    "green": 1500.0,
    "azure": 1600.0,
    "blue": 1700.0,
    "indigo": 1800.0,
    "violet": 1900.0,
}


logger = logging.getLogger(__name__)

try:
    import Adafruit_PCA9685  # Matches usage in adafruit_Pca_example.py
except Exception:  # ImportError on dev machines without hardware
    Adafruit_PCA9685 = None  # type: ignore[assignment]


@dataclass
class WristState:
    angle: float = 0.0
    claw_open: bool = False
    claw_mode: str = CLAW_MODE_CLOSED
    busy: bool = False
    eta_ts: float = 0.0
    intake_power: float = 0.0
    indicator_color: str = ""


class WristNode(BaseNode):
    """
    Topics:
      - cmd/wrist/angle: float (radians)
      - cmd/wrist/claw: bool (True=open)
      - cmd/wrist/intake_speed: float (-1..1 power)

      - state/wrist/angle: float
      - state/wrist/claw: bool
      - state/wrist/arrived: bool
      - state/wrist/intake_power: float (-1..1 echo)
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
        self.intake_esc_left = int(cfg.get("intake_esc_left", 2))
        self.intake_esc_right = int(cfg.get("intake_esc_right", 3))
        self.indicator_channel = int(cfg.get("indicator_channel", 15))

        # Servo pulse bounds (PCA9685 tick counts, 0..4095). Typical ~150..600 at 60Hz.
        self.servo_min = int(cfg.get("servo_min", 150))
        self.servo_max = int(cfg.get("servo_max", 600))

        # ESC pulse mapping (three-point to support bidirectional brushed controllers)
        self.esc_min = int(cfg.get("intake_esc_min", 123))
        self.esc_max = int(cfg.get("intake_esc_max", 492))
        default_neutral = int(round((123 + 492) * 0.5))
        self.esc_neutral = int(cfg.get("intake_esc_neutral", default_neutral))

        # Wrist mechanical range in radians mapped linearly to [servo_min, servo_max]
        self.wrist_min_rad = float(cfg.get("wrist_min_rad", 0.0))
        self.wrist_max_rad = float(cfg.get("wrist_max_rad", np.pi))

        # Claw open/close positions as normalized [0..1] in the servo travel
        self.claw_open_norm = float(cfg.get("claw_open_norm", 0.75))
        self.claw_closed_norm = float(cfg.get("claw_closed_norm", 0.45))
        default_spear_norm = (self.claw_open_norm + self.claw_closed_norm) * 0.5
        self.claw_spear_norm = float(cfg.get("claw_spear_norm", default_spear_norm))

        # Topics
        self.cmd_angle_topic = robot_topic(self.robot_id, "cmd/wrist/angle")
        self.cmd_claw_topic = robot_topic(self.robot_id, "cmd/wrist/claw")
        self.cmd_intake_speed_topic = robot_topic(self.robot_id, "cmd/wrist/intake_speed")
        self.cmd_indicator_topic = robot_topic(self.robot_id, "cmd/wrist/light_color")
        self.state_angle_topic = robot_topic(self.robot_id, "state/wrist/angle")
        self.state_claw_topic = robot_topic(self.robot_id, "state/wrist/claw")
        self.state_arrived_topic = robot_topic(self.robot_id, "state/wrist/arrived")
        self.state_intake_power_topic = robot_topic(self.robot_id, "state/wrist/intake_power")

        # Subscribe to commands
        self.subscribe(self.cmd_angle_topic, self._on_cmd_angle)
        self.subscribe(self.cmd_claw_topic, self._on_cmd_claw)
        self.subscribe(self.cmd_intake_speed_topic, self._on_cmd_intake_speed)
        self.subscribe(self.cmd_indicator_topic, self._on_cmd_indicator_color)

        # Runtime state
        self._state = WristState()
        self._target_angle = 0.0
        self._last_intake_cmd: float | None = None
        self._last_indicator_pulse: int | None = None

        # Precompute indicator color PWM counts using servo controller frequency
        self._indicator_pwm: dict[str, int] = {}
        for name, micros in _INDICATOR_COLOR_US.items():
            self._indicator_pwm[name] = self._us_to_pwm_counts(micros)

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
            f"WristNode ready: cmd=({self.cmd_angle_topic}, {self.cmd_claw_topic}, {self.cmd_intake_speed_topic})"
            f" state=({self.state_angle_topic}, {self.state_claw_topic}, {self.state_arrived_topic}, {self.state_intake_power_topic})"
        )

        # arming sequence for the intakes
        self._apply_intake_power(0.0)
        time.sleep(1.0)
        self._apply_intake_power(1.0)
        time.sleep(1.0)
        self._apply_intake_power(0.0)
        time.sleep(1.0)

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

    def _us_to_pwm_counts(self, micros: float) -> int:
        freq = max(float(self.pwm_freq), 1e-6)
        counts = int(round(float(micros) * freq * 4096.0 / 1_000_000.0))
        return int(self._clamp(counts, 0, 4095))

    def _normalize_indicator_color(self, sample) -> str | None:
        try:
            if isinstance(sample, str):
                color = sample.strip().lower()
            elif isinstance(sample, (bytes, bytearray)):
                color = bytes(sample).decode().strip().lower()
            elif isinstance(sample, dict):
                for key in ("color", "value", "state"):
                    if key in sample and isinstance(sample[key], str):
                        nested = sample[key].strip().lower()
                        if nested in self._indicator_pwm:
                            return nested
                color = str(sample).strip().lower()
            else:
                color = str(sample).strip().lower()
        except Exception:
            return None

        if color in self._indicator_pwm:
            return color
        return None

    def _set_servo(self, channel: int, pulse: int):
        if self.pwm is None:
            return
        try:
            self.pwm.set_pwm(int(channel), 0, int(pulse))  # API per adafruit_Pca_example.py
        except Exception as e:
            logger.warning(f"Failed to set PWM on ch{channel}: {e}")

    def _power_to_pulse(self, power: float) -> int:
        clamped = max(-1.0, min(1.0, float(power)))
        hi_span = max(self.esc_max - self.esc_neutral, 1)
        lo_span = max(self.esc_neutral - self.esc_min, 1)
        if clamped >= 0.0:
            pulse = self.esc_neutral + int(round(clamped * hi_span))
        else:
            pulse = self.esc_neutral + int(round(clamped * lo_span))
        return int(self._clamp(pulse, self.esc_min, self.esc_max))

    def _apply_intake_power(self, power: float) -> None:
        pulse = self._power_to_pulse(power)
        self._set_servo(self.intake_esc_left, pulse)
        self._set_servo(self.intake_esc_right, pulse)
        logger.debug(f"Intake power {power:.2f} -> pulse {pulse}")

    def _set_indicator_color(self, color: str) -> None:
        pulse = self._indicator_pwm.get(color)
        if pulse is None:
            logger.debug(f"Indicator color '{color}' has no mapped pulse")
            return
        if self._last_indicator_pulse == pulse:
            return
        self._set_servo(self.indicator_channel, pulse)
        self._last_indicator_pulse = pulse
        logger.debug(f"Indicator color {color} -> pulse {pulse}")

    def _resolve_claw_mode(self, sample) -> str | None:
        try:
            if isinstance(sample, str):
                text = sample.strip().upper()
                if text in _CLAW_MODES:
                    return text
                if text in ("TRUE", "1"):
                    return CLAW_MODE_OPEN
                if text in ("FALSE", "0"):
                    return CLAW_MODE_CLOSED
            elif isinstance(sample, (bytes, bytearray)):
                return self._resolve_claw_mode(sample.decode())
            elif isinstance(sample, dict):
                for key in ("mode", "state"):
                    val = sample.get(key)
                    if isinstance(val, str):
                        resolved = self._resolve_claw_mode(val)
                        if resolved is not None:
                            return resolved
            if isinstance(sample, bool):
                return CLAW_MODE_OPEN if sample else CLAW_MODE_CLOSED
            numeric = float(sample)
            return CLAW_MODE_OPEN if numeric != 0.0 else CLAW_MODE_CLOSED
        except Exception:
            return None

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
        # logger.info(f"Wrist angle cmd: {target:.3f} rad (pulse={pulse}), eta {travel:.2f}s")

    def _on_cmd_claw(self, sample):
        mode = self._resolve_claw_mode(sample)
        if mode is None:
            logger.debug(f"Invalid claw sample: {sample}")
            return

        if mode == CLAW_MODE_SPEAR:
            self._state.claw_open = False
            self._state.claw_mode = CLAW_MODE_SPEAR
            norm = self.claw_spear_norm
            log_label = CLAW_MODE_SPEAR
        else:
            is_open = mode == CLAW_MODE_OPEN
            self._state.claw_open = is_open
            self._state.claw_mode = CLAW_MODE_OPEN if is_open else CLAW_MODE_CLOSED
            norm = self.claw_open_norm if is_open else self.claw_closed_norm
            log_label = CLAW_MODE_OPEN if is_open else CLAW_MODE_CLOSED

        pulse = self._norm_to_pulse(norm)
        self._set_servo(self.claw_channel, pulse)
        # logger.info(f"Claw {log_label} (pulse={pulse})")

        # Publish immediately (legacy bool state)
        try:
            self.put(self.state_claw_topic, to_zenoh_value(self._state.claw_open))
        except Exception:
            pass

    def _on_cmd_intake_speed(self, sample) -> None:
        try:
            logger.info(f"Intake speed sample: {sample}")
            power = float(sample)
        except Exception:
            logger.debug(f"Invalid intake speed sample: {sample}")
            return
        clamped = max(-1.0, min(1.0, power))
        if self._last_intake_cmd is not None and abs(clamped - self._last_intake_cmd) < 1e-3:
            return
        self._last_intake_cmd = clamped
        self._state.intake_power = clamped
        self._apply_intake_power(clamped)

    def _on_cmd_indicator_color(self, sample) -> None:
        color = self._normalize_indicator_color(sample)
        if color is None:
            logger.debug(f"Invalid indicator color sample: {sample}")
            return
        if self._state.indicator_color == color:
            return
        self._state.indicator_color = color
        self._set_indicator_color(color)

    # --- Node loop ---
    def step(self):
        now = time.time()
        if self._state.busy and now >= self._state.eta_ts:
            self._state.busy = False
            self._state.angle = float(self._target_angle)
        try:
            self.put(self.state_angle_topic, to_zenoh_value(float(self._state.angle)))
            self.put(self.state_arrived_topic, to_zenoh_value(not self._state.busy))
            self.put(self.state_intake_power_topic, to_zenoh_value(float(self._state.intake_power)))
        except Exception:
            pass

    def cleanup(self):
        logger.info("WristNode shutting down")
        # Leave servos at last commanded position; optionally move claw to closed
        super().cleanup()
