#!/usr/bin/env python3
"""Battery alarm node that speaks alerts for power and match timing.

The node mirrors the power telemetry subscription used by the Rerun
visualization node so it can tap into the same Tide topics. When telemetry
indicates the robot battery is at or below a configurable threshold, or the
current draw rises above a configured value, the node uses macOS's `say`
utility to warn the driver station operator. It also monitors the teleop
command stream and starts a configurable match timer the first time the
driver requests more than a threshold of linear speed, issuing periodic
time-remaining callouts and endgame reminders.
"""

from __future__ import annotations

import logging
import math
import queue
import shutil
import subprocess
import threading
import time
from typing import Mapping, Optional

from tide.core.node import BaseNode
from tide.namespaces import robot_topic


logger = logging.getLogger(__name__)


class BatteryAlarmNode(BaseNode):
    """Speaks audible warnings for low battery voltage and high current draw."""

    def __init__(self, *, config=None):
        super().__init__(config=config)

        # Defaults mirror the driver station configuration.
        self.robot_id = "cash"
        self.update_rate = 5.0  # Hz
        self.power_topic = robot_topic(self.robot_id, "state/power/dist")
        self.teleop_topic = robot_topic(self.robot_id, "cmd/teleop")

        self.low_voltage_threshold = 21.0  # Volts
        self.high_current_threshold = 15.0  # Amps
        self.alert_cooldown_s = 30.0  # Minimum seconds between repeated alerts
        self.voltage_clear_hysteresis = 0.5  # Volts required to reset alert state
        self.current_clear_hysteresis = 2.0  # Amps required to reset alert state

        self.match_timer_enabled = True
        self.match_duration_s = 150.0  # 2 minutes 30 seconds
        self.match_announce_interval_s = 10.0
        self.match_completion_message = "Match timer complete."
        self.match_special_callouts: list[tuple[int, str]] = [
            (25, "Ensure all available nukes are scored."),
            (10, "Go balance."),
        ]
        self.match_timer_tolerance_s = 0.05
        self.motion_speed_threshold = 0.1

        if config:
            self.robot_id = config.get("robot_id", self.robot_id)
            self.update_rate = float(config.get("update_rate", self.update_rate))
            self.power_topic = config.get("power_topic", robot_topic(self.robot_id, "state/power/dist"))
            self.teleop_topic = config.get("teleop_topic", robot_topic(self.robot_id, "cmd/teleop"))
            self.low_voltage_threshold = float(config.get("low_voltage_threshold", self.low_voltage_threshold))
            self.high_current_threshold = float(config.get("high_current_threshold", self.high_current_threshold))
            self.alert_cooldown_s = float(config.get("alert_cooldown_s", self.alert_cooldown_s))
            self.voltage_clear_hysteresis = float(
                config.get("voltage_clear_hysteresis", self.voltage_clear_hysteresis)
            )
            self.current_clear_hysteresis = float(
                config.get("current_clear_hysteresis", self.current_clear_hysteresis)
            )
            self.match_timer_enabled = bool(config.get("match_timer_enabled", self.match_timer_enabled))
            self.match_duration_s = float(config.get("match_duration_s", self.match_duration_s))
            self.match_announce_interval_s = float(
                config.get("match_announce_interval_s", self.match_announce_interval_s)
            )
            self.motion_speed_threshold = float(
                config.get("motion_speed_threshold", self.motion_speed_threshold)
            )
            self.match_completion_message = str(
                config.get("match_completion_message", self.match_completion_message)
            )
            raw_special = config.get("match_special_callouts")
            if isinstance(raw_special, Mapping):
                parsed: list[tuple[int, str]] = []
                for key, message in raw_special.items():
                    try:
                        seconds = int(round(float(key)))
                    except (TypeError, ValueError):
                        continue
                    if not isinstance(message, str):
                        try:
                            message = str(message)
                        except Exception:
                            continue
                    parsed.append((seconds, message))
                if parsed:
                    parsed.sort(reverse=True)
                    self.match_special_callouts = parsed

        self.hz = self.update_rate

        # Track the latest telemetry and alert timings.
        self._last_voltage: Optional[float] = None
        self._last_current: Optional[float] = None
        self._last_sample_time: Optional[float] = None

        self._last_voltage_alert_time: Optional[float] = None
        self._last_current_alert_time: Optional[float] = None
        self._voltage_in_alert = False
        self._current_in_alert = False

        # Match timer tracking
        self._match_started = False
        self._match_running = False
        self._match_start_time: Optional[float] = None
        self._match_end_time: Optional[float] = None
        self._time_offset_seconds = 2.3 # 2 seconds to account for teleop lag
        self._next_interval_remaining: Optional[float] = None
        self._special_callouts_fired: dict[int, bool] = {sec: False for sec, _ in self.match_special_callouts}
        self._match_completion_announced = False
        self._countdown_spoken: set[int] = set()

        # Determine if the macOS `say` command is available once up front.
        self._say_command = shutil.which("say")
        if not self._say_command:
            logger.warning("macOS 'say' command not found; battery alarm will not speak alerts")
        self._speech_queue: queue.Queue[str] = queue.Queue()
        self._speech_thread: Optional[threading.Thread] = None
        if self._say_command:
            self._speech_thread = threading.Thread(target=self._speech_worker, daemon=True)
            self._speech_thread.start()

        # Subscribe to power telemetry updates.
        self.subscribe(self.power_topic, self._on_power_sample)
        if self.match_timer_enabled:
            self.subscribe(self.teleop_topic, self._on_teleop_command)

        logger.info(
            "BatteryAlarmNode initialized for %s: power_topic=%s, voltage<=%.1fV, current>=%.1fA",
            self.robot_id,
            self.power_topic,
            self.low_voltage_threshold,
            self.high_current_threshold,
        )
        if self.match_timer_enabled:
            logger.info(
                "Match timer: motion_threshold=%.2fm/s, duration=%.0fs, interval=%.0fs",
                self.motion_speed_threshold,
                self.match_duration_s,
                self.match_announce_interval_s,
            )

    # ------------------------------------------------------------------
    def _on_power_sample(self, sample) -> None:
        """Capture the latest power telemetry and evaluate alert conditions."""
        if not isinstance(sample, Mapping):
            return

        voltage = self._extract_float(sample, "input_voltage_V")
        if voltage is None:
            voltage = self._extract_float(sample, "output_voltage_V")

        current = self._extract_float(sample, "output_current_A")
        if current is None:
            current = self._extract_float(sample, "input_current_A")

        now = time.monotonic()
        self._last_voltage = voltage
        self._last_current = current
        self._last_sample_time = now

        if voltage is not None:
            self._evaluate_voltage_alert(voltage, now)

        if current is not None:
            self._evaluate_current_alert(current, now)

    def _on_teleop_command(self, sample) -> None:
        """Start the match timer when the teleop command exceeds the motion threshold."""
        if not self.match_timer_enabled or self._match_started:
            return

        vx, vy = self._extract_linear_velocity(sample)
        if vx is None or vy is None:
            return

        speed = math.hypot(vx, vy)
        if speed >= self.motion_speed_threshold:
            logger.info("Match timer starting from teleop motion: speed=%.3f m/s", speed)
            self._start_match_timer(time.monotonic())

    # ------------------------------------------------------------------
    def step(self) -> None:
        """Periodically re-check alerts so persistent issues re-announce."""
        now = time.monotonic()
        if self.match_timer_enabled:
            self._update_match_timer(now)

        if self._last_sample_time is None:
            return

        if (
            self._last_voltage is not None
            and self._last_voltage <= self.low_voltage_threshold
        ):
            self._evaluate_voltage_alert(self._last_voltage, now)

        if (
            self._last_current is not None
            and self._last_current >= self.high_current_threshold
        ):
            self._evaluate_current_alert(self._last_current, now)

    def _start_match_timer(self, now: float) -> None:
        if self._match_running:
            return
        self._match_started = True
        self._match_running = True
        self._match_start_time = now
        self._match_end_time = now + max(0.0, self.match_duration_s) - self._time_offset_seconds
        self._next_interval_remaining = max(0.0, self.match_duration_s)
        self._special_callouts_fired = {sec: False for sec, _ in self.match_special_callouts}
        self._match_completion_announced = False
        self._countdown_spoken = set()
        logger.info("Match timer started with %.1f seconds remaining", self.match_duration_s)
        if self._next_interval_remaining > 0.0:
            self._emit_time_callout(self._next_interval_remaining)
            self._schedule_next_interval()

    def _update_match_timer(self, now: float) -> None:
        if not self._match_running or self._match_end_time is None:
            return

        remaining = max(0.0, self._match_end_time - now)

        if remaining <= 10.0:
            self._next_interval_remaining = None
            self._handle_final_countdown(remaining)
        elif self._next_interval_remaining is not None:
            threshold = self._next_interval_remaining
            if remaining <= threshold + self.match_timer_tolerance_s:
                self._emit_time_callout(threshold)
                self._schedule_next_interval()

        if remaining > 10.0:
            for seconds, message in self.match_special_callouts:
                if self._special_callouts_fired.get(seconds, False):
                    continue
                if remaining <= seconds + self.match_timer_tolerance_s:
                    self._emit_special_callout(seconds, message)

        if remaining <= 0.0 and not self._match_completion_announced:
            self._match_completion_announced = True
            self._match_running = False
            if self.match_completion_message:
                self._speak(self.match_completion_message)

    def _schedule_next_interval(self) -> None:
        if self._next_interval_remaining is None:
            return
        self._next_interval_remaining -= self.match_announce_interval_s
        if self._next_interval_remaining is not None and self._next_interval_remaining <= 0.0:
            self._next_interval_remaining = None

    def _emit_time_callout(self, remaining_seconds: float) -> None:
        seconds_int = max(0, int(round(remaining_seconds)))
        base_message = f"{self._format_time(seconds_int)}."
        special_message = None
        for threshold, message in self.match_special_callouts:
            if threshold == seconds_int and not self._special_callouts_fired.get(threshold, False):
                special_message = message
                self._special_callouts_fired[threshold] = True
                break
        if special_message:
            announcement = f"{base_message} {special_message}".strip()
        else:
            announcement = base_message
        self._speak(announcement)

    def _handle_final_countdown(self, remaining: float) -> None:
        second = max(1, min(10, int(math.ceil(remaining))))
        if second in self._countdown_spoken:
            return
        message = f"{self._format_time(second)}."
        extra = None
        for threshold, special in self.match_special_callouts:
            if threshold == second and not self._special_callouts_fired.get(threshold, False):
                extra = special
                self._special_callouts_fired[threshold] = True
                break
        if extra:
            message = f"{message} {extra}".strip()
        self._countdown_spoken.add(second)
        self._speak(message)

    def _emit_special_callout(self, seconds: int, message: str) -> None:
        if self._special_callouts_fired.get(seconds, False):
            return
        announcement = f"{self._format_time(seconds)} {message}".strip()
        self._special_callouts_fired[seconds] = True
        self._speak(announcement)

    @staticmethod
    def _format_time(total_seconds: int) -> str:
        total_seconds = max(0, int(total_seconds))
        minutes, seconds = divmod(total_seconds, 60)
        parts: list[str] = []
        if minutes:
            parts.append(f"{minutes} ")
        if seconds or not parts:
            parts.append(f"{seconds} ")
        return " ".join(parts)

    # ------------------------------------------------------------------
    def _evaluate_voltage_alert(self, voltage: float, now: float) -> None:
        if voltage <= self.low_voltage_threshold:
            if self._can_alert(self._last_voltage_alert_time, now):
                self._speak(f"Battery at {voltage:.1f} volts. Swap soon.")
                self._last_voltage_alert_time = now
                self._voltage_in_alert = True
        elif self._voltage_in_alert and voltage >= self.low_voltage_threshold + self.voltage_clear_hysteresis:
            self._voltage_in_alert = False

    def _evaluate_current_alert(self, current: float, now: float) -> None:
        if current >= self.high_current_threshold:
            if self._can_alert(self._last_current_alert_time, now):
                self._speak(f"Current draw {current:.1f} amps. Watch the load.")
                self._last_current_alert_time = now
                self._current_in_alert = True
        elif self._current_in_alert and current <= self.high_current_threshold - self.current_clear_hysteresis:
            self._current_in_alert = False

    # ------------------------------------------------------------------
    @staticmethod
    def _extract_float(sample: Mapping[str, object], key: str) -> Optional[float]:
        value = sample.get(key)
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _can_alert(self, last_alert_time: Optional[float], now: float) -> bool:
        if last_alert_time is None:
            return True
        return (now - last_alert_time) >= self.alert_cooldown_s

    @staticmethod
    def _extract_linear_velocity(sample) -> tuple[Optional[float], Optional[float]]:
        vx = vy = None
        if isinstance(sample, Mapping):
            linear = sample.get("linear")
            if isinstance(linear, Mapping):
                vx = BatteryAlarmNode._extract_float(linear, "x")
                vy = BatteryAlarmNode._extract_float(linear, "y")
            else:
                vx = BatteryAlarmNode._extract_float(sample, "linear_x")
                vy = BatteryAlarmNode._extract_float(sample, "linear_y")
        else:
            try:
                linear = getattr(sample, "linear", None)
                if isinstance(linear, Mapping):
                    vx = BatteryAlarmNode._extract_float(linear, "x")
                    vy = BatteryAlarmNode._extract_float(linear, "y")
                elif linear is not None:
                    vx = float(getattr(linear, "x"))
                    vy = float(getattr(linear, "y"))
            except Exception:
                vx = vy = None
        return vx, vy

    def _speak(self, message: str) -> None:
        if not self._say_command:
            logger.info("Alert requested but macOS 'say' is unavailable: %s", message)
            return
        try:
            self._speech_queue.put_nowait(message)
        except Exception as exc:
            logger.error("Failed to enqueue speech message '%s': %s", message, exc)

    def _speech_worker(self) -> None:
        while True:
            try:
                message = self._speech_queue.get()
            except Exception:
                continue
            try:
                result = subprocess.run([self._say_command, "--", message], check=False)
                if result.returncode == 0:
                    logger.info("Spoke alert: %s", message)
                else:
                    logger.warning(
                        "'say' command exited with %s for message: %s", result.returncode, message
                    )
            except Exception as exc:
                logger.error("Failed to execute 'say' command: %s", exc)
            finally:
                self._speech_queue.task_done()
