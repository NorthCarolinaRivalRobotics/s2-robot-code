#!/usr/bin/env python3
"""PyQt-based UI node for adjusting arm joint angle offsets."""

from __future__ import annotations

import logging
import math
import multiprocessing as mp
import queue
import time
from typing import Any, Dict, Tuple

import numpy as np

from tide.core.node import BaseNode
from tide.namespaces import robot_topic
from tide.models import Vector2
from tide.models.serialization import to_zenoh_value


_OffsetTuple = Tuple[float, float]


class ArmOffsetUINode(BaseNode):
    """Node that launches a PyQt UI for editing arm joint offsets."""

    def __init__(self, *, config: Dict[str, Any] | None = None):
        super().__init__(config=config)

        cfg = config or {}
        self.robot_id = cfg.get("robot_id", self.ROBOT_ID)
        self.hz = float(cfg.get("update_rate", 20.0))

        self.logger = logging.getLogger(f"ArmOffsetUI_{self.robot_id}")

        self.arm_offset_topic = robot_topic(self.robot_id, "cmd/arm/offsets")
        self._arm_offsets = self._load_initial_offsets(cfg)
        self._last_published = self._arm_offsets.copy()

        self._mp_ctx = mp.get_context("spawn")
        self._ui_to_node: mp.Queue = self._mp_ctx.Queue(maxsize=64)
        self._node_to_ui: mp.Queue = self._mp_ctx.Queue(maxsize=64)
        self._ui_stop_event = self._mp_ctx.Event()
        self._ui_process: mp.Process | None = None

        self._last_ui_restart_attempt = 0.0
        self._ui_restart_backoff = float(cfg.get("ui_restart_backoff_s", 1.0))

        # Publish initial offsets once UI is running
        self._should_republish_offsets = True

        self.logger.info(
            "Arm offset UI node initialized for robot %s (shoulder=%+.3f rad, elbow=%+.3f rad)",
            self.robot_id,
            float(self._arm_offsets[0]),
            float(self._arm_offsets[1]),
        )

    # Lifecycle -----------------------------------------------------------------
    def start(self):
        self._start_ui_process()
        self._publish_offsets(force=True)
        return super().start()

    def step(self) -> None:
        self._drain_ui_messages()
        self._check_ui_health()

    def stop(self) -> None:
        self.logger.info("Stopping ArmOffsetUINode")
        try:
            self._ui_stop_event.set()
            self._send_to_ui({"type": "shutdown"})
        except Exception:
            pass

        if self._ui_process is not None:
            self._ui_process.join(timeout=2.0)
            if self._ui_process.is_alive():
                self.logger.warning("Force terminating unresponsive UI process")
                self._ui_process.terminate()
            self._ui_process = None

        try:
            self._ui_to_node.close()
            self._node_to_ui.close()
        except Exception:
            pass

        super().stop()

    # Internal helpers ----------------------------------------------------------
    def _load_initial_offsets(self, cfg: Dict[str, Any]) -> np.ndarray:
        rad_cfg = cfg.get("arm_joint_offsets_rad")
        deg_cfg = cfg.get("arm_joint_offsets_deg") if rad_cfg is None else None

        offsets = self._parse_offsets(rad_cfg)
        if offsets is not None:
            return offsets

        offsets = self._parse_offsets(deg_cfg)
        if offsets is not None:
            return np.deg2rad(offsets)

        return np.array([0.0, 0.0], dtype=float)

    def _parse_offsets(self, raw) -> np.ndarray | None:
        if raw is None:
            return None
        try:
            if isinstance(raw, dict):
                shoulder = float(raw.get("shoulder", 0.0))
                elbow = float(raw.get("elbow", 0.0))
            elif isinstance(raw, (list, tuple)) and len(raw) >= 2:
                shoulder = float(raw[0])
                elbow = float(raw[1])
            else:
                return None
            return np.array([shoulder, elbow], dtype=float)
        except Exception as exc:
            self.logger.debug("Invalid arm offset config %s: %s", raw, exc)
            return None

    def _start_ui_process(self) -> None:
        if self._ui_process is not None and self._ui_process.is_alive():
            return

        self._ui_stop_event.clear()
        self._ui_process = self._mp_ctx.Process(
            target=_run_ui_process,
            name=f"{self.robot_id}_ArmOffsetUI",
            args=(
                self.robot_id,
                self._node_to_ui,
                self._ui_to_node,
                self._ui_stop_event,
            ),
            daemon=True,
        )
        self._ui_process.start()
        self.logger.info("Launched arm offset UI process (pid=%s)", self._ui_process.pid)
        self._send_offsets_to_ui(force=True)

    def _send_offsets_to_ui(self, *, force: bool = False) -> None:
        payload = {
            "type": "set_offsets",
            "shoulder_rad": float(self._arm_offsets[0]),
            "elbow_rad": float(self._arm_offsets[1]),
        }
        if force:
            payload["force"] = True
        self._send_to_ui(payload)

    def _send_to_ui(self, message: Dict[str, Any]) -> None:
        try:
            self._node_to_ui.put_nowait(message)
        except queue.Full:
            self.logger.debug("UI outbound queue full; dropping message %s", message.get("type"))

    def _drain_ui_messages(self) -> None:
        updated = False
        while True:
            try:
                message = self._ui_to_node.get_nowait()
            except queue.Empty:
                break

            if not isinstance(message, dict):
                continue

            msg_type = message.get("type")
            if msg_type == "offsets":
                updated |= self._handle_offset_update(message)
            elif msg_type == "ready":
                self._send_offsets_to_ui(force=True)
            elif msg_type == "closed":
                self.logger.info("Arm offset UI window closed by user")
                self._ui_stop_event.set()
            elif msg_type == "log":
                level = message.get("level", "info").lower()
                text = str(message.get("message", ""))
                getattr(self.logger, level if hasattr(self.logger, level) else "info")(text)

        if updated or self._should_republish_offsets:
            self._publish_offsets(force=updated or self._should_republish_offsets)
            self._should_republish_offsets = False

    def _handle_offset_update(self, message: Dict[str, Any]) -> bool:
        if "shoulder_rad" in message and "elbow_rad" in message:
            new_offsets = np.array(
                [float(message["shoulder_rad"]), float(message["elbow_rad"])],
                dtype=float,
            )
        else:
            shoulder_deg = float(message.get("shoulder_deg", 0.0))
            elbow_deg = float(message.get("elbow_deg", 0.0))
            new_offsets = np.deg2rad(np.array([shoulder_deg, elbow_deg], dtype=float))

        if np.allclose(new_offsets, self._arm_offsets, atol=1e-6):
            return False

        self._arm_offsets = new_offsets
        self.logger.info(
            "Updated offsets from UI (shoulder=%+.3f rad, elbow=%+.3f rad)",
            float(self._arm_offsets[0]),
            float(self._arm_offsets[1]),
        )
        self._send_offsets_to_ui()
        return True

    def _publish_offsets(self, *, force: bool = False) -> None:
        if not force and np.allclose(self._arm_offsets, self._last_published, atol=1e-6):
            return
        try:
            payload = Vector2(
                x=float(self._arm_offsets[0]),
                y=float(self._arm_offsets[1]),
            )
            self.put(self.arm_offset_topic, to_zenoh_value(payload))
            self._last_published = self._arm_offsets.copy()
            self.logger.debug(
                "Published arm offsets (shoulder=%+.4f rad, elbow=%+.4f rad) to %s",
                float(self._arm_offsets[0]),
                float(self._arm_offsets[1]),
                self.arm_offset_topic,
            )
        except Exception as exc:
            self.logger.warning("Failed to publish arm offsets: %s", exc)

    def _check_ui_health(self) -> None:
        if self._ui_process is None:
            return

        if self._ui_process.is_alive() or self._ui_stop_event.is_set():
            return

        now = time.time()
        if (now - self._last_ui_restart_attempt) < self._ui_restart_backoff:
            return

        self.logger.warning("Arm offset UI exited unexpectedly; restarting")
        self._last_ui_restart_attempt = now
        self._start_ui_process()


# -----------------------------------------------------------------------------
# PyQt UI process
# -----------------------------------------------------------------------------


def _run_ui_process(
    robot_id: str,
    inbound: mp.Queue,
    outbound: mp.Queue,
    stop_event: mp.Event,
) -> None:
    """Entry point for the PyQt UI process."""

    try:
        from PyQt5 import QtCore, QtWidgets
    except ImportError as exc:
        try:
            outbound.put_nowait({
                "type": "log",
                "level": "error",
                "message": f"PyQt5 not available: {exc}",
            })
        except Exception:
            pass
        return

    class ArmOffsetWindow(QtWidgets.QWidget):
        def __init__(self, initial_deg: _OffsetTuple):
            super().__init__()

            self.setWindowTitle(f"{robot_id} Arm Joint Offsets")
            self._inbound = inbound
            self._outbound = outbound
            self._stop_event = stop_event

            self._shoulder_spin = QtWidgets.QDoubleSpinBox()
            self._shoulder_spin.setRange(-180.0, 180.0)
            self._shoulder_spin.setDecimals(3)
            self._shoulder_spin.setSingleStep(5.0)

            self._elbow_spin = QtWidgets.QDoubleSpinBox()
            self._elbow_spin.setRange(-180.0, 180.0)
            self._elbow_spin.setDecimals(3)
            self._elbow_spin.setSingleStep(5.0)

            self._shoulder_label = QtWidgets.QLabel()
            self._elbow_label = QtWidgets.QLabel()

            zero_button = QtWidgets.QPushButton("Zero Offsets")
            zero_button.clicked.connect(self._zero_offsets)

            layout = QtWidgets.QGridLayout()
            layout.addWidget(QtWidgets.QLabel("Shoulder (deg)"), 0, 0)
            layout.addWidget(self._shoulder_spin, 0, 1)
            layout.addWidget(self._shoulder_label, 0, 2)
            layout.addWidget(QtWidgets.QLabel("Elbow (deg)"), 1, 0)
            layout.addWidget(self._elbow_spin, 1, 1)
            layout.addWidget(self._elbow_label, 1, 2)
            layout.addWidget(zero_button, 2, 0, 1, 3)

            self.setLayout(layout)

            self._pending_emit = False

            self._shoulder_spin.valueChanged.connect(self._on_value_changed)
            self._elbow_spin.valueChanged.connect(self._on_value_changed)

            self._emit_timer = QtCore.QTimer(self)
            self._emit_timer.setInterval(100)
            self._emit_timer.timeout.connect(self._flush_pending)
            self._emit_timer.start()

            self._poll_timer = QtCore.QTimer(self)
            self._poll_timer.setInterval(100)
            self._poll_timer.timeout.connect(self._poll_incoming)
            self._poll_timer.start()

            self._apply_deg(initial_deg, emit=False)

        # UI helpers -----------------------------------------------------
        def _format_rad(self, value_deg: float) -> str:
            return f"{math.radians(value_deg):+.4f} rad"

        def _apply_deg(self, offsets_deg: _OffsetTuple, *, emit: bool = True) -> None:
            with QtCore.QSignalBlocker(self._shoulder_spin), QtCore.QSignalBlocker(self._elbow_spin):
                self._shoulder_spin.setValue(float(offsets_deg[0]))
                self._elbow_spin.setValue(float(offsets_deg[1]))
            self._update_labels()
            if emit:
                self._schedule_emit()

        def _update_labels(self) -> None:
            self._shoulder_label.setText(self._format_rad(self._shoulder_spin.value()))
            self._elbow_label.setText(self._format_rad(self._elbow_spin.value()))

        def _schedule_emit(self) -> None:
            self._pending_emit = True

        def _on_value_changed(self) -> None:
            self._update_labels()
            self._schedule_emit()

        def _flush_pending(self) -> None:
            if not self._pending_emit:
                return
            self._pending_emit = False
            message = {
                "type": "offsets",
                "shoulder_deg": float(self._shoulder_spin.value()),
                "elbow_deg": float(self._elbow_spin.value()),
            }
            try:
                self._outbound.put_nowait(message)
            except queue.Full:
                self._pending_emit = True

        def _poll_incoming(self) -> None:
            if self._stop_event.is_set():
                self.close()
                return
            while True:
                try:
                    message = self._inbound.get_nowait()
                except queue.Empty:
                    break
                if not isinstance(message, dict):
                    continue
                msg_type = message.get("type")
                if msg_type == "set_offsets":
                    if "shoulder_rad" in message and "elbow_rad" in message:
                        offsets_deg = (
                            math.degrees(float(message["shoulder_rad"])),
                            math.degrees(float(message["elbow_rad"])),
                        )
                    else:
                        offsets_deg = (
                            float(message.get("shoulder_deg", 0.0)),
                            float(message.get("elbow_deg", 0.0)),
                        )
                    self._apply_deg(offsets_deg, emit=False)
                elif msg_type == "shutdown":
                    self.close()
                elif msg_type == "log":
                    # Bubble up log messages from node to stdout for visibility
                    level = message.get("level", "info").upper()
                    text = message.get("message", "")
                    print(f"[{level}] {text}")

        def _zero_offsets(self) -> None:
            self._apply_deg((0.0, 0.0))

        def closeEvent(self, event) -> None:  # type: ignore[override]
            try:
                self._outbound.put_nowait({"type": "closed"})
            except Exception:
                pass
            super().closeEvent(event)

    app = QtWidgets.QApplication([])
    window = ArmOffsetWindow((0.0, 0.0))
    window.show()
    try:
        outbound.put_nowait({"type": "ready"})
    except Exception:
        pass
    app.exec()


__all__ = ["ArmOffsetUINode"]
