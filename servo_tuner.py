#!/usr/bin/env python3
"""
Servo Tuner for Adafruit PCA9685

Interactive CLI to tune hobby servos driven by a PCA9685.

Features
- Set PWM frequency and target channel.
- Drive by raw pulse value, angle (radians), or normalized 0..1.
- Configure servo_min/servo_max (tick counts), wrist min/max radians.
- Configure claw open/closed normalized positions and drive them.
- Sweep across ranges for observation.
- Save/load JSON config.

Usage
  python3 servo_tuner.py [--addr 0x40] [--bus 1] [--freq 60] \
                         [--wrist 0] [--claw 1]

Then type commands at the prompt, e.g.:
  help                       Show all commands
  status                     Print current configuration
  ch 0                       Select channel 0 (current channel)
  pulse 375                  Set current channel to raw pulse 375
  angle 0.3                  Set current channel (wrist) to 0.3 rad
  norm 0.5                   Set current channel to normalized 0..1 position
  wristch 0                  Select wrist channel
  clawch 1                   Select claw channel
  open                       Drive claw to open position
  close                      Drive claw to closed position
  set servo_min 150          Set global servo_min
  set servo_max 600          Set global servo_max
  set wrist_min_rad -1.0     Set wrist min radians
  set wrist_max_rad 1.0      Set wrist max radians
  set open_norm 0.65         Set claw open normalized position
  set closed_norm 0.35       Set claw closed normalized position
  sweep angle 3              Sweep wrist angle min..max over 3 seconds
  sweep norm 3               Sweep normalized 0..1 over 3 seconds
  release                    Turn channel output off (pulse=0)
  save cfg.json              Save configuration to JSON
  load cfg.json              Load configuration from JSON
  quit                       Exit

Notes
- Pulses are in PCA9685 ticks (0..4095). Typical hobby servo: ~150..600 at 60Hz.
- Angle mapping is linear between wrist_min_rad..wrist_max_rad.
- If Adafruit_PCA9685 is unavailable, the tool runs in dry-run (prints only).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any

import numpy as np


try:
    import Adafruit_PCA9685  # Matches API used in repo example
except Exception:  # pragma: no cover
    Adafruit_PCA9685 = None  # type: ignore


@dataclass
class TunerConfig:
    address: int = 0x40
    busnum: int = 1
    pwm_freq: int = 60
    servo_min: int = 150
    servo_max: int = 600
    wrist_min_rad: float = -1.0
    wrist_max_rad: float = 1.0
    wrist_channel: int = 0
    claw_channel: int = 1
    claw_open_norm: float = 0.75
    claw_closed_norm: float = 0.45


class PCA9685Driver:
    def __init__(self, cfg: TunerConfig, dry_run: bool = False):
        self.cfg = cfg
        self.dry_run = dry_run or (Adafruit_PCA9685 is None)
        self.pwm = None
        if not self.dry_run:
            try:
                self.pwm = Adafruit_PCA9685.PCA9685(address=cfg.address, busnum=cfg.busnum)
                self.pwm.set_pwm_freq(cfg.pwm_freq)
                print(f"PCA9685 ready addr=0x{cfg.address:02X} bus={cfg.busnum} freq={cfg.pwm_freq}Hz")
            except Exception as e:
                print(f"WARN: PCA9685 init failed: {e}. Running in dry-run.")
                self.dry_run = True

    def set_freq(self, freq: int):
        self.cfg.pwm_freq = int(freq)
        if not self.dry_run and self.pwm is not None:
            try:
                self.pwm.set_pwm_freq(self.cfg.pwm_freq)
            except Exception as e:
                print(f"WARN: Failed to set PWM freq: {e}")

    def set_pwm(self, channel: int, pulse: int):
        pulse = int(max(0, min(4095, pulse)))
        if self.dry_run or self.pwm is None:
            print(f"[dry] ch{channel} <- pulse {pulse}")
            return
        try:
            self.pwm.set_pwm(int(channel), 0, pulse)
        except Exception as e:
            print(f"WARN: set_pwm failed on ch{channel}: {e}")


class ServoTuner:
    def __init__(self, cfg: TunerConfig, driver: PCA9685Driver):
        self.cfg = cfg
        self.drv = driver
        self.current_channel = cfg.wrist_channel

    # --- mapping helpers ---
    def clamp(self, v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    def angle_to_pulse(self, angle: float) -> int:
        angle = self.clamp(angle, self.cfg.wrist_min_rad, self.cfg.wrist_max_rad)
        span = max(self.cfg.wrist_max_rad - self.cfg.wrist_min_rad, 1e-6)
        t = (angle - self.cfg.wrist_min_rad) / span  # 0..1
        pulse = int(round(self.cfg.servo_min + t * (self.cfg.servo_max - self.cfg.servo_min)))
        return int(self.clamp(pulse, self.cfg.servo_min, self.cfg.servo_max))

    def norm_to_pulse(self, n: float) -> int:
        t = self.clamp(n, 0.0, 1.0)
        pulse = int(round(self.cfg.servo_min + t * (self.cfg.servo_max - self.cfg.servo_min)))
        return int(self.clamp(pulse, self.cfg.servo_min, self.cfg.servo_max))

    # --- high-level ops ---
    def set_pulse(self, channel: int, pulse: int):
        pulse = int(self.clamp(pulse, 0, 4095))
        self.drv.set_pwm(channel, pulse)

    def set_angle(self, channel: int, angle: float):
        pulse = self.angle_to_pulse(angle)
        print(f"angle {angle:.3f} rad -> pulse {pulse}")
        self.drv.set_pwm(channel, pulse)

    def set_norm(self, channel: int, n: float):
        pulse = self.norm_to_pulse(n)
        print(f"norm {n:.3f} -> pulse {pulse}")
        self.drv.set_pwm(channel, pulse)

    def sweep_angle(self, seconds: float = 3.0, steps: int = 60):
        lo, hi = self.cfg.wrist_min_rad, self.cfg.wrist_max_rad
        for i in range(steps + 1):
            t = i / steps
            ang = lo + t * (hi - lo)
            self.set_angle(self.cfg.wrist_channel, ang)
            time.sleep(max(0.0, seconds / steps))

    def sweep_norm(self, seconds: float = 3.0, steps: int = 60):
        for i in range(steps + 1):
            t = i / steps
            self.set_norm(self.current_channel, t)
            time.sleep(max(0.0, seconds / steps))

    def claw_open(self):
        self.set_norm(self.cfg.claw_channel, self.cfg.claw_open_norm)

    def claw_close(self):
        self.set_norm(self.cfg.claw_channel, self.cfg.claw_closed_norm)

    # --- persistence ---
    def save(self, path: str):
        with open(path, 'w') as f:
            json.dump(asdict(self.cfg), f, indent=2)
        print(f"Saved config -> {path}")

    def load(self, path: str):
        with open(path, 'r') as f:
            data = json.load(f)
        for k, v in data.items():
            if hasattr(self.cfg, k):
                setattr(self.cfg, k, v)
        # Re-apply frequency if changed
        self.drv.set_freq(self.cfg.pwm_freq)
        print(f"Loaded config <- {path}")

    # --- UI ---
    def print_status(self):
        print(
            "\n".join(
                [
                    "--- Servo Tuner Status ---",
                    f"addr=0x{self.cfg.address:02X} bus={self.cfg.busnum} freq={self.cfg.pwm_freq}Hz",
                    f"servo_min={self.cfg.servo_min} servo_max={self.cfg.servo_max}",
                    f"wrist_ch={self.cfg.wrist_channel} claw_ch={self.cfg.claw_channel} current_ch={self.current_channel}",
                    f"wrist_range_rad=[{self.cfg.wrist_min_rad:.3f}, {self.cfg.wrist_max_rad:.3f}]",
                    f"claw open_norm={self.cfg.claw_open_norm:.3f} closed_norm={self.cfg.claw_closed_norm:.3f}",
                ]
            )
        )

    def help(self):
        print(__doc__)

    def repl(self):
        self.print_status()
        print("Type 'help' for commands. Ctrl+C or 'quit' to exit.\n")
        try:
            while True:
                try:
                    line = input(
                        f"[ch={self.current_channel} min={self.cfg.servo_min} max={self.cfg.servo_max}]> "
                    )
                except EOFError:
                    break
                if not line:
                    continue
                if not self.dispatch(line.strip()):
                    break
        except KeyboardInterrupt:
            print("\nInterrupted.")

    def dispatch(self, line: str) -> bool:
        parts = line.split()
        if not parts:
            return True
        cmd, *args = parts
        cmd = cmd.lower()

        try:
            if cmd in ("quit", "exit", "q"):
                return False
            if cmd in ("help", "h", "?"):
                self.help(); return True
            if cmd == "status":
                self.print_status(); return True

            # Channel selection
            if cmd in ("ch", "channel"):
                self.current_channel = int(args[0]); print(f"current channel = {self.current_channel}"); return True
            if cmd == "wristch":
                self.current_channel = int(args[0]); self.cfg.wrist_channel = self.current_channel; print("wrist channel updated"); return True
            if cmd == "clawch":
                self.current_channel = int(args[0]); self.cfg.claw_channel = self.current_channel; print("claw channel updated"); return True

            # Driving
            if cmd == "pulse":
                self.set_pulse(self.current_channel, int(args[0])); return True
            if cmd == "angle":
                self.set_angle(self.current_channel, float(args[0])); return True
            if cmd == "norm":
                self.set_norm(self.current_channel, float(args[0])); return True
            if cmd == "release":
                self.set_pulse(self.current_channel, 0); return True

            # Sweeps
            if cmd == "sweep":
                mode = args[0]
                seconds = float(args[1]) if len(args) > 1 else 3.0
                if mode == "angle":
                    self.sweep_angle(seconds)
                elif mode == "norm":
                    self.sweep_norm(seconds)
                else:
                    print("Usage: sweep angle|norm [seconds]")
                return True

            # Claw helpers
            if cmd == "open":
                self.claw_open(); return True
            if cmd == "close":
                self.claw_close(); return True

            # Settings
            if cmd == "set":
                key = args[0].lower(); val = args[1]
                if key == "servo_min":
                    self.cfg.servo_min = int(val)
                elif key == "servo_max":
                    self.cfg.servo_max = int(val)
                elif key == "wrist_min_rad":
                    self.cfg.wrist_min_rad = float(val)
                elif key == "wrist_max_rad":
                    self.cfg.wrist_max_rad = float(val)
                elif key in ("open_norm", "claw_open_norm"):
                    self.cfg.claw_open_norm = float(val)
                elif key in ("closed_norm", "claw_closed_norm"):
                    self.cfg.claw_closed_norm = float(val)
                elif key in ("freq", "pwm_freq"):
                    self.drv.set_freq(int(val))
                else:
                    print("Unknown setting. Valid: servo_min, servo_max, wrist_min_rad, wrist_max_rad, open_norm, closed_norm, freq")
                    return True
                print(f"Updated {key} -> {val}")
                return True

            # Persistence
            if cmd == "save":
                path = args[0]; self.save(path); return True
            if cmd == "load":
                path = args[0]; self.load(path); return True

            print("Unknown command. Type 'help' for usage.")
            return True
        except (IndexError, ValueError) as e:
            print(f"Error: {e}. Type 'help' for usage.")
            return True


def parse_args(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Servo tuner for Adafruit PCA9685")
    ap.add_argument("--addr", type=lambda s: int(s, 0), default=0x40, help="I2C address (e.g., 0x40)")
    ap.add_argument("--bus", type=int, default=1, help="I2C bus number")
    ap.add_argument("--freq", type=int, default=60, help="PWM frequency (Hz)")
    ap.add_argument("--wrist", type=int, default=0, help="Wrist servo channel")
    ap.add_argument("--claw", type=int, default=1, help="Claw servo channel")
    # Typical hobby servo range at ~60Hz is around 150..600 ticks
    ap.add_argument("--servo-min", dest="servo_min", type=int, default=150, help="Servo min pulse (ticks)")
    ap.add_argument("--servo-max", dest="servo_max", type=int, default=600, help="Servo max pulse (ticks)")
    ap.add_argument("--wrist-min", dest="wrist_min", type=float, default=0.0, help="Wrist min angle (rad)")
    ap.add_argument("--wrist-max", dest="wrist_max", type=float, default=np.pi, help="Wrist max angle (rad)")
    ap.add_argument("--open-norm", dest="open_norm", type=float, default=0.1, help="Claw open normalized position")
    ap.add_argument("--closed-norm", dest="closed_norm", type=float, default=0.55, help="Claw closed normalized position")
    ap.add_argument("--dry-run", action="store_true", help="Do not access hardware; print only")
    return ap.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    # Heuristic: If user passed values that look like microseconds (e.g., 500/2500), warn.
    if args.servo_min > 1000 or args.servo_max > 1000:
        print(
            "WARN: servo_min/servo_max look high. Values are PCA9685 ticks (0..4095).\n"
            "      Typical range is ~150..600 at 60Hz. Use --servo-min/--servo-max accordingly."
        )
    cfg = TunerConfig(
        address=args.addr,
        busnum=args.bus,
        pwm_freq=args.freq,
        servo_min=args.servo_min,
        servo_max=args.servo_max,
        wrist_min_rad=args.wrist_min,
        wrist_max_rad=args.wrist_max,
        wrist_channel=args.wrist,
        claw_channel=args.claw,
        claw_open_norm=args.open_norm,
        claw_closed_norm=args.closed_norm,
    )
    drv = PCA9685Driver(cfg, dry_run=args.dry_run)
    tuner = ServoTuner(cfg, drv)
    tuner.repl()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
