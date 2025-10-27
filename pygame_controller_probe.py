#!/usr/bin/env python3
"""
Standalone pygame controller mapping probe.

Purpose:
  - Identify axis indices for sticks and triggers (L2/R2)
  - Identify button indices for square/cross/circle/triangle/L1/R1/etc
  - Inspect hat (D-pad) mapping

Usage:
  python3 pygame_controller_probe.py [--index N] [--hz 60] [--deadzone 0.02]

Notes:
  - Prints only on change (axes beyond deadzone, button/hat edges)
  - Normalizes triggers from raw [-1,1] to [0,1] and reports both
  - Press ESC or Ctrl+C to quit
"""

import os
import sys
import time
import argparse
from typing import List, Tuple

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')  # allow headless

import pygame  # type: ignore


def norm_axis(value: float, dz: float = 0.02) -> float:
    if abs(value) < dz:
        return 0.0
    # scale after deadzone for better readability
    sign = 1.0 if value >= 0.0 else -1.0
    scaled = (abs(value) - dz) / max(1.0 - dz, 1e-6)
    return max(0.0, min(1.0, scaled)) * sign


def norm_trigger(raw: float) -> float:
    # Map from [-1, 1] to [0, 1]
    return max(0.0, min(1.0, (raw + 1.0) * 0.5))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--index', type=int, default=0, help='Joystick index (default 0)')
    ap.add_argument('--hz', type=float, default=60.0, help='Poll rate (Hz)')
    ap.add_argument('--deadzone', type=float, default=0.02, help='Axis deadzone for change printing')
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count <= 0:
        print('No joystick detected. Plug in a controller and try again.')
        return 1
    if args.index < 0 or args.index >= count:
        print(f'Invalid joystick index {args.index}; available: 0..{count-1}')
        return 2

    js = pygame.joystick.Joystick(args.index)
    js.init()

    name = js.get_name()
    axes = js.get_numaxes()
    buttons = js.get_numbuttons()
    hats = js.get_numhats()

    print('Controller connected:')
    print(f'  Name   : {name}')
    print(f'  Axes   : {axes}')
    print(f'  Buttons: {buttons}')
    print(f'  Hats   : {hats}')
    print('Move each control; changes will print below. ESC or Ctrl+C to quit.')
    print('-' * 72)

    prev_axes: List[float] = [0.0 for _ in range(axes)]
    prev_buttons: List[int] = [0 for _ in range(buttons)]
    prev_hats: List[Tuple[int, int]] = [tuple([0, 0]) for _ in range(hats)]

    interval = 1.0 / max(args.hz, 1e-3)
    last_summary = 0.0

    try:
        while True:
            start = time.time()
            pygame.event.pump()

            # Read axes
            ax_changed = False
            for i in range(axes):
                try:
                    raw = float(js.get_axis(i))
                except Exception:
                    raw = 0.0
                # report when normalized delta exceeds threshold
                n_prev = prev_axes[i]
                n_curr = norm_axis(raw, args.deadzone)
                if abs(n_curr - n_prev) > 1e-3:
                    print(f'AXIS[{i}] raw={raw:+.3f} norm={n_curr:+.3f} trig_norm={norm_trigger(raw):.3f}')
                    prev_axes[i] = n_curr
                    ax_changed = True

            # Read buttons
            for i in range(buttons):
                try:
                    b = int(bool(js.get_button(i)))
                except Exception:
                    b = 0
                if b != prev_buttons[i]:
                    print(f'BUTTON[{i}] -> {"DOWN" if b else "UP"}')
                    prev_buttons[i] = b

            # Read hats
            for i in range(hats):
                try:
                    h = js.get_hat(i)
                except Exception:
                    h = (0, 0)
                if h != prev_hats[i]:
                    print(f'HAT[{i}] -> x={h[0]} y={h[1]}')
                    prev_hats[i] = h

            # Periodic summary if quiet
            now = time.time()
            if not ax_changed and now - last_summary > 2.0:
                try:
                    ax_vals = [f'{js.get_axis(i):+.2f}' for i in range(axes)]
                except Exception:
                    ax_vals = []
                print('Summary:', ' '.join(f'A{i}={v}' for i, v in enumerate(ax_vals)))
                last_summary = now

            # Exit on ESC
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    raise KeyboardInterrupt

            # Sleep to maintain rate
            dt = time.time() - start
            sleep_s = max(0.0, interval - dt)
            time.sleep(sleep_s)

    except KeyboardInterrupt:
        print('\nExiting.')
    finally:
        try:
            js.quit()
        except Exception:
            pass
        pygame.joystick.quit()
        pygame.quit()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())

