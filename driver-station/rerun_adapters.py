#!/usr/bin/env python3
"""
Adapters that convert Tide SDK Pydantic types to Rerun types/primitives.

These helpers keep the visualization node simple and decoupled from
the underlying message schemas.
"""

from __future__ import annotations

from typing import Any, Mapping, Optional, Tuple

try:
    # Tide SDK Pydantic models (if available at runtime)
    from tide.models import Quaternion as TideQuaternion
    from tide.models import Vector3 as TideVector3
    from tide.models import Vector2 as TideVector2
except Exception:  # pragma: no cover - fall back to duck typing
    TideQuaternion = Any  # type: ignore
    TideVector3 = Any  # type: ignore
    TideVector2 = Any  # type: ignore

import math
import rerun as rr


def quat_wxyz_to_rr_rotation(w: float, x: float, y: float, z: float) -> rr.Quaternion:
    """Create a Rerun Quaternion from a WXYZ quaternion.

    Note: rerun>=0.24 uses `QuaternionLike`/`RotationAxisAngleLike` for Transform3D.
    We return a `rr.Quaternion` object which `rr.Transform3D` accepts directly.
    """
    # Rerun expects XYZW ordering for quaternions.
    return rr.Quaternion(xyzw=(x, y, z, w))


def tide_quat_to_rr_rotation(quat: TideQuaternion | Mapping[str, float]) -> rr.Quaternion:
    """Convert Tide Quaternion (w,x,y,z) or dict to a Rerun Quaternion."""
    if hasattr(quat, "w"):
        return quat_wxyz_to_rr_rotation(quat.w, quat.x, quat.y, quat.z)
    # Dict-like sample already parsed by Tide
    return quat_wxyz_to_rr_rotation(quat["w"], quat["x"], quat["y"], quat["z"])  # type: ignore[index]


def tide_vec3_to_tuple(v: TideVector3 | Mapping[str, float]) -> Tuple[float, float, float]:
    """Convert Tide Vector3 to a simple (x, y, z) tuple."""
    if hasattr(v, "x"):
        return (float(v.x), float(v.y), float(v.z))
    return (float(v["x"]), float(v["y"]), float(v["z"]))  # type: ignore[index]


def tide_vec2_to_tuple(v: TideVector2 | Mapping[str, float]) -> Tuple[float, float]:
    """Convert Tide Vector2 to a simple (x, y) tuple."""
    if hasattr(v, "x"):
        return (float(v.x), float(v.y))
    return (float(v["x"]), float(v["y"]))  # type: ignore[index]


def yaw_to_rr_rotation(yaw_rad: float) -> rr.RotationAxisAngle:
    """Create a RotationAxisAngle representing a yaw about +Z by yaw_rad."""
    return rr.RotationAxisAngle(axis=(0.0, 0.0, 1.0), angle=yaw_rad)


def pose2d_to_transform(
    x: float,
    y: float,
    yaw_rad: float,
    z: float = 0.0,
) -> rr.Transform3D:
    """Build a Rerun Transform3D from 2D pose (x,y,yaw) placed at height z."""
    return rr.Transform3D(
        translation=(x, y, z),
        rotation=yaw_to_rr_rotation(yaw_rad),
    )


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
