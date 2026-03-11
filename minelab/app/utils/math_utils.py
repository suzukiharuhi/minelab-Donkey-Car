"""Math utilities (ROS-free)."""
from __future__ import annotations

import math


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp *value* to the closed interval [*lo*, *hi*]."""
    return max(lo, min(hi, value))


def wrap_angle_rad(angle: float) -> float:
    """Wrap *angle* (radians) into the range (−π, π]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def deg2rad(deg: float) -> float:
    return math.radians(deg)


def rad2deg(rad: float) -> float:
    return math.degrees(rad)
