"""Time utilities (ROS-free)."""
from __future__ import annotations

import time


def now() -> float:
    """Return the current monotonic time in seconds."""
    return time.monotonic()


def age(timestamp: float) -> float:
    """Return seconds elapsed since *timestamp* (monotonic)."""
    return time.monotonic() - timestamp
