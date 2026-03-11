"""RealSense camera data holder (ROS-free base).

Holds the latest colour and depth frames received by the node layer.
"""
from __future__ import annotations

import logging
import threading
from typing import Optional

import numpy as np

from minelab.app.sensors.base_sensor import BaseSensor

logger = logging.getLogger(__name__)


class RealSenseSensor(BaseSensor):
    """Thread-safe camera frame holder (ROS-free)."""

    def __init__(self) -> None:
        super().__init__(name="RealSenseSensor")
        self._lock = threading.Lock()
        self._color: Optional[np.ndarray] = None
        self._depth: Optional[np.ndarray] = None
        self._color_ts: float = 0.0
        self._depth_ts: float = 0.0

    def update_color(self, frame: np.ndarray, timestamp: float = 0.0) -> None:
        with self._lock:
            self._color = frame
            self._color_ts = timestamp
            self._initialized = True

    def update_depth(self, frame: np.ndarray, timestamp: float = 0.0) -> None:
        with self._lock:
            self._depth = frame
            self._depth_ts = timestamp
            self._initialized = True

    def get_latest_data(self):
        with self._lock:
            return self._color, self._depth, self._color_ts, self._depth_ts
