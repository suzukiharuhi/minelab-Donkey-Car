"""IMU sensor stub (ROS-free base).

The actual ROS subscription lives in ``nodes/vehicle_io_node.py`` (or a
dedicated sensor node).  This class is a data-holding stub that the node
populates via ``update()``.
"""
from __future__ import annotations

import logging
import threading
from typing import Optional

from minelab.app.sensors.base_sensor import BaseSensor
from minelab.interfaces.data_models import SensorSnapshot

logger = logging.getLogger(__name__)


class IMUSensor(BaseSensor):
    """Thread-safe IMU data holder (ROS-free)."""

    def __init__(self) -> None:
        super().__init__(name="IMUSensor")
        self._lock = threading.Lock()
        self._latest: Optional[SensorSnapshot] = None

    def update(self, snapshot: SensorSnapshot) -> None:
        """Update latest data (called by ROS callback in the node layer)."""
        with self._lock:
            self._latest = snapshot
            self._initialized = True

    def get_latest_data(self) -> Optional[SensorSnapshot]:
        with self._lock:
            return self._latest
