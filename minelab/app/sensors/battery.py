"""Battery sensor stub (ROS-free base)."""
from __future__ import annotations

import logging
import threading
from typing import Optional

from minelab.app.sensors.base_sensor import BaseSensor
from minelab.interfaces.data_models import SensorSnapshot

logger = logging.getLogger(__name__)


class BatterySensor(BaseSensor):
    """Thread-safe battery data holder (ROS-free)."""

    def __init__(self) -> None:
        super().__init__(name="BatterySensor")
        self._lock = threading.Lock()
        self._latest: Optional[SensorSnapshot] = None

    def update(self, snapshot: SensorSnapshot) -> None:
        with self._lock:
            self._latest = snapshot
            self._initialized = True

    def get_latest_data(self) -> Optional[SensorSnapshot]:
        with self._lock:
            return self._latest
