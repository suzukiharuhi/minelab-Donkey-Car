"""Abstract base sensor class (ROS-free).

Concrete sensors that live in ``nodes/`` may extend this with ROS
subscriber logic.  The base class itself has no ROS dependency.
"""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Any, Optional

logger = logging.getLogger(__name__)


class BaseSensor(ABC):
    """Minimal sensor interface with thread-safe latest-value access."""

    def __init__(self, name: str = "BaseSensor") -> None:
        self._name = name
        self._initialized: bool = False

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @abstractmethod
    def get_latest_data(self) -> Optional[Any]:
        """Return the most recently received data, or ``None``."""
