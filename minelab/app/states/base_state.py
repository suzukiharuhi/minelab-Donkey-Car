"""Abstract base class for all UGV states (ROS-free).

Concrete states implement ``enter``, ``execute``, ``check_transition``,
and ``exit``.  The ``execute`` method receives the latest sensor data and
must return a ``ControlCommand``.
"""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import Any, Optional

from minelab.app.core.commands import ControlCommand
from minelab.app.states.state_type import StateType

logger = logging.getLogger(__name__)


class BaseState(ABC):
    """Abstract base for all states in the minelab state machine."""

    def __init__(self) -> None:
        self.state_name: str = self.__class__.__name__
        self._miss_count: int = 0
        self._miss_limit_default: int = 5

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abstractmethod
    def enter(self) -> None:
        """Called once when transitioning into this state."""

    @abstractmethod
    def execute(
        self,
        imu_data: Any,
        depth_features: Any,
        marker_info: Any,
    ) -> ControlCommand:
        """Produce a control command for this cycle.

        Args:
            imu_data:       Latest ``SensorSnapshot`` (may be ``None``).
            depth_features: Latest ``DepthFeatures`` (may be ``None``).
            marker_info:    Latest ``MarkerInfo`` (may be ``None``).

        Returns:
            ``ControlCommand`` to apply this cycle.
        """

    @abstractmethod
    def check_transition(
        self,
        imu_data: Any,
        depth_features: Any,
        marker_info: Any,
    ) -> Optional[StateType]:
        """Return the next ``StateType`` if a transition should occur, else ``None``."""

    @abstractmethod
    def exit(self) -> None:
        """Called once when leaving this state."""

    # ------------------------------------------------------------------
    # Miss-count helpers (shared pattern across most states)
    # ------------------------------------------------------------------

    def reset_miss_count(self) -> None:
        self._miss_count = 0

    def inc_miss_count(self) -> int:
        self._miss_count += 1
        return self._miss_count

    def get_miss_count(self) -> int:
        return self._miss_count

    def handle_transient_loss(
        self,
        recover_command: ControlCommand,
        miss_limit: Optional[int] = None,
        log_tag: str = "[BaseState]",
    ) -> ControlCommand:
        """Return ``recover_command`` up to ``miss_limit``; then STOP."""
        limit = miss_limit if miss_limit is not None else self._miss_limit_default
        if self._miss_count <= limit:
            logger.info(
                "%s transient loss (miss=%d/%d) → recover", log_tag, self._miss_count, limit
            )
            return recover_command
        logger.info(
            "%s loss exceeded limit (miss=%d/%d) → STOP", log_tag, self._miss_count, limit
        )
        return ControlCommand.stop()

    def is_lost_beyond_limit(
        self,
        miss_limit: Optional[int] = None,
        log_tag: str = "[BaseState]",
    ) -> bool:
        limit = miss_limit if miss_limit is not None else self._miss_limit_default
        if self._miss_count > limit:
            logger.info("%s lost beyond limit (miss=%d/%d)", log_tag, self._miss_count, limit)
            return True
        return False
