"""Idle state – waits for initial sensor data or a start signal (ROS-free)."""
from __future__ import annotations

import logging
from typing import Any, Optional

from minelab.app.core.commands import ControlCommand
from minelab.app.states.base_state import BaseState
from minelab.app.states.state_type import StateType

logger = logging.getLogger(__name__)


class IdleState(BaseState):
    """Idle: hold stop, transition to CROP_NAVIGATION once sensors are ready."""

    def enter(self) -> None:
        logger.info("[IdleState] Entered IDLE")

    def execute(
        self,
        imu_data: Any,
        depth_features: Any,
        marker_info: Any,
    ) -> ControlCommand:
        return ControlCommand.stop()

    def check_transition(
        self,
        imu_data: Any,
        depth_features: Any,
        marker_info: Any,
    ) -> Optional[StateType]:
        if imu_data is not None and depth_features is not None:
            return StateType.CROP_NAVIGATION
        return None

    def exit(self) -> None:
        logger.info("[IdleState] Exiting IDLE")
