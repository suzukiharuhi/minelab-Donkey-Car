"""Auto-charging state skeleton (ROS-free)."""
from __future__ import annotations

import logging
from typing import Any, Optional

from minelab.app.core.commands import ControlCommand
from minelab.app.states.base_state import BaseState
from minelab.app.states.state_type import StateType

logger = logging.getLogger(__name__)


class AutoChargingState(BaseState):
    """Top-level charging sequence; sub-states are handled elsewhere."""

    def enter(self) -> None:
        logger.info("[AutoChargingState] Entered AUTO_CHARGING")

    def execute(self, imu_data: Any, depth_features: Any, marker_info: Any) -> ControlCommand:
        return ControlCommand.stop()

    def check_transition(
        self, imu_data: Any, depth_features: Any, marker_info: Any
    ) -> Optional[StateType]:
        return StateType.APPROACH_ENTRY_POINT

    def exit(self) -> None:
        logger.info("[AutoChargingState] Exiting AUTO_CHARGING")
