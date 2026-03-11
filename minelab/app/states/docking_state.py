"""Docking state – align with and enter the charging station (ROS-free)."""
from __future__ import annotations

import logging
from typing import Any, Optional

from minelab.app.core.commands import ControlCommand, SpeedLevel
from minelab.app.states.base_state import BaseState
from minelab.app.states.state_type import StateType
from minelab.interfaces.data_models import MarkerInfo

logger = logging.getLogger(__name__)

_DOCK_MARKER_ID = 20              # charging-station marker
_DOCKED_DISTANCE_CM = 15.0       # within this range → charging
_ALIGN_LATERAL_THRESH_CM = 3.0
_ALIGN_YAW_THRESH_DEG = 5.0


class DockingState(BaseState):
    """Align precisely with the charging-station marker and advance until docked."""

    def enter(self) -> None:
        logger.info("[DockingState] Entered DOCKING")
        self.reset_miss_count()

    def execute(
        self,
        imu_data: Any,
        depth_features: Any,
        marker_info: Optional[MarkerInfo],
    ) -> ControlCommand:
        if marker_info is None or not marker_info.visible:
            self.inc_miss_count()
            return self.handle_transient_loss(
                ControlCommand.stop(),
                miss_limit=3,
                log_tag="[DockingState]",
            )

        self.reset_miss_count()

        if marker_info.distance_z_cm <= _DOCKED_DISTANCE_CM:
            return ControlCommand.stop()

        if abs(marker_info.yaw_error_deg) > _ALIGN_YAW_THRESH_DEG:
            if marker_info.yaw_error_deg < 0:
                return ControlCommand.turn_left(SpeedLevel.VERY_SLOW)
            else:
                return ControlCommand.turn_right(SpeedLevel.VERY_SLOW)

        if abs(marker_info.distance_x_cm) > _ALIGN_LATERAL_THRESH_CM:
            if marker_info.distance_x_cm < 0:
                return ControlCommand.forward_left(SpeedLevel.VERY_SLOW)
            else:
                return ControlCommand.forward_right(SpeedLevel.VERY_SLOW)

        return ControlCommand.move_forward(SpeedLevel.VERY_SLOW)

    def check_transition(
        self,
        imu_data: Any,
        depth_features: Any,
        marker_info: Optional[MarkerInfo],
    ) -> Optional[StateType]:
        if self.is_lost_beyond_limit(miss_limit=3, log_tag="[DockingState]"):
            return StateType.RETRYING

        if marker_info is not None and marker_info.visible:
            if marker_info.distance_z_cm <= _DOCKED_DISTANCE_CM:
                logger.info("[DockingState] Docked → CHARGING")
                return StateType.CHARGING

        return None

    def exit(self) -> None:
        logger.info("[DockingState] Exiting DOCKING")
