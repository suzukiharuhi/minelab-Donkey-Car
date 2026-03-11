"""Approach-marker state – follow a detected ArUco marker (ROS-free)."""
from __future__ import annotations

import logging
from typing import Any, Optional

from minelab.app.core.commands import ControlCommand, SpeedLevel
from minelab.app.states.base_state import BaseState
from minelab.app.states.state_type import StateType
from minelab.interfaces.data_models import MarkerInfo

logger = logging.getLogger(__name__)

_TARGET_MARKER_ID = 10          # end-of-row marker
_APPROACH_DISTANCE_CM = 40.0   # stop this close to the marker
_LATERAL_THRESH_CM = 5.0       # lateral error threshold for straight drive


class ApproachMarkerState(BaseState):
    """Drive towards a target ArUco marker until within docking range."""

    def enter(self) -> None:
        logger.info("[ApproachMarkerState] Entered APPROACH_MARKER")
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
                ControlCommand.move_forward(SpeedLevel.SLOW),
                log_tag="[ApproachMarkerState]",
            )

        self.reset_miss_count()

        if marker_info.distance_z_cm <= _APPROACH_DISTANCE_CM:
            return ControlCommand.stop()

        if abs(marker_info.distance_x_cm) > _LATERAL_THRESH_CM:
            if marker_info.distance_x_cm < 0:
                return ControlCommand.forward_left(SpeedLevel.SLOW)
            else:
                return ControlCommand.forward_right(SpeedLevel.SLOW)

        return ControlCommand.move_forward(SpeedLevel.SLOW)

    def check_transition(
        self,
        imu_data: Any,
        depth_features: Any,
        marker_info: Optional[MarkerInfo],
    ) -> Optional[StateType]:
        if self.is_lost_beyond_limit(log_tag="[ApproachMarkerState]"):
            return StateType.IDLE

        if marker_info is not None and marker_info.visible:
            if marker_info.distance_z_cm <= _APPROACH_DISTANCE_CM:
                logger.info("[ApproachMarkerState] Marker reached → ROTATE180")
                return StateType.ROTATE180

        return None

    def exit(self) -> None:
        logger.info("[ApproachMarkerState] Exiting APPROACH_MARKER")
