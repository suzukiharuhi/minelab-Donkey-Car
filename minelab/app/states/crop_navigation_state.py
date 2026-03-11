"""Crop-navigation state – uses DepthFeatures / FTG-i output to steer (ROS-free)."""
from __future__ import annotations

import logging
import math
from typing import Any, Optional

from minelab.app.core.commands import ControlCommand, SpeedLevel
from minelab.app.states.base_state import BaseState
from minelab.app.states.state_type import StateType
from minelab.interfaces.data_models import DepthFeatures, MarkerInfo

logger = logging.getLogger(__name__)

# Steering-angle thresholds [rad]
_STEER_THRESH_SMALL = math.radians(5.0)
_STEER_THRESH_LARGE = math.radians(15.0)

# Marker IDs that trigger a state transition
_MARKER_ID_TURN = 10   # e.g. end-of-row marker


class CropNavigationState(BaseState):
    """Drive between crop rows using the FTG-i gap result.

    The state receives a ``DepthFeatures`` object (published by
    ``depth_feature_node``) and chooses a directional command based on
    ``gap_angle_rad``.
    """

    def enter(self) -> None:
        logger.info("[CropNavigationState] Entered CROP_NAVIGATION")
        self.reset_miss_count()

    def execute(
        self,
        imu_data: Any,
        depth_features: Optional[DepthFeatures],
        marker_info: Optional[MarkerInfo],
    ) -> ControlCommand:
        if depth_features is None or not depth_features.range_array:
            self.inc_miss_count()
            return self.handle_transient_loss(
                ControlCommand.move_forward(SpeedLevel.SLOW),
                log_tag="[CropNavigationState]",
            )

        self.reset_miss_count()
        angle = depth_features.gap_angle_rad

        if abs(angle) <= _STEER_THRESH_SMALL:
            return ControlCommand.move_forward(SpeedLevel.MID)
        elif angle < -_STEER_THRESH_SMALL:
            speed = SpeedLevel.SLOW if abs(angle) > _STEER_THRESH_LARGE else SpeedLevel.MID
            return ControlCommand.forward_left(speed)
        else:
            speed = SpeedLevel.SLOW if abs(angle) > _STEER_THRESH_LARGE else SpeedLevel.MID
            return ControlCommand.forward_right(speed)

    def check_transition(
        self,
        imu_data: Any,
        depth_features: Optional[DepthFeatures],
        marker_info: Optional[MarkerInfo],
    ) -> Optional[StateType]:
        if self.is_lost_beyond_limit(log_tag="[CropNavigationState]"):
            return StateType.IDLE

        if marker_info is not None and marker_info.visible:
            if marker_info.marker_id == _MARKER_ID_TURN:
                logger.info(
                    "[CropNavigationState] Turn marker %d detected → APPROACH_MARKER",
                    _MARKER_ID_TURN,
                )
                return StateType.APPROACH_MARKER

        return None

    def exit(self) -> None:
        logger.info("[CropNavigationState] Exiting CROP_NAVIGATION")
