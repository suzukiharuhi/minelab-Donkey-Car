import rospy
from typing import Optional, List

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from .config import ChargingConfig

class RetryingState(BaseState):
    """Back off and then return to approach entry point."""
    def __init__(self):
        super().__init__()
        self.cfg = ChargingConfig()
        self._ticks: int = 0

    def needs_pose_estimation(self) -> bool:
        return False

    def enter(self):
        rospy.loginfo("[RETRY] Entered")
        self._ticks = 0

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        self._ticks += 1
        if self._ticks <= self.cfg.retry_back_ticks:
            return ControlCommand.move_backward(SpeedLevel.SLOW)
        return ControlCommand.stop()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        if self._ticks > self.cfg.retry_back_ticks:
            return StateType.APPROACH_ENTRY_POINT
        return None

    def exit(self):
        rospy.loginfo("[RETRY] Exiting")
