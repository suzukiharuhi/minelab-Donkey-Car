import rospy
from typing import Optional, List

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData

class ChargingState(BaseState):
    """Charging confirmation placeholder: keep stopped."""
    def __init__(self):
        super().__init__()
        self._ticks: int = 0

    def needs_pose_estimation(self) -> bool:
        return False

    def enter(self):
        rospy.loginfo("[CHARGING] Entered (holding)")
        self._ticks = 0

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        self._ticks += 1
        return ControlCommand.stop()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        # In real system, confirm via sensor; here we just hold position
        return None

    def exit(self):
        rospy.loginfo("[CHARGING] Exiting")
