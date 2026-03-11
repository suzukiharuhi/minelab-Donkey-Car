import rospy
from typing import Optional, List

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from .session_logger import get_logger

class AutoChargingState(BaseState):
    """Entry state that routes into the charging"""
    def __init__(self):
        super().__init__()

    def needs_pose_estimation(self) -> bool:
        return True

    def enter(self):
        rospy.loginfo("[AUTO_CHG] Entered")
        try:
            get_logger().start_session()
            rospy.loginfo("[AUTO_CHG] Session logging started")
        except Exception as exc:
            rospy.logwarn(f"[AUTO_CHG] Failed to start session logger: {exc}")

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        return ControlCommand.no_operation()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        return StateType.APPROACH_ENTRY_POINT

    def exit(self):
        rospy.loginfo("[AUTO_CHG] Exiting")
