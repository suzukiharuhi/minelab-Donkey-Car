"""アイドル状態"""
from typing import Optional, List
import rospy

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.core.commands import ControlCommand, SpeedLevel

class IdleState(BaseState):
    """初期待機状態、マーカー探索"""
    
    def __init__(self):
        super().__init__()
    
    def needs_pose_estimation(self) -> bool:
        """marker IDのみでOK（高速）"""
        return False
    
    def enter(self):
        """状態開始"""
        rospy.loginfo("[IDLE] Entered ---------------")

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """目標マーカー検出で APPROACH_MARKER へ遷移"""
        if not markers:
            return None
        target_marker_ids = {20, 21}
        for m in markers:
            if m.marker_id in target_marker_ids:
                rospy.loginfo(f"[IDLE] Marker {m.marker_id} detected → FOLLOW_ROW_CENTER")
                return StateType.FOLLOW_ROW_CENTER

        rospy.loginfo(f"[IDLE] Marker detected → APPROACH_MARKER")
        return StateType.APPROACH_MARKER

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """状態動作: マーカー探索．未検出なら前進，検出したら状態遷移"""
        if len(markers) == 0:
            #
            # ほんとうなら
            # 障害物検知をおりまぜながら
            # 前進するべし
            #
            rospy.loginfo_throttle(1.0, "[IDLE] No markers → move_forward (marker search)")
            return ControlCommand.move_forward(speed=SpeedLevel.MID)
        else:
            rospy.loginfo_throttle(1.0, f"[IDLE] Marker detected ({len(markers)}) → transition")
            return ControlCommand.no_operation()

    def exit(self):
        """状態終了"""
        rospy.loginfo("[IDLE] Exiting")
