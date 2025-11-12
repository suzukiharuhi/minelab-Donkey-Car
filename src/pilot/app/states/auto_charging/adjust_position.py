from typing import Optional, List
import rospy

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from .config import ChargingConfig

class AdjustPositionState(BaseState):
    """マーカー0（中央）を正面に調整する状態
    調整後，DOCKING状態へ遷移する
    """
    def __init__(self):
        super().__init__()
        self.cfg = ChargingConfig()
        # 状態フラグ・カウンタ
        self._complete_adjust: bool = False # 調整完了フラグ
        self._miss_count: int = 0

    def needs_pose_estimation(self) -> bool:
        """marker IDと距離情報が必要"""
        return True

    def enter(self):
        """状態開始"""
        rospy.loginfo("[ADJUST] Entered ----------")
        self._miss_count = 0

    def _get_marker(self, markers: List[ArUcoMarker], marker_id: int) -> Optional[ArUcoMarker]:
        """指定IDのマーカーオブジェクトを取得
        Returns:
            Optional[ArUcoMarker]: 指定IDのマーカーオブジェクト（存在しない場合はNone）
        """
        for m in markers:
            if m.marker_id == marker_id:
                return m
        return None

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """制御動作"""
        m0 = self._get_marker(markers, 0)
        if m0 is not None and m0.distance_xz is not None:
            x0, _ = m0.distance_xz
            if abs(x0) <= 1.0:
                self._complete_adjust = True
                return ControlCommand.stop()
            return ControlCommand.turn_right(SpeedLevel.SLOW) if x0 >= 0 else ControlCommand.turn_left(SpeedLevel.SLOW)

        # マーカー0が見つからない場合、マーカー3と4の位置で方向を決定
        m3 = self._get_marker(markers, 3)
        m4 = self._get_marker(markers, 4)
        if (m3 is not None and m3.distance_xz is not None) and (m4 is not None and m4.distance_xz is not None):
            x3, _ = m3.distance_xz
            x4, _ = m4.distance_xz
            return ControlCommand.turn_right(SpeedLevel.SLOW) if abs(x3) < abs(x4) else ControlCommand.turn_left(SpeedLevel.SLOW)

        self._miss_count += 1
        if self._miss_count <= self.cfg.miss_limit:
            return ControlCommand.no_operation()
        return ControlCommand.stop()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """次状態への遷移判定
        遷移条件：
          - マーカー0が正面に調整できたらDOCKINGへ
          - マーカーが一定回数見つからなければRETRYINGへ
        """
        if self._complete_adjust:
            m0 = self._get_marker(markers, 0)
            if m0 is not None and m0.distance_xz is not None and m0.euler_angles is not None:
                x0, _ = m0.distance_xz
                _, pitch, _ = m0.euler_angles
                if abs(x0) <= 1.0 and abs(pitch) <= 1.5:
                    return StateType.DOCKING
                else:
                    return StateType.RETRYING
        elif self._miss_count > self.cfg.miss_limit:
            return StateType.RETRYING
        return None

    def exit(self):
        rospy.loginfo("[ADJUST] Exiting")
