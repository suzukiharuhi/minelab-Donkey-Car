from typing import Optional, List
import rospy
import time

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from .config import ChargingConfig
from .session_logger import get_logger

class DockingState(BaseState):
    """Move forward until marker 0 is within docking threshold."""
    def __init__(self):
        super().__init__()
        self.cfg = ChargingConfig()
        self._miss_count: int = 0
        # 状態フラグ・カウンタ
        self._complete_docking: bool = False # ドッキング完了フラグ
        self._failure_logged: bool = False
        # self._start_time = None

    def needs_pose_estimation(self) -> bool:
        """marker IDと距離情報が必要"""
        return True

    def enter(self):
        """状態開始"""
        rospy.loginfo("[DOCKING] Entered ----------")
        self._miss_count = 0
        self._failure_logged = False
        try:
            get_logger().record_docking_attempt()
        except Exception as exc:
            rospy.logwarn(f"[DOCKING] logger record_docking_attempt failed: {exc}")
        # self._start_time = None
    
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
        # if self._start_time is None:
        #     self._start_time = time.time()
        #     return ControlCommand.stop()
        # # 1秒経過待ち
        # if time.time() - self._start_time < 1.0:
        #     return ControlCommand.stop()  
            
        m0 = self._get_marker(markers, 0)
        if m0 is None:
            self._miss_count += 1
            if self._miss_count <= self.cfg.miss_limit:
                # return ControlCommand.no_operation()
                return ControlCommand.stop()
            return ControlCommand.stop()

        x0, z0 = m0.distance_xz
        pitch= m0.euler_angles[1]
        ######pitchの調整とxの調整を両方やるのが最善
        #####33
        #########
        #####
        if z0 <= self.cfg.docking_threshold:
            self._complete_docking = True
            return ControlCommand.stop()
        if x0 > 1.0:
            return ControlCommand.forward_right(SpeedLevel.SLOW)
        elif x0 < -1.0:
            return ControlCommand.forward_left(SpeedLevel.SLOW)
        return ControlCommand.move_forward(SpeedLevel.SLOW)

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """次状態への遷移判定
        遷移条件：
            - マーカー0がドッキング閾値以内に入ったらCHARGINGへ
            - マーカーが一定回数見つからなければRETRYINGへ
        """
        m0 = self._get_marker(markers, 0)
        if m0 is not None and self._complete_docking:
            self._complete_docking = False
            try:
                get_logger().record_docking_success()
            except Exception as exc:
                rospy.logwarn(f"[DOCKING] logger record_docking_success failed: {exc}")
            return StateType.CHARGING
        if self._miss_count > self.cfg.miss_limit:
            if not self._failure_logged:
                try:
                    get_logger().record_docking_failure()
                except Exception as exc:
                    rospy.logwarn(f"[DOCKING] logger record_docking_failure failed: {exc}")
                self._failure_logged = True
            return StateType.RETRYING
        return None

    def exit(self):
        rospy.loginfo("[DOCKING] Exiting")
