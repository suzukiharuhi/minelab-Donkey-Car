from typing import Optional, List
import rospy
import time

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from .session_logger import get_logger
from .config import ChargingConfig

class AdjustPositionState(BaseState):
    """マーカー0（中央）を正面に調整する状態
    調整後DOCKING状態へ遷移する
    """
    def __init__(self):
        super().__init__()
        self.cfg = ChargingConfig()
        # 状態フラグ・カウンタ
        self._complete_adjust: bool = False # 調整完了フラグ
        self._miss_count: int = 0
        self._start_time: Optional[float] = None
        self._turning: bool = False
        self._turn_direction: Optional[str] = None
        self._turn_start_time: Optional[float] = None
        self._pause_until: Optional[float] = None

    def needs_pose_estimation(self) -> bool:
        """marker IDと距離情報が必要"""
        return True

    def enter(self):
        """状態開始"""
        rospy.loginfo("[ADJUST] Entered ----------")
        self._miss_count = 0
        self._start_time = None
        self._turning = False
        self._turn_direction = None
        self._turn_start_time = None
        self._pause_until = None

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
        """制御動作
        - マーカー0が見つかる場合
            - マーカー0のx位置が閾値以内なら停止、完了フラグを立てる
            - それ以外はマーカー0のx位置に応じて左右調整
        - いずれのマーカーも見つからない場合
            - miss_countをインクリメントし、miss_limit以内ならNOP、超えたら停止
        """
        m0 = self._get_marker(markers, 0)
        if m0 is not None and m0.distance_xz is not None:
            x0, _ = m0.distance_xz
            now = time.time()

            if self._turning:
                if self._turn_start_time is not None and (now - self._turn_start_time) < 0.1:
                    return ControlCommand.turn_right(SpeedLevel.VERY_SLOW) if self._turn_direction == 'right' else ControlCommand.turn_left(SpeedLevel.VERY_SLOW)
                self._turning = False
                self._turn_direction = None
                self._turn_start_time = None
                self._pause_until = now + 0.5
                return ControlCommand.stop()

            if self._pause_until is not None and now < self._pause_until:
                return ControlCommand.stop()

            if abs(x0) <= 1.0:
                self._complete_adjust = True
                print("[adjust_position]  complete_adjust set to True")
                return ControlCommand.stop()
            else:
                self._complete_adjust = False
                self._turning = True
                self._turn_direction = 'right' if x0 > 1.0 else 'left'
                self._turn_start_time = now
                return ControlCommand.turn_right(SpeedLevel.VERY_SLOW) if self._turn_direction == 'right' else ControlCommand.turn_left(SpeedLevel.VERY_SLOW)
            # elif not self._complete_adjust:
            #     # 超低速で微調整
            #     return ControlCommand.turn_right(SpeedLevel.VERY_SLOW) if x0 >= 0 else ControlCommand.turn_left(SpeedLevel.VERY_SLOW)
            # else:
            #     print("[adjust_position]  complete_adjust already True")
            #     return ControlCommand.stop()

        self._turning = False
        self._turn_direction = None
        self._turn_start_time = None
        self._pause_until = None
        return ControlCommand.stop()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """次状態への遷移判定
        遷移条件：
          - マーカー0が正面に調整できたらDOCKINGへ
          - マーカーが一定回数見つからなければRETRYINGへ
        """
        if self._complete_adjust:
            if self._start_time is None:
                self._start_time = time.time()
                return None
            # 1秒経過待ち
            if time.time() - self._start_time < 1.0:
                return None 

            m0 = self._get_marker(markers, 0)
            if m0 is not None and m0.distance_xz is not None and m0.euler_angles is not None:
                x0, _ = m0.distance_xz
                pitch= m0.euler_angles[1]
                if abs(x0) <= 1.0 and abs(pitch) <= 4.5:
                    print("[adjust_position]  transitioning to DOCKING")
                    return StateType.DOCKING
                else:
                    print(f"[adjust_position] not aligned: x0 = {x0}, pitch = {pitch}, retrying")
                    # record docking retry ---------------------
                    try:
                        get_logger().record_docking_failure()
                    except Exception:
                        rospy.logwarn("[adjust_position] failed to record docking retry")
                    # ---------------------------------------------
                    self._complete_adjust = False
                    self._turning = False
                    self._turn_direction = None
                    self._turn_start_time = None
                    self._pause_until = None
                    return StateType.RETRYING
        # elif self._miss_count > self.cfg.miss_limit:
        else:
            self._start_time = None
            if self._miss_count > 15:
                # --------------------------------
                try:
                    get_logger().record_docking_failure()
                except Exception:
                    rospy.logwarn("[adjust_position] failed to record docking retry (miss_count)")
                #-----------------------------------
                return StateType.RETRYING
        return None

    def exit(self):
        rospy.loginfo("[ADJUST] Exiting")
