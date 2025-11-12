from typing import Optional, List, Tuple
import math
import rospy
import numpy as np

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.turning.turn_controller import TurnController, TurnDecision
from .config import ChargingConfig
from .turn_utils import turn_towards


class ApproachEntryPointState(BaseState):
    """
    充電ステーションへの接近エントリポイント状態
    マーカー3と4を使用して目標点P（中央法線方向）を計算する
    """
    def __init__(self):
        super().__init__()
        self.cfg = ChargingConfig()
        self._phase: int = 1 # フェーズ管理: 1=θ算出, 2=IMU旋回, 3=前進
        self._target_yaw: Optional[float] = None # 目標yaw
        # 状態フラグ・カウンタ
        self._complete_turn: bool = False  # 旋回完了フラグ
        self._miss_count: int = 0          # マーカーロストカウント（phase3で使用）

    def needs_pose_estimation(self) -> bool:
        """marker IDと距離情報が必要"""
        return True

    def enter(self):
        """状態開始"""
        rospy.loginfo("[ENTRY_P] Entered ----------")
        self._phase = 1
        self._controller = TurnController()
        self._started = False
        self._target_yaw = None
        self._complete_turn = False
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

    def _compute_theta(self, A: Tuple[float, float], B: Tuple[float, float]) -> float:
        """目標点Pとその角度thetaを計算"""
        theta = draw_p.compute_theta(A, B) * (-1)
        print(f"A: {A}, B: {B}")
        print(f"draw_theta: {theta}")
        return theta

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """制御動作: フェーズ1→2→3を順に実行"""
        # Phase 1: マーカー3,4からθと目標yawを算出
        if self._phase == 1:
            m3 = self._get_marker(markers, 3)
            m4 = self._get_marker(markers, 4)
            if not (m3 is not None and m4 is not None):
                print("[approach_entry_point]  Markers 3 or 4 not found")
                return ControlCommand.stop()

            x3, z3 = m3.distance_xz
            x4, z4 = m4.distance_xz
            self._theta = self._compute_theta(np.array([x3, z3]), np.array([x4, z4]))
            # current_yaw = float(imu_data.euler_angles[2])  # 現在角度 current_yaw を取得
            # self._target_yaw = current_yaw + float(self._theta)  # 目標角度 target_yaw を計算
            rospy.loginfo(f"[ENTRY_P] theta={self._theta:.3f}°")
            self._phase = 2
            return ControlCommand.no_operation()

        # Phase 2: IMU(yaw)を使って目標角度へ旋回
        if self._phase == 2:
            if imu_data is None:
                return ControlCommand.no_operation()
            current_yaw = float(imu_data.euler_angles[2])
            """旋回制御"""
            if current_yaw is None:
                rospy.logwarn_throttle(1.0, "[ENTRY_P] IMU yaw not available → NOP")
                return ControlCommand.no_operation()
            if not self._started and self._controller is not None:
                self._controller.start(theta_deg=self._theta, current_yaw_deg=current_yaw)
                self._started = True
                rospy.loginfo(f"[ROTATE_ENTRY_P] Start relative turn {self._theta:+.2f}° (initial={current_yaw:.2f}°)")
                return ControlCommand.no_operation()
            if self._controller is None:
                return ControlCommand.no_operation()

            decision: TurnDecision = self._controller.step(current_yaw_deg=current_yaw)
            if decision.done:
                rospy.loginfo(f"[ENTRY_P] Completed.")
                return ControlCommand.stop()
            
            # 速度レベル選択
            speed = SpeedLevel.SLOW if decision.slow else SpeedLevel.MID
            if decision.direction == 'left':
                return ControlCommand.turn_left(speed)
            elif decision.direction == 'right':
                return ControlCommand.turn_right(speed)
            return ControlCommand.no_operation()

        # Phase 3: 目標点Pまで前進（marker 0 が閾値以内になるまで）
        if self._phase == 3:
            m0 = self._get_marker(markers, 0)
            if m0 is not None and m0.distance_xz is not None:
                _, z0 = m0.distance_xz
                if z0 <= self.cfg.approach_point_distance:
                    return ControlCommand.stop()
                return ControlCommand.move_forward(SpeedLevel.SLOW)
            # マーカー0が見えない間は一定時間までは前進、それ以上は停止
            self._miss_count += 1
            if self._miss_count <= self.cfg.miss_limit:
                return ControlCommand.move_forward(SpeedLevel.SLOW)
            return ControlCommand.stop()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """次状態への遷移判定
        phase3における遷移判定

        遷移条件:
            - 閾値以内になったらADJUST_POSITIONへ遷移
            - マーカー3または4が見えなくなり、miss_limitを超えたらRETRYINGへ遷移
        """
        if self._phase < 3:
            return None
        
        if self._miss_count > self.cfg.miss_limit:
            return StateType.RETRYING
        m0 = self._get_marker(markers, 0)
        if m0 is not None and m0.distance_xz is not None:
            _, z0 = m0.distance_xz
            if z0 <= self.cfg.approach_point_distance:
                return StateType.ADJUST_POSITION
        return None

    def exit(self):
        """状態終了"""
        rospy.loginfo("[ENTRY_P] Exiting")
