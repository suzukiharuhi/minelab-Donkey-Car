from typing import Optional, List, Tuple
import math
import rospy
import numpy as np
import time

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.perception.compute_entry_point import EntryPointCalculator
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.turning.turn_controller import TurnController, TurnDecision
from pilot.app.turning.turn_controller import SmallAngleTurnController, SmallTurnDecision
from .config import ChargingConfig

class ApproachEntryPointState(BaseState):
    """
    充電ステーションへの接近エントリポイント状態
    マーカー3と4を使用して目標点P（中央法線方向）を計算し移動する
    """
    def __init__(self):
        super().__init__()
        self.cfg = ChargingConfig()
        self._phase: int = 1 # フェーズ管理: 1=θ算出, 2=IMU旋回, 3=前進
        self._target_yaw: Optional[float] = None # 目標yaw
        self.theta_cal = EntryPointCalculator()
        # 状態フラグ・カウンタ
        self._complete_turn: bool = False  # 旋回完了フラグ
        self._complete_entry: bool = False # エントリ完了フラグ
        self._miss_count: int = 0          # マーカーロストカウント（phase3で使用）
        self._phase_start_time = None
        self._phase3_started = False
        self._retry_flag = False

        self.ctrl: Optional[SmallAngleTurnController] = None
        self.start_flag = False


    def needs_pose_estimation(self) -> bool:
        """marker IDと距離情報が必要"""
        if self._phase == 1 or self._phase == 3:
            return True
        return False

    def enter(self):
        """状態開始"""
        rospy.loginfo("[ENTRY_P] Entered ----------")
        self._phase = 1
        self._controller = TurnController()
        self._started = False
        self._target_yaw = None
        self._complete_turn = False
        self._complete_entry = False
        self._miss_count = 0
        self._phase_start_time = None
        self._phase3_started = False
        self._retry_flag = False
        self.ctrl = SmallAngleTurnController(pulse_sec=0.1, wait_sec=0.1, tolerance_deg=0.5)
        self.start_flag = False


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
        theta = self.theta_cal.compute_theta(A, B) * (-1)
        print(f"A: {A}, B: {B}")
        print(f"draw_theta: {theta}")
        return theta

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """制御動作: フェーズ1→2→3を順に実行"""
        # Phase 1: マーカー3,4からθと目標yawを算出
        if self._phase == 1:
            # --- 追加：Phase1開始直後の1秒だけ停止 ---
            if self._phase_start_time is None:
                self._phase_start_time = time.time()   # ★ Phase1開始時刻を記録
                return ControlCommand.stop()            # ★ 最初の1回は停止
            # 1秒経過待ち
            if time.time() - self._phase_start_time < 1.0:
                return ControlCommand.stop()  
            
            m3 = self._get_marker(markers, 3)
            m4 = self._get_marker(markers, 4)
            if not (m3 is not None and m4 is not None):
                rospy.logwarn_throttle(1.0, "[approach_entry_point]  Markers 3 or 4 not found ----------------------")
                self._retry_flag = True
                return ControlCommand.stop()

            x3, z3 = m3.distance_xz
            x4, z4 = m4.distance_xz
            self._theta = self._compute_theta(np.array([x3, z3]), np.array([x4, z4]))
            # current_yaw = float(imu_data.euler_angles[2])  # 現在角度 current_yaw を取得
            # self._target_yaw = current_yaw + float(self._theta)  # 目標角度 target_yaw を計算
            rospy.loginfo(f"[ENTRY_P] theta={self._theta:.3f}°")
            # # 補正式による調整 --------------------------
            # if abs(self._theta) > 3: 
            #     if self._theta >= 0:
            #         self._theta_corr = self._theta - (0.472242*self._theta**2 + -5.813364*self._theta + 16.091139)
            #     else:
            #         self._theta_corr = self._theta - (-0.090549*self._theta**2 + -0.275565*self._theta + -3.046643)
            #     # self._theta_corr = self._theta - (0.146329*self._theta**2 + 0.174783*self._theta + -4.768208)
            #     # self._theta_corr = self._theta - (0.096495*self._theta**2 + 0.211969*self._theta + -4.026977)
            # else:
            #     self._theta_corr = self._theta
            # self._theta_corr = self._theta - (0.014922*self._theta**2 + 0.081912*self._theta + -0.212986) #廊下
            # self._theta = self._theta - (0.000390*self._theta**2 + -0.072934*self._theta + -0.236024)
            # ------------------------------------------
            # rospy.loginfo(f"[ENTRY_P] adjusted theta={self._theta:.3f}°, corrected theta={self._theta_corr:.3f}°")
            
            self._phase = 2
            self._phase_start_time = None
            return ControlCommand.no_operation()

        # Phase 2: IMU(yaw)を使って目標角度へ旋回
        if self._phase == 2:
            # if imu_data is None:
            #     return ControlCommand.no_operation()
            # current_yaw = float(imu_data.euler_angles[2])
            # """旋回制御"""
            # if current_yaw is None:
            #     rospy.logwarn_throttle(1.0, "[ENTRY_P] IMU yaw not available → NOP")
            #     return ControlCommand.no_operation()
            # if not self._started and self._controller is not None:
            #     self._controller.start(theta_deg=self._theta_corr, current_yaw_deg=current_yaw)
            #     self._started = True
            #     rospy.loginfo(f"[ROTATE_ENTRY_P] Start relative turn {self._theta:+.2f}° (initial={current_yaw:.2f}°)")
            #     return ControlCommand.no_operation()
            # if self._controller is None:
            #     return ControlCommand.no_operation()

            # decision: TurnDecision = self._controller.step(current_yaw_deg=current_yaw)
            # rospy.logdebug("[ENTRY_P] controller.step -> dir=%s slow=%s done=%s", str(decision.direction), str(decision.slow), str(decision.done))
            # if decision.done:
            #     self._phase = 3
            #     rospy.loginfo(f"[ENTRY_P] Completed.")
            #     return ControlCommand.stop()
            ## 速度レベル選択
            # speed = SpeedLevel.SLOW if decision.slow else SpeedLevel.MID
            # if decision.direction == 'left':
            #     return ControlCommand.turn_left(speed)
            # elif decision.direction == 'right':
            #     return ControlCommand.turn_right(speed)
            # return ControlCommand.no_operation()

            # 現状態情報
            now_time = rospy.get_time()
            current_yaw = float(imu_data.euler_angles[2]) # IMU から取得
            if not self.start_flag:
                target_yaw = current_yaw + self._theta  # 目標絶対角度
                self.ctrl.start(target_yaw_deg=target_yaw, current_yaw_deg=current_yaw, now_sec=now_time)
                self.start_flag = True
            elif self._phase == 3:
                return ControlCommand.stop()

            # ループ内
            decision = self.ctrl.step(now_sec=now_time, current_yaw_deg=current_yaw)

            if decision.done:
                cmd = ControlCommand.stop()
                self._phase = 3
            elif decision.direction == 'left':
                cmd = ControlCommand.turn_left(SpeedLevel.SLOW)
            elif decision.direction == 'right':
                cmd = ControlCommand.turn_right(SpeedLevel.SLOW)
            else:
                # phase == 'wait' など → STOP 維持
                cmd = ControlCommand.stop()
            print(f"cmd: {cmd}, error: {decision.error_deg}, phase: {decision.phase}")
            return cmd
            

        # Phase 3: 目標点Pまで前進（marker 0 が閾値以内になるまで）
        if self._phase == 3:
            # --- 追加：Phase1開始直後の1秒だけ停止 ---
            if self._phase_start_time is None:
                self._phase3_started = True
                self._phase_start_time = time.time()   # ★ Phase1開始時刻を記録
                return ControlCommand.stop()            # ★ 最初の1回は停止
            # 1秒経過待ち
            if time.time() - self._phase_start_time < 1.0:
                return ControlCommand.stop()  
            
            # yawが正しいか確認用 -----------------------------
            # rospy.sleep(2)
            rospy.loginfo_throttle(1.0, f"current_yaw: {float(imu_data.euler_angles[2])}")
            # -----------------------------------------------


            m0 = self._get_marker(markers, 0)
            if m0 is not None and m0.distance_xz is not None:
                _, z0 = m0.distance_xz
                if z0 <= self.cfg.approach_point_distance:
                    self._complete_entry = True
                    return ControlCommand.stop()
                return ControlCommand.move_forward(SpeedLevel.MID)
            # マーカー0が見えない間は一定時間までは前進、それ以上は停止
            self._miss_count += 1
            if self._miss_count <= self.cfg.miss_limit:
                return ControlCommand.move_forward(SpeedLevel.SLOW)
            print("[approach_entry_point]  Marker 0 lost, stopping")
            return ControlCommand.stop()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """次状態への遷移判定
        phase3における遷移判定

        遷移条件:
            - 閾値以内になったらADJUST_POSITIONへ遷移
            - マーカーが見えなくなり、miss_limitを超えたらRETRYINGへ遷移
        """
        if self._phase == 1 and self._retry_flag:
            return StateType.RETRYING

        if not self._phase3_started:
            return None
                
        if self._miss_count > self.cfg.miss_limit:
            return StateType.RETRYING

        m0 = self._get_marker(markers, 0)
        if m0 is not None and self._complete_entry:
            return StateType.ADJUST_POSITION
        return None

    def exit(self):
        """状態終了"""

        rospy.loginfo("[ENTRY_P] Exiting")
