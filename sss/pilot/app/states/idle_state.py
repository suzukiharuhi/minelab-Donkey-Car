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
        self._row_ids = {20, 21}  # 植物列走行のためのマーカーID
        self._target_ids = {0, 10} # その他マーカーID
    
    def needs_pose_estimation(self) -> bool:
        """marker IDのみでOK（高速）"""
        return False
    
    def enter(self):
        """状態開始"""
        rospy.loginfo("[IDLE] Entered ---------------")

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """状態遷移判定
        遷移条件:
        - 植物列走行用マーカー(20,21)検出で FOLLOW_ROW_CENTER へ遷移
        - その他マーカー検出で APPROACH_MARKER へ遷移
        """
        if not markers:
            return None
        for m in markers:
            if m.marker_id in self._row_ids:
                rospy.loginfo(f"[IDLE] Marker {m.marker_id} detected → FOLLOW_ROW_CENTER")
                return StateType.FOLLOW_ROW_CENTER
        for m in markers:
            if m.marker_id in self._target_ids:
                rospy.loginfo(f"[IDLE] Marker {m.marker_id} detected → APPROACH_MARKER")
                return StateType.APPROACH_MARKER

        rospy.loginfo(f"[IDLE] No target markers detected → remain IDLE")
        return None

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """
        状態動作:
        - マーカー未検出時は前進してマーカー探索
        - マーカー検出時は停止
        """
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
# """アイドル状態"""
# from typing import Optional, List
# import rospy
# import os

# from pilot.app.states.base_state import BaseState
# from pilot.app.states.state_type import StateType
# from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
# from pilot.app.core.commands import ControlCommand, SpeedLevel
# from pilot.app.turning.turn_controller import SmallAngleTurnController, SmallTurnDecision
# from pilot.app.core.commands import ControlCommand, SpeedLevel
# def _get_yaw_deg(imu: Optional[IMUData]) -> Optional[float]:
#     """IMUData から yaw(deg) を抽出。 None 可。"""
#     if imu is None or imu.euler_angles is None:
#         return None
#     try:
#         # print(f"yaw: {imu.euler_angles[2]}")
#         return float(imu.euler_angles[2]) # yawを返す
#     except Exception:
#         return None
# class IdleState(BaseState):
#     """初期待機状態、マーカー探索"""
    
#     def __init__(self):
#         super().__init__()
#         self._row_ids = {20, 21}  # 植物列走行のためのマーカーID
#         self._target_ids = {0, 10} # その他マーカーID
#         self.ctrl: Optional[SmallAngleTurnController] = None
#         self.start_flag = False
#         self.finisi_flag = False
    
#     def needs_pose_estimation(self) -> bool:
#         """marker IDのみでOK（高速）"""
#         return True
    
#     def enter(self):
#         """状態開始"""
#         rospy.loginfo("[IDLE] Entered ---------------")
#         self.ctrl = SmallAngleTurnController(pulse_sec=0.1, wait_sec=0.1, tolerance_deg=0.5)
#         self.start_flag = False
#         self.finisi_flag = False

#     def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
#         return None

#     def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
#         # 現状態情報
#         now_time = rospy.get_time()
#         current_yaw = _get_yaw_deg(imu_data) # IMU から取得
#         if not self.start_flag:
#             target_yaw = current_yaw - 7.0  # 目標絶対角度
#             self.ctrl.start(target_yaw_deg=target_yaw, current_yaw_deg=current_yaw, now_sec=now_time)
#             self.start_flag = True
#         elif self.finisi_flag:
#             return ControlCommand.stop()

#         # ループ内
#         decision = self.ctrl.step(now_sec=now_time, current_yaw_deg=current_yaw)

#         if decision.done:
#             cmd = ControlCommand.stop()
#             self.finisi_flag = True
#         elif decision.direction == 'left':
#             cmd = ControlCommand.turn_left(SpeedLevel.SLOW)
#         elif decision.direction == 'right':
#             cmd = ControlCommand.turn_right(SpeedLevel.SLOW)
#         else:
#             # phase == 'wait' など → STOP 維持
#             cmd = ControlCommand.stop()
#         print(f"cmd: {cmd}, error: {decision.error_deg}, phase: {decision.phase}")
#         return cmd
    
#     def exit(self):
#         """状態終了"""
#         rospy.loginfo("[IDLE] Exiting")
