"""180度旋回状態

order.py の turn_theta() ロジックを TurnController に抽象化し利用する。
State は IMU の yaw(deg) を取り出し controller.step() を呼び、返却を
ControlCommand にマッピングする責務のみを持つ。
"""
from typing import Optional, List
import rospy
import os

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.turning.turn_controller import PidTurnController, PidTurnDecision


def _get_yaw_deg(imu: Optional[IMUData]) -> Optional[float]:
    """IMUData から yaw(deg) を抽出。 None 可。"""
    if imu is None or imu.euler_angles is None:
        return None
    try:
        # print(f"yaw: {imu.euler_angles[2]}")
        return float(imu.euler_angles[2])
    except Exception:
        return None

class Rotate180State(BaseState):
    """その場で 180 度旋回する状態"""
    def __init__(self):
        super().__init__()
        self._controller: Optional[PidTurnController] = None
        self._started: bool = False
        self.theta_deg: float = 180.0

    def needs_pose_estimation(self) -> bool:
        """marker IDのみでOK"""
        return False

    def enter(self):
        """状態開始"""
        rospy.loginfo("[ROTATE_180] Entered ---------------")
        self._controller = PidTurnController()
        self._started = False
        self.theta_deg: float = 180.0

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        yaw_deg = _get_yaw_deg(imu_data)
        now = rospy.get_time()
        if yaw_deg is None:
            rospy.logwarn_throttle(1.0, "[ROTATE_180] IMU yaw not available → NOP")
            return ControlCommand.no_operation()
        # 初回呼び出しで開始
        if not self._started and self._controller is not None:
            self._controller.start(theta_deg=self.theta_deg, current_yaw=yaw_deg, now_sec=now)
            self._started = True
            rospy.loginfo(f"[ROTATE_180] Start relative turn 180° (initial={yaw_deg:.2f}°)")
            return ControlCommand.no_operation()
        if self._controller is None:
            return ControlCommand.no_operation()
        # PID 制御ステップ
        decision: PidTurnDecision = self._controller.step(now_sec=now, current_yaw=yaw_deg)
        print(f"[ROTATE_180] PID decision: done={decision.done}, direction={decision.direction}, speed_norm={decision.speed_norm}, error_deg={decision.error_deg:.2f}")
        if decision.done:
            rospy.loginfo(f"[ROTATE_180] PID turn completed (error={decision.error_deg:.2f}°)")
            return ControlCommand.stop()
        elif decision.direction is None:
            return ControlCommand.stop()
        pwm = float(decision.speed_norm)

        if decision.direction == 'left':
            return ControlCommand.turn_left(pwm)
        elif decision.direction == 'right':
            return ControlCommand.turn_right(pwm)
        return ControlCommand.stop()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        # 旋回完了で次の列走行へ遷移 (要件に応じて変更)。
        if self._controller and self._controller.finished():
            os.exit(0)  # テストのため終了
            return StateType.IDLE
        return None

    def exit(self):
        rospy.loginfo("[ROTATE_180] Exiting")