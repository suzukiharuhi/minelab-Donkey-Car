import rospy
from typing import Optional, List

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData


class RetryingState(BaseState):
    """後退して再アプローチする状態（pitchベースの調整）
    - マーカー0のpitchを用いて正面（pitch≈0°）になるよう後進
    - pitchが負: 右斜め後ろ（backward_right）
    - pitchが正: 左斜め後ろ（backward_left）
    - |pitch| ≤ 5°: 直後進（move_backward）
    - zが100cmに達したら完了
    """

    def __init__(self):
        super().__init__()
        self.target: Optional[ArUcoMarker] = None
        self._miss_count: int = 0
        self._target_marker_id: int = 0
        self._back_target_cm: float = 100.0
        self._pitch_align_thr_deg: float = 5.0  # |pitch|がこの範囲なら直後進
        self._phase: str = 'align_back'  # 'align_back' | 'done'

    def needs_pose_estimation(self) -> bool:
        return True

    def enter(self):
        rospy.loginfo("[RETRY] Entered")
        self._miss_count = 0
        self._phase = 'align_back'
        self.target = None

    def _select_marker0(self, markers: List[ArUcoMarker]) -> Optional[ArUcoMarker]:
        for m in markers:
            if m.marker_id == self._target_marker_id and m.distance_xz is not None:
                return m
        return None

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        if self._phase == 'align_back':
            # マーカー0のpitchで方向を決定しながら後進
            self.target = self._select_marker0(markers) if markers else None
            if self.target is None or self.target.distance_xz is None or self.target.euler_angles is None:
                self._miss_count += 1
                rospy.loginfo_throttle(1.0, f"[RETRY] Marker0 lost or missing pose (miss={self._miss_count}) → BACK(MID)")
                return ControlCommand.move_backward(SpeedLevel.SLOW)

            _, pitch_deg, _ = self.target.euler_angles
            _, z_cm = self.target.distance_xz
            rospy.loginfo_throttle(1.0, f"[RETRY] z={z_cm:.1f}cm pitch={pitch_deg:.1f}°")

            # 完了条件: 既定の後退距離に到達
            if z_cm >= self._back_target_cm:
                self._phase = 'done'
                rospy.loginfo(f"[RETRY] Reached {self._back_target_cm:.0f}cm from marker0 → STOP")
                return ControlCommand.stop()

            # pitchで方向を決定（|pitch|<=閾値は直後進）
            thr = self._pitch_align_thr_deg
            if pitch_deg < -thr:
                return ControlCommand.backward_left(SpeedLevel.MID)
            if pitch_deg > thr:
                return ControlCommand.backward_right(SpeedLevel.MID)
            return ControlCommand.move_backward(SpeedLevel.MID)

        return ControlCommand.stop()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        if self._phase == 'align_back':
            # マーカーが無い場合の安全遷移は無し（短時間のロストを許容）。
            return None
        # done フェーズで次段へ
        if self._phase == 'done':
            return StateType.APPROACH_ENTRY_POINT
        return None

    def exit(self):
        rospy.loginfo("[RETRY] Exiting")
