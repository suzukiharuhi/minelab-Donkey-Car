"""充電ステーション離脱状態 (Withdrawal)

要件:
- マーカー0を見ながら x が -5~5cm に入るよう調整しつつバック
- マーカー0との z 距離が 60cm 到達で 180度旋回へ移行
- 180度旋回完了で IDLE に遷移

approach_marker と rotate180 のロジックを参考に構成。
"""
from dataclasses import dataclass, field
from typing import Optional, List, Dict
import rospy
import time

from pilot.app.states.base_state import BaseState
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.turning.turn_controller import TurnController, TurnDecision


@dataclass
class WithdrawalConfig:
	target_marker_id: int = 0
	align_thr_cm: float = 7.0          # x の許容範囲
	turn_trigger_dist_cm: float = 60.0 # z がこの距離に到達したら旋回開始
	miss_limit: int = 5                # ロスト許容


def _get_yaw_deg(imu: Optional[IMUData]) -> Optional[float]:
	if imu is None or imu.euler_angles is None:
		return None
	try:
		print(f"yaw: {imu.euler_angles[2]}")
		return float(imu.euler_angles[2])
	except Exception:
		return None


class WithdrawalState(BaseState):
	"""充電ステーションから離脱するための状態

	フェーズ:
	1) Align&Back: マーカー0の x を合わせつつ後退
	2) Turn180: z<=60cm を満たしたら 180 度旋回
	3) Done: 旋回完了で IDLE へ
	"""

	def __init__(self):
		super().__init__()
		self.config = WithdrawalConfig()
		self._miss_count: int = 0
		self._phase: str = 'align_back'  # 'align_back' | 'turn' | 'done'
		self._phase_turn_started = None
		self._controller: Optional[TurnController] = None
		self.theta_deg: float = -170.0
		self._turn_started: bool = False
		self.target: Optional[ArUcoMarker] = None

	def needs_pose_estimation(self) -> bool:
		"""marker IDと距離情報が必要"""
		return True

	def enter(self):
		rospy.loginfo("[WITHDRAWAL] Entered ---------------")
		self._controller = None
		self._miss_count = 0
		self._phase = 'align_back'
		self._phase_turn_started = None
		self._turn_started = False
		self.target = None

	def _select_marker0(self, markers: List[ArUcoMarker]) -> Optional[ArUcoMarker]:
		for m in markers:
			if m.marker_id == self.config.target_marker_id and m.distance_xz is not None:
				return m
		return None

	def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
		# フェーズごとの制御
		if self._phase == 'align_back':
			# マーカー0を取得
			self.target = self._select_marker0(markers) if markers else None

			if self.target is None or self.target.distance_xz is None:
				# self._miss_count += 1
				# if self._miss_count <= self.config.miss_limit:
				# 	rospy.loginfo_throttle(1.0, f"[WITHDRAWAL] Marker0 temporarily lost (miss={self._miss_count}/{self.config.miss_limit}) → BACK(MID)")
				# 	return ControlCommand.move_backward(SpeedLevel.MID)
				rospy.loginfo("[WITHDRAWAL] Marker0 lost beyond limit → BACK")
				return ControlCommand.move_backward(SpeedLevel.SLOW)

			self._miss_count = 0
			x_cm, z_cm = self.target.distance_xz
			rospy.loginfo_throttle(1.0, f"[WITHDRAWAL] x={x_cm:.1f}cm z={z_cm:.1f}cm")

			# 旋回トリガー判定: z が 60cm 以上になったら旋回へ
			if z_cm > self.config.turn_trigger_dist_cm:
				self._phase = 'turn'
				return ControlCommand.stop()

			# x 整列しつつバック
			thr = self.config.align_thr_cm
			if x_cm >= thr:
				rospy.loginfo_throttle(1.0, f"[WITHDRAWAL] x={x_cm:.1f}cm → BACK_RIGHT(MID)")
				return ControlCommand.backward_right(SpeedLevel.MID)
			if x_cm <= -thr:
				rospy.loginfo_throttle(1.0, f"[WITHDRAWAL] x={x_cm:.1f}cm → BACK_LEFT(MID)")
				return ControlCommand.backward_left(SpeedLevel.MID)
			rospy.loginfo_throttle(1.0, "[WITHDRAWAL] x aligned → BACK(MID)")
			return ControlCommand.move_backward(SpeedLevel.MID)

		elif self._phase == 'turn':
			# --- 追加：'turn'開始直後の1秒だけ停止 ---
			if self._phase_turn_started is None:
				self._phase_turn_started = time.time()   # ★ Phase1開始時刻を記録
				return ControlCommand.stop()            # ★ 最初の1回は停止
            # 1秒経過待ち
			if time.time() - self._phase_turn_started < 1.0:
				return ControlCommand.stop()  

			if not self._turn_started:
				self._controller = TurnController()
				self._theta = self.theta_deg - (-0.000112*self.theta_deg**2 + 0.020123*self.theta_deg + -3.622113)
				print("[WITHDRAWAL] stopping before turn")
				rospy.sleep(1.0)  # 後退停止待ち
			# 180度旋回制御
			yaw_deg = _get_yaw_deg(imu_data)
			if yaw_deg is None:
				rospy.logwarn_throttle(1.0, "[WITHDRAWAL] IMU yaw not available → NOP")
				return ControlCommand.no_operation()

			if not self._turn_started and self._controller is not None:
				self._controller.start(theta_deg=self._theta, current_yaw_deg=yaw_deg)
				self._turn_started = True
				rospy.loginfo(f"[WITHDRAWAL] Start relative turn 180° (initial={yaw_deg:.2f}°)")
				return ControlCommand.no_operation()

			decision: TurnDecision = self._controller.step(current_yaw_deg=yaw_deg)
			if decision.done:
				rospy.loginfo(f"[WITHDRAWAL] Turn completed diff={decision.diff_deg:.2f} remaining={decision.remaining_deg:.2f}")
				self._phase = 'done'
				return ControlCommand.stop()

			speed = SpeedLevel.SLOW if decision.slow else SpeedLevel.MID
			if decision.direction == 'left':
				# print("[WITHDRAWAL] turning left")
				return ControlCommand.turn_left(speed)
			elif decision.direction == 'right':
				return ControlCommand.turn_right(speed)
			return ControlCommand.no_operation()

		# done フェーズ: 停止維持
		return ControlCommand.stop()

	def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
		# ロストによる安全遷移（align_back フェーズのみ）
		if self._phase == 'align_back' and (self.target is None):
			if self._miss_count > self.config.miss_limit:
				rospy.loginfo("[WITHDRAWAL] Marker0 lost beyond limit → transition to IDLE")
				return StateType.IDLE
			return None

		# 旋回完了で IDLE に遷移
		if self._phase == 'done':
			return StateType.IDLE

		# 旋回中に controller が finished を返した場合も遷移
		if self._phase == 'turn' and self._controller and self._controller.finished():
			return StateType.IDLE

		return None

	def exit(self):
		rospy.loginfo("[WITHDRAWAL] Exiting")

