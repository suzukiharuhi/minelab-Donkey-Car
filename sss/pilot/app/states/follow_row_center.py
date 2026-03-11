from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple
import rospy
import numpy as np
import time as _time

from pilot.app.states.base_state import BaseState
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.core.step_logger import get_state_logger

@dataclass
class FollowRowCenterConfig:
    """
    作物列走行の設定値
    - left_marker_id: 左列を示すArUcoマーカーID
    - right_marker_id: 右列を示すArUcoマーカーID
    - max_dist_cm: 判定対象とする最大距離[cm]（この距離以内のマーカーのみ使用）
    - center_offset_cm: 片側のみ検出時に仮の中心を置くためのオフセット量[cm]
    - miss_limit: ロスト許容フレーム数
    - specific_markers: 特殊マーカーID -> 遷移先StateType の対応
    """
    left_marker_id: int = 20
    right_marker_id: int = 21
    max_dist_cm: float = 400.0
    center_offset_cm: float = 60.0
    miss_limit: int = 5
    specific_markers: List[int] = field(default_factory=lambda: [0, 10])

class FollowRowCenterState(BaseState):
    def __init__(self):
        super().__init__()
        self.config = FollowRowCenterConfig()
        self._miss_flag = False # 一時ロストフラグ

    def needs_pose_estimation(self) -> bool:
        """marker IDと距離情報が必要"""
        return True

    def enter(self):
        """状態開始"""
        rospy.loginfo("[FOLLOW_ROW_CENTER] Entered ----------")
        self.reset_miss_count()
        self._miss_flag = False

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """
        状態動作: 
        - マーカー検出無し → 停止
        - マーカー範囲外 → 停止
        - 作物列マーカー未検出 → ロストカウント増加 → 低速前進/停止を共通処理に委譲
        - 作物列マーカー検出 → 列中心xを推定 → 制御命令生成
        """
        center_x: Optional[float] = None

        if not markers:
            # マーカー未検出
            rospy.logwarn_throttle(1.0, "[FOLLOW_ROW_CENTER] No markers detected 	 STOP")
            cmd = ControlCommand.stop()
            self._log_step(imu_data, markers, center_x, cmd)
            return cmd

        markers_in_range = self._filter_markers_in_range(markers)
        if not markers_in_range:
            # マーカー範囲外
            rospy.logwarn_throttle(1.0, "[FOLLOW_ROW_CENTER] No markers in range 	 STOP")
            cmd = ControlCommand.stop()
            self._log_step(imu_data, markers, center_x, cmd)
            return cmd

        left_markers, right_markers = self._split_left_right(markers_in_range)
        if not left_markers and not right_markers:
            # 作物列マーカー未検出
            self.inc_miss_count() # ロストカウント増加
            cmd = self.handle_transient_loss(
                recover_command=ControlCommand.move_forward(SpeedLevel.SLOW),
                miss_limit=self.config.miss_limit,
                log_tag="[FOLLOW_ROW_CENTER]",
            )
            self._log_step(imu_data, markers, center_x, cmd)
            return cmd

        center_x = self._compute_center_x(left_markers, right_markers)
        cmd = self._decide_motion(center_x)
        self._log_step(imu_data, markers, center_x, cmd)
        return cmd
    
    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker],) -> Optional[StateType]:
        """状態遷移判定
        遷移条件：
        - マーカー未検出 or ロスト許容超過 → IDLEへ
        - 作物列マーカー検出 or ロスト許容 → None（継続）
        """
        left_markers, right_markers = self._split_left_right(markers)
        if not left_markers and not right_markers:
            if self.is_lost_beyond_limit(
                miss_limit=self.config.miss_limit,
                log_tag="[FOLLOW_ROW_CENTER] Marker",
            ):
                rospy.loginfo("[FOLLOW_ROW_CENTER] Row markers lost beyond limit → transition to IDLE")
                return StateType.IDLE
            return None
        return None

    def exit(self):
        """状態終了"""
        rospy.loginfo("[FOLLOW_ROW_CENTER] Exiting")

    # -------------------------
    # 内部処理メソッド
    # -------------------------

    def _filter_markers_in_range(self, markers: List[ArUcoMarker]) -> List[ArUcoMarker]:
        """最大距離以内のマーカーのみを抽出する
        条件:
          - distance_xz が存在
          - z成分(distance_xz[1])が max_dist_cm 以下
        """
        result: List[ArUcoMarker] = []
        for m in markers:
            if m.distance_xz is None or len(m.distance_xz) < 2:
                continue
            z = m.distance_xz[1]

            if z <= self.config.max_dist_cm:
                result.append(m)
        return result

    def _split_left_right(self, markers: List[ArUcoMarker]) -> Tuple[List[ArUcoMarker], List[ArUcoMarker]]:
        """左右のマーカー群へ振り分ける"""
        left = [m for m in markers if m.marker_id == self.config.left_marker_id]
        right = [m for m in markers if m.marker_id == self.config.right_marker_id]
        return left, right

    @staticmethod
    def _mean_x(markers: List[ArUcoMarker]) -> Optional[float]:
        """マーカー群の x[cm] の平均値取得"""
        x_val: List[float] = []
        for m in markers:
            if m.distance_xz is None:
                continue
            x = m.distance_xz[0]
            if x is None:
                continue
            x_val.append(x)
        if not x_val:
            return None
        return float(np.mean(x_val))

    def _compute_center_x(self, left_markers: List[ArUcoMarker], right_markers: List[ArUcoMarker]) -> float:
        """左右マーカー群から列の中心 x[cm] を推定
            - 両側あれば平均
            - 片側のみならオフセットで疑似中心を生成
            - どちらも無ければ 0.0
        """
        left_x = self._mean_x(left_markers)
        right_x = self._mean_x(right_markers)

        if left_x is not None and right_x is not None:
            return (left_x + right_x) / 2.0
        if left_x is not None:
            return left_x + self.config.center_offset_cm
        if right_x is not None:
            return right_x - self.config.center_offset_cm
        return 0.0

    def _decide_motion(self, center_x: float) -> ControlCommand:
        """
        center_x:
            + 正 → 車体が左側へ寄っている（または左向き）→ 車体を右へ
            + 負 → 車体が右側へ寄っている → 車体を左へ
            0 付近 → 中心

        制御方針:
           - |center_x| < 5cm → 真ん中とみなし直進
           - center_x > 5 → 右前進（右旋回）
           - center_x < -5 → 左前進（左旋回）
        """
        
        # デッドゾーン（車体がほぼ中心にいるとみなす範囲）
        dead_zone = 5.0  # cm

        if abs(center_x) <= dead_zone:
            # 中心 → まっすぐ前進
            rospy.loginfo_throttle(0.5, f"[RUNNING_CROP_ROWS] center_x={center_x:.1f}cm → move forward")
            return ControlCommand.move_forward(SpeedLevel.HIGH)

        elif center_x > 0:
            # 車体が左に寄っている → 右方向へ補正
            rospy.loginfo_throttle(0.5, f"[RUNNING_CROP_ROWS] center_x={center_x:.1f}cm → right forward")
            return ControlCommand.forward_right(SpeedLevel.MID)

        else:
            # 車体が右に寄っている → 左方向へ補正
            rospy.loginfo_throttle(0.5, f"[RUNNING_CROP_ROWS] center_x={center_x:.1f}cm → left forward")
            return ControlCommand.forward_left(SpeedLevel.MID)

    @staticmethod
    def _min_z(markers: List[ArUcoMarker]) -> Optional[float]:
        """マーカー群の distance_xz[1] (z成分: 前方距離[cm]) の最小値を取得
            有効データが無ければ None
        """
        z_vals: List[float] = []
        for m in markers:
            if m.distance_xz is None or len(m.distance_xz) < 2:
                continue
            z = m.distance_xz[1]
            if z is None:
                continue
            z_vals.append(z)
        if not z_vals:
            return None
        return float(np.min(z_vals))

    def _log_step(
        self,
        imu_data: Optional[IMUData],
        # camera_data: Optional[CameraData],
        markers: List[ArUcoMarker],
        center_x: Optional[float],
        cmd: ControlCommand,
    ) -> None:
        """FOLLOW_ROW_CENTER 専用の1ステップログを書き出す。"""
        logger = get_state_logger(
            state_name="FOLLOW_ROW_CENTER",
            header=[
                "timestamp",
                "miss_count",
                "num_markers_all",
                "num_markers_in_range",
                "num_left",
                "num_right",
                "center_x_cm",
                "command_type",
                "speed_level",
            ],
        )

        if imu_data is not None and getattr(imu_data, "timestamp", None) is not None:
            ts = float(imu_data.timestamp)
        # elif camera_data is not None and getattr(camera_data, "timestamp", None) is not None:
        #     ts = float(camera_data.timestamp)
        else:
            ts = _time.time()

        num_all = len(markers) if markers is not None else 0
        markers_in_range = self._filter_markers_in_range(markers) if markers else []
        num_in_range = len(markers_in_range)
        left_markers, right_markers = self._split_left_right(markers_in_range)

        cmd_type = getattr(getattr(cmd, "command", None), "name", "")
        speed_level = getattr(getattr(cmd, "speed", None), "name", "")

        try:
            logger.log_row(
                [
                    f"{ts:.6f}",
                    self.get_miss_count(),
                    num_all,
                    num_in_range,
                    len(left_markers),
                    len(right_markers),
                    center_x if center_x is not None else "",
                    cmd_type,
                    speed_level,
                ]
            )
        except Exception:
            # ログ失敗時も制御に影響を与えない
            pass
