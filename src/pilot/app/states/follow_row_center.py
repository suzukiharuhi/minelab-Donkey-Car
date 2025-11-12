"""作物列走行状態"""
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple
import rospy
import numpy as np

from pilot.app.states.base_state import BaseState
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData

@dataclass
class FollowRowCenterConfig:
    """作物列走行の設定値
        - left_marker_id: 左列を示すArUcoマーカーID
        - right_marker_id: 右列を示すArUcoマーカーID
        - max_dist_cm: 判定対象とする最大距離[cm]（この距離以内のマーカーのみ使用）
        - center_offset_cm: 片側のみ検出時に仮の中心を置くためのオフセット量[cm]
        - specific_markers: 特殊マーカーID -> 遷移先StateType の対応
    """
    left_marker_id: int = 20
    right_marker_id: int = 21
    max_dist_cm: float = 400.0
    center_offset_cm: float = 40.0
    specific_markers: List[int] = field(default_factory=lambda: [0, 10])

class FollowRowCenterState(BaseState):
    def __init__(self):
        super().__init__()
        self.cfg = FollowRowCenterConfig()

    def needs_pose_estimation(self) -> bool:
        """marker IDと距離情報が必要"""
        return True

    def enter(self):
        """状態開始"""
        rospy.loginfo("[RUNNING_CROP_ROWS] Entered ----------")

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker],) -> Optional[StateType]:
        """遷移条件の判定
            - 有効距離内のマーカーがなければ遷移なし
            - 作物列用マーカー(20,21)が全く無く、かつ specific_markers(0,10) が見つかった場合 → APPROACH_MARKER 遷移
            - 作物列用マーカー(20,21)が存在しても、specific_markers(0,10) の最近距離が列マーカー群の最近距離より近い場合 → APPROACH_MARKER 遷移
            - 上記以外は遷移なし（列走行継続）
        """
        if not markers:
            return None

        markers_in_range = self._filter_markers_in_range(markers)
        if not markers_in_range:
            return None
        # 作物列マーカー群（左右）
        left_markers, right_markers = self._split_left_right(markers_in_range)
        row_markers: List[ArUcoMarker] = []
        row_markers.extend(left_markers)
        row_markers.extend(right_markers)

        # 特殊マーカー群
        approach_markers = [m for m in markers_in_range if m.marker_id in self.cfg.specific_markers]

        if not approach_markers:
            # 特殊マーカーが無ければ遷移なし
            return None

        if not row_markers and approach_markers:
            # 行走行用マーカーが無い状態で 0/10 が見つかった → 接近遷移
            mid_list = ",".join(str(m.marker_id) for m in approach_markers)
            rospy.loginfo(f"[RUNNING_CROP_ROWS] row markers absent; approach markers {mid_list} → APPROACH")
            return StateType.APPROACH_MARKER

        # 距離比較（z成分を前方距離として利用）
        min_row_z = self._min_z(row_markers)
        min_app_z = self._min_z(approach_markers)
        if min_row_z is None and min_app_z is not None:
            rospy.loginfo(f"[RUNNING_CROP_ROWS] no row distance; approach distance {min_app_z:.1f}cm → APPROACH")
            return StateType.APPROACH_MARKER
        if (min_row_z is not None and min_app_z is not None) and (min_app_z < min_row_z):
            rospy.loginfo(f"[RUNNING_CROP_ROWS] approach marker closer ({min_app_z:.1f}cm < {min_row_z:.1f}cm) → APPROACH")
            return StateType.APPROACH_MARKER
        return None

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """制御動作: 
        マーカーから中心を推定し走行コマンドを決定する
        """
        if not markers:
            return ControlCommand.stop()

        markers_in_range = self._filter_markers_in_range(markers)
        if not markers_in_range:
            rospy.logwarn_throttle(1.0, "[RUNNING_CROP_ROWS] No markers in range → STOP")
            return ControlCommand.stop()

        left_markers, right_markers = self._split_left_right(markers_in_range)
        if not left_markers and not right_markers:
            rospy.logwarn_throttle(1.0, "[RUNNING_CROP_ROWS] No left/right markers → STOP")
            return ControlCommand.stop()

        center_x = self._compute_center_x(left_markers, right_markers)
        rospy.logdebug(
            f"[RUNNING_CROP_ROWS] center_x={center_x:.2f} (L={len(left_markers)}, R={len(right_markers)})"
        )

        return self._decide_motion(center_x)

    def exit(self):
        """状態終了時のフック"""
        rospy.loginfo("[RUNNING_CROP_ROWS] Exiting")


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

            if z <= self.cfg.max_dist_cm:
                result.append(m)
        return result

    def _split_left_right(self, markers: List[ArUcoMarker]) -> Tuple[List[ArUcoMarker], List[ArUcoMarker]]:
        """左右のマーカー群へ振り分ける"""
        left = [m for m in markers if m.marker_id == self.cfg.left_marker_id]
        right = [m for m in markers if m.marker_id == self.cfg.right_marker_id]
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
            return left_x + self.cfg.center_offset_cm
        if right_x is not None:
            return right_x - self.cfg.center_offset_cm
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