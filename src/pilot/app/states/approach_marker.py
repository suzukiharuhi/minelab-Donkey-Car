"""マーカー接近状態"""
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple
import rospy

from pilot.app.states.base_state import BaseState
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData

@dataclass
class MarkerStopConfig:
    """
    マーカー接近設定（停止距離・優先度）
    """
    stop_dist_by_id: Dict[int, float] = field(default_factory=lambda: {0: 80.0, 10: 80.0, 3: 80.0, 4: 80.0, 7: 80.0})
    priority: Dict[int, int] = field(default_factory=lambda: {0: 0, 10: 1, 3: 2, 4: 2, 7: 3})  # 優先度
    miss_limit: int = 10
    reselect_after_miss: int = 3  # 追従: 何フレームまではロストしてもターゲット維持するか

class ApproachMarkerState(BaseState):
    # ...existing code...
    def __init__(self):
        super().__init__()
        self.config = MarkerStopConfig()
        self._last_seen_time: Optional[rospy.Time] = None
        self._miss_count: int = 0
        self._target_id: Optional[int] = None  # 追従用
    
    def needs_pose_estimation(self) -> bool:
        """marker IDと距離情報が必要"""
        return True

    def enter(self):
        """状態開始"""
        rospy.loginfo(f"[APPROACH] Entered ----------")
        self._last_seen_time = rospy.Time.now()
        self._miss_count = 0
        self._target_id = None

    def _pick_target(self, markers: List[ArUcoMarker]) -> Optional[ArUcoMarker]:
        """追従（ロックオン）付きターゲット選択
            - 既存ターゲットが見えていればそれを優先維持
            - 一時ロスト中は reselect_after_miss までは維持（再選択しない）
            - それ以外は優先度→距離で新規選択
        """
        if not markers:
            return None

        # 既存ターゲットが見えていれば維持
        if self._target_id is not None:
            for m in markers:
                if m.marker_id == self._target_id and m.distance_xz is not None:
                    return m
                
            # ロスト中だが、許容範囲内 → グレース期間なので再選択しない
            if self._miss_count <= self.config.reselect_after_miss:
                return None
            
        # 新しくターゲットを選び直す（優先度 → 距離）
        candidates = []
        for m in markers:
            if m.distance_xz is not None:
                candidates.append(m)
        if not candidates:
            return None
        
        # 優先度テーブル + 距離 z に基づいて比較
        def sort_key(m: ArUcoMarker):
            priority = self.config.priority.get(m.marker_id, 99)
            _, z = m.distance_xz
            return (priority, z)

        # 最も「優先度が高く → 距離が近い」マーカーを選ぶ
        chosen = min(candidates, key=sort_key)

        # ターゲットIDを更新
        self._target_id = chosen.marker_id

        return chosen            

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """制御動作

        動作:
            - ターゲットが見えている間は常に SpeedLevel.MID で前進
            - 未検出/距離不明: ロストカウントを増やし、miss_limit 以内は MID 前進、超過で STOP
        異常停止条件:
            - ターゲットが見えなくなった場合、ロストカウントを増やす
            - ロストカウントが miss_limit を超えた場合、STOP
        """
        target = self._pick_target(markers) if markers else None

        if target is None or target.distance_xz is None:
            # ロスト判定
            self._miss_count += 1
            if self._miss_count <= self.config.miss_limit:
                rospy.loginfo_throttle(1.0, f"[APPROACH] Marker temporarily lost (miss={self._miss_count}/{self.config.miss_limit}) → MID")
                return ControlCommand.move_forward(SpeedLevel.MID)
            rospy.logwarn_throttle(1.0, "[APPROACH] Marker lost beyond limit → STOP")
            return ControlCommand.stop()

        # ターゲット可視
        self._miss_count = 0
        self._last_seen_time = rospy.Time.now()
        # マーカーの左右オフセットで前進方向を調整
        if target.distance_xz is not None:
            x_cm, z_cm = target.distance_xz
            threshold = 5.0  # 左右判定しきい値 [cm]
            if x_cm >= threshold:
                rospy.loginfo_throttle(1.0, f"[APPROACH] x={x_cm:.1f}cm → FORWARD_RIGHT(MID)")
                return ControlCommand.forward_right(SpeedLevel.MID)
            elif x_cm <= -threshold:
                rospy.loginfo_throttle(1.0, f"[APPROACH] x={x_cm:.1f}cm → FORWARD_LEFT(MID)")
                return ControlCommand.forward_left(SpeedLevel.MID)

        # しきい値内は直進
        return ControlCommand.move_forward(SpeedLevel.MID)

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """次状態へ遷移"""
        # ターゲット選択（ロックオン含む）
        target = self._pick_target(markers) if markers else None

        # ロスト遷移判定（miss_limit 超過で IDLE へ）
        if target is None:
            if self._miss_count > self.config.miss_limit:
                rospy.loginfo("[APPROACH] Marker lost beyond limit → transition to IDLE")
                return StateType.IDLE
            # まだ猶予内は遷移なし
            return None

        # 以降はターゲットが見えている場合の距離判定
        marker_id: Optional[int] = None
        z_cm: Optional[float] = None
        if target and target.distance_xz is not None:
            marker_id = target.marker_id
            _, z_cm = target.distance_xz

        if z_cm is not None and marker_id is not None:
            stop_dist = self.config.stop_dist_by_id.get(marker_id, 80.0)
            if z_cm <= stop_dist:
                if marker_id == 10:
                    return StateType.ROTATE180_MARKER10
                elif marker_id == 0:
                    return StateType.AUTO_CHARGING
        return None

    def exit(self):
        """状態終了"""
        rospy.loginfo("[APPROACH] Exiting")

