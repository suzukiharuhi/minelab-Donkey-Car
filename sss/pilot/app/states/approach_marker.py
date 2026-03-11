"""マーカー接近状態"""
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple
import rospy
import time as _time

from pilot.app.states.base_state import BaseState
from pilot.app.core.commands import ControlCommand, SpeedLevel
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.core.step_logger import get_state_logger

@dataclass
class MarkerStopConfig:
    """
    マーカー接近設定（停止距離・優先度）
    """
    stop_dist_by_id: Dict[int, float] = field(default_factory=lambda: {0: 150.0, 10: 100.0}) # マーカー毎の停止距離 [cm]
    priority: Dict[int, int] = field(default_factory=lambda: {0: 0, 10: 1})  # 優先度
    miss_limit: int = 5          # ロスト許容: 何フレームまでロストしてもよいか
    reselect_after_miss: int = 5  # 追従: 何フレームまではロストしてもターゲット維持するか
    offset_thr: float = 5.0    # 偏差閾値 [cm]
    max_candidate_distance: float = 400.0  # 候補とみなす最大距離 [cm]

class ApproachMarkerState(BaseState):
    def __init__(self):
        super().__init__()
        self.config = MarkerStopConfig()
        self._target_id: Optional[int] = None  # 追従対象マーカーID
        self._target: Optional[ArUcoMarker] = None  # 追従対象マーカーオブジェクト
        self._target_lost_frames: int = 0  # 追従対象ロストフレーム数

    def needs_pose_estimation(self) -> bool:
        """marker IDと距離情報が必要"""
        return True

    def enter(self):
        """状態開始"""
        rospy.loginfo(f"[APPROACH] Entered ----------")
        self.reset_miss_count()
        self._target_id = None
        self.target = None
        self._target_lost_frames = 0

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """
        状態動作:
        - ターゲット更新とロストカウント処理
        - ロスト時: 低速前進/停止を共通処理に委譲
        - 接近制御: ターゲットのx座標に応じて前進/斜め前進
        """
        # ターゲット更新とロストカウント処理
        if not self._update_target_and_miss(markers):
            # 有効なターゲットがいない間は、一時ロスト扱いとして低速前進/停止を共通処理に委譲
            cmd = self.handle_transient_loss(
                recover_command=ControlCommand.move_forward(SpeedLevel.SLOW),
                miss_limit=self.config.miss_limit,
                log_tag="[APPROACH]",
            )
            self._log_step(imu_data, markers, cmd)
            return cmd

        # ここから先は self.target が存在する前提で接近制御
        if self.target.distance_xz is None:
            # ターゲットの距離不明
            cmd = ControlCommand.move_forward(SpeedLevel.SLOW)
            self._log_step(imu_data, markers, cmd)
            return cmd

        x_cm, z_cm = self.target.distance_xz
        if x_cm >= self.config.offset_thr:
            rospy.loginfo_throttle(1.0, f"[APPROACH] x={x_cm:.1f}cm 	 FORWARD_RIGHT(HIGH)")
            rospy.loginfo_throttle(1.0, f"[APPROACH] z={z_cm:.1f}cm")
            cmd = ControlCommand.forward_right(SpeedLevel.MID)
        elif x_cm <= -self.config.offset_thr:
            rospy.loginfo_throttle(1.0, f"[APPROACH] x={x_cm:.1f}cm 	 FORWARD_LEFT(MID)")
            rospy.loginfo_throttle(1.0, f"[APPROACH] z={z_cm:.1f}cm")
            cmd = ControlCommand.forward_left(SpeedLevel.MID)
        else:
            rospy.loginfo_throttle(1.0, f"[APPROACH] z={z_cm:.1f}cm 	 FORWARD(MID)")
            cmd = ControlCommand.move_forward(SpeedLevel.MID)

        self._log_step(imu_data, markers, cmd)
        return cmd
    
    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """状態遷移
        遷移条件:
        - マーカー未検出 or ターゲットロストが許容限度を超えた → IDLEへ
        - ターゲットが停止距離に達した → 次状態へ
        """
        # マーカー未検出
        if markers is None:
            if self.is_lost_beyond_limit(
                miss_limit=self.config.miss_limit,
                log_tag="[APPROACH] Marker",
            ):
                rospy.loginfo("[APPROACH] Marker lost beyond limit → transition to IDLE")
                return StateType.IDLE
            return None
        
        # マーカー検出（ターゲットマーカーロスト遷移判定）
        if self.target is None:
            if self.is_lost_beyond_limit(
                miss_limit=self.config.miss_limit,
                log_tag="[APPROACH] Marker",
            ):
                rospy.loginfo("[APPROACH] Marker lost beyond limit → transition to IDLE")
                return StateType.IDLE
            return None

        # ターゲットが見えている場合の距離判定
        if self.target and self.target.distance_xz is not None:
            stop_dist = self.config.stop_dist_by_id.get(self.target.marker_id, 100.0)
            if self.target.distance_xz[1] <= stop_dist:
                rospy.loginfo(f"[APPROACH] transition for marker {self.target.marker_id} (z={self.target.distance_xz[1]:.1f}cm <= {stop_dist}cm)")
                if self.target.marker_id == 10:
                    return StateType.ROTATE180
                if self.target.marker_id == 0:
                    return StateType.AUTO_CHARGING
        return None

    def exit(self):
        """状態終了"""
        rospy.loginfo("[APPROACH] Exiting")
        self._target_id = None
        self._target_lost_frames = 0

    # -------------------------
    # 内部処理メソッド
    # -------------------------

    def _marker_priority(self, marker_id: int) -> int:
        return self.config.priority.get(marker_id, 100)

    def _log_step(
        self,
        imu_data: Optional[IMUData],
        # camera_data: Optional[CameraData],
        markers: List[ArUcoMarker],
        cmd: ControlCommand,
    ) -> None:
        """APPROACH_MARKER 専用の1ステップログを書き出す。"""
        logger = get_state_logger(
            state_name="approach_marker",
            header=[
                "timestamp",
                "target_id",
                "miss_count",
                "target_lost_frames",
                "num_markers",
                "target_x_cm",
                "target_z_cm",
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

        target_x = None
        target_z = None
        if self.target is not None and self.target.distance_xz is not None:
            target_x, target_z = self.target.distance_xz

        cmd_type = getattr(getattr(cmd, "command", None), "name", "")
        speed_level = getattr(getattr(cmd, "speed", None), "name", "")

        try:
            logger.log_row(
                [
                    f"{ts:.6f}",
                    self._target_id if self._target_id is not None else "",
                    self.get_miss_count(),
                    self._target_lost_frames,
                    len(markers) if markers is not None else 0,
                    target_x if target_x is not None else "",
                    target_z if target_z is not None else "",
                    cmd_type,
                    speed_level,
                ]
            )
        except Exception:
            # ログ失敗時も制御に影響を与えない
            pass

    def _update_target_and_miss(self, markers: List[ArUcoMarker]) -> bool:
        """マーカー群からターゲットを更新し、ロストカウンタを更新する

        Returns:
            True: 有効なターゲットあり（self.target が設定済み）
            False: ターゲット無し（ロストカウンタを 1 増加済み）
        """
        # マーカー自体が 1 枚もない → ターゲット無しとしてロストカウント加算
        if not markers:
            self.target = None
            self.inc_miss_count()
            return False

        # 接近用ターゲット選択
        self.target = self._pick_target(markers)

        # 接近に使えるマーカーが選べない場合もロスト扱い
        if self.target is None:
            self.inc_miss_count()
            return False

        # ターゲットが見えている場合はロストカウントをリセット
        self.reset_miss_count()
        return True

    def _pick_target(self, markers: List[ArUcoMarker]) -> Optional[ArUcoMarker]:
        ##### ここに穴がありそう
        """ターゲット選択ロジック"""
        # フィルタ: 距離情報がないマーカーは候補外
        candidates = [
            m
            for m in markers
            if m.distance_xz is not None and m.distance_xz[1] <= self.config.max_candidate_distance
        ]
        # 候補なし → ロストフレーム数更新・None返却
        if not candidates:
            if self._target_id is not None:
                self._target_lost_frames += 1
                rospy.loginfo(f"[APPROACH] Target {self._target_id} temporarily lost (frames={self._target_lost_frames}/{self.config.reselect_after_miss})")
            return None

        # 候補マーカーの辞書作成{marker_id: marker}
        id_to_marker = {m.marker_id: m for m in candidates}

        # 既存ターゲット維持
        if self._target_id is not None:
            current_marker = id_to_marker.get(self._target_id) # 現ターゲットのマーカー取得（辞書にない場合はNone）
            current_priority = self._marker_priority(self._target_id) # 現ターゲットの優先度取得
            if current_marker is not None: # 現ターゲットが見えている場合
                self._target_lost_frames = 0
                # より高優先度のマーカーが見えた場合のみ切り替え
                higher_priority_candidates = [
                    m
                    for m in candidates
                    if self._marker_priority(m.marker_id) < current_priority
                ]
                if higher_priority_candidates:
                    best = min(
                        higher_priority_candidates,
                        key=lambda m: (
                            self._marker_priority(m.marker_id),
                            m.distance_xz[1],
                        ),
                    )
                    self._target_id = best.marker_id
                    rospy.loginfo(
                        "[APPROACH] Switch target to ID=%d (priority %d, z=%.1fcm) due to higher priority",
                        best.marker_id,
                        self._marker_priority(best.marker_id),
                        best.distance_xz[1],
                    )
                    return best
                # そのまま維持
                return current_marker

            # 現ターゲットが見えない → ロストカウンタ更新
            self._target_lost_frames += 1
            if self._target_lost_frames < self.config.reselect_after_miss:
                rospy.loginfo_throttle(
                    1.0,
                    f"[APPROACH] Maintaining target ID={self._target_id} (lost {self._target_lost_frames}/{self.config.reselect_after_miss})",
                )
                return None
            rospy.loginfo(
                "[APPROACH] Target ID=%d lost for %d frames → reselect",
                self._target_id,
                self._target_lost_frames,
            )
            self._target_id = None
            self._target_lost_frames = 0

        # 新規ターゲット選択
        best_marker = min(
            candidates,
            key=lambda m: (
                self._marker_priority(m.marker_id),
                m.distance_xz[1],
            ),
        )
        self._target_id = best_marker.marker_id
        self._target_lost_frames = 0
        rospy.loginfo(
            "[APPROACH] Lock target ID=%d (priority %d, z=%.1fcm)",
            best_marker.marker_id,
            self._marker_priority(best_marker.marker_id),
            best_marker.distance_xz[1],
        )
        return best_marker


