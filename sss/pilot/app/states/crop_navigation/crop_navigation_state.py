"""作物列走行状態 (CropNavigationState).

各フレームで以下のステップを実行する統合状態クラス:

    Step 1: 障害物検知
        - ROI を 3×3=9 セルに分割
        - 各セルの中央値距離を算出
        - いずれかのセルが閾値未満 → has_obstacle=True

    Step 2: 障害物あり → 障害物回避
        - 距離 → 危険度 変換
        - 左・中央・右 列リスク算出
        - NORMAL/BACK/TURN フェーズで回避コマンド生成

    Step 3: 障害物なし → FTG-i による列中央走行
        - depth → 1D レンジプロファイル生成
        - 自由空間ギャップ抽出
        - 評価関数 U(θ) で最良方位を選択
        - 方位 → ControlCommand 変換

クラス構成:
    CropNavigationState (BaseState)   ← 状態クラス（本ファイル）
    ├── ObstacleDetector              ← Step 1
    ├── ObstacleAvoider               ← Step 2
    └── RowFollower                   ← Step 3

    各サブクラスは BaseState に依存せず単独テスト可能.
"""
from __future__ import annotations

from enum import Enum, auto
from typing import List, Optional

import numpy as np
import rospy

from pilot.app.core.commands import CommandType, ControlCommand
from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, CameraData, IMUData

from .config import CropNavigationConfig
from .obstacle_detector import DetectionResult, ObstacleDetector
from .obstacle_avoider import ObstacleAvoider
from .row_follower import RowFollower


# ---------------------------------------------------------------------------
# 走行モード定義
# ---------------------------------------------------------------------------

class NavigationMode(Enum):
    """現在の走行モード（ログ・デバッグ用）."""
    ROW_FOLLOWING      = auto()   # 障害物なし: FTG-i 列走行
    OBSTACLE_AVOIDANCE = auto()   # 障害物あり: 回避行動中


# ---------------------------------------------------------------------------
# 状態クラス
# ---------------------------------------------------------------------------

class CropNavigationState(BaseState):
    """作物列走行 + 障害物回避を統合した状態クラス.

    フレームごとの処理フロー:
        1. ObstacleDetector.detect(depth) → DetectionResult
        2. has_obstacle=True  → ObstacleAvoider.compute_command(cell_stats)
           has_obstacle=False → RowFollower.compute_command(depth)

    インタフェース:
        - BaseState を継承
        - enter()            : サブコンポーネントをリセット
        - execute()          : 毎フレーム走行コマンドを生成
        - check_transition() : 常に None（本状態で走行継続）
        - exit()             : 終了ログ

    設定変更:
        コンストラクタに CropNavigationConfig を渡すか,
        デフォルト値のまま使用する.
    """

    def __init__(self, config: Optional[CropNavigationConfig] = None) -> None:
        super().__init__()
        self._STATE = "crop_navigation"
        self.config = config or CropNavigationConfig()

        # ---- サブコンポーネント ----
        self._detector = ObstacleDetector(self.config.detection)
        self._avoider  = ObstacleAvoider(self.config.avoidance)
        self._follower = RowFollower(self.config.row_follower)

        # ---- 状態変数 ----
        self._mode:        NavigationMode = NavigationMode.ROW_FOLLOWING
        self._frame_index: int            = 0

    # ----------------------------------------------------------------
    # BaseState 実装
    # ----------------------------------------------------------------

    def needs_pose_estimation(self) -> bool:
        """depth のみ使用するため ArUco 姿勢推定は不要."""
        return False

    def enter(self) -> None:
        """状態開始 — 全サブコンポーネントをリセットする."""
        rospy.loginfo("[CropNav] Entered -----------------")
        self._avoider.reset()
        self._follower.reset()
        self._mode        = NavigationMode.ROW_FOLLOWING
        self._frame_index = 0

    def execute(
        self,
        imu_data:    Optional[IMUData],
        camera_data: Optional[CameraData],
        markers:     List[ArUcoMarker],
    ) -> ControlCommand:
        """毎フレーム depth image から走行コマンドを生成する.

        処理フロー:
            Step 1: 障害物検知 (ObstacleDetector)
            Step 2/3: 走行モードに応じてコマンド生成
        """
        # ---- depth image 取得 ----
        depth = camera_data.depth_image if camera_data is not None else None
        if depth is None:
            rospy.logwarn_throttle(1.0, "[CropNav] depth_image is None → STOP")
            return ControlCommand(CommandType.STOP)
        if depth.ndim != 2:
            rospy.logwarn_throttle(1.0, "[CropNav] depth_image is not 2D → STOP")
            return ControlCommand(CommandType.STOP)

        # ==============================================================
        # Step 1: 障害物検知
        # ==============================================================
        detection = self._detector.detect(depth)

        # デバッグ画像：検知グリッド（描画はワーカースレッドで実行）
        self.log_image_lazy(
            lambda det=detection: self._render_detection_grid(det),
            tag="detection_grid",
        )

        # ==============================================================
        # Step 2 / Step 3: モード切り替え & コマンド生成
        # ==============================================================
        if detection.has_obstacle:
            self._mode = NavigationMode.OBSTACLE_AVOIDANCE
            cmd = self._run_avoidance(detection)
        else:
            self._mode = NavigationMode.ROW_FOLLOWING
            cmd = self._run_row_following(depth)

        # ---- メトリクスログ ----
        self.log_metric(
            metrics={
                "mode":             self._mode.name,
                "has_obstacle":     int(detection.has_obstacle),
                "triggered_cells":  len(detection.triggered_cells),
                "avoider_phase":    self._avoider.phase.name,
                "frame":            self._frame_index,
            },
            tag="execute",
        )

        rospy.loginfo_throttle(
            1.0,
            "[CropNav] frame=%d mode=%s obstacle=%s triggered=%s cmd=%s",
            self._frame_index,
            self._mode.name,
            detection.has_obstacle,
            detection.triggered_cells,
            cmd.command.name,
        )

        self.log_jsonl(
            tag="debug",
            data={
                "t_ros": rospy.get_time(),
                "mode": self._mode.name,  # "OBSTACLE" or "ROW_FOLLOW"
                # 1. 障害物検知
                "obstacle": {
                    "present": detection.has_obstacle,
                },
            }
        )

        self._frame_index += 1
        return cmd

    def check_transition(
        self,
        imu_data:    Optional[IMUData],
        camera_data: Optional[CameraData],
        markers:     List[ArUcoMarker],
    ) -> Optional[StateType]:
        """状態遷移判定（本状態では常に None を返して走行継続）."""
        return None

    def exit(self) -> None:
        rospy.loginfo("[CropNav] Exiting")

    # ----------------------------------------------------------------
    # Step 2: 障害物回避（内部メソッド）
    # ----------------------------------------------------------------

    def _run_avoidance(self, detection: DetectionResult) -> ControlCommand:
        """ObstacleAvoider でコマンドを生成する."""
        cmd = self._avoider.compute_command(detection.cell_stats)
        # rospy.loginfo_throttle(
        #     0.5,
        #     "[CropNav] Avoidance phase=%s triggered=%s cmd=%s",
        #     self._avoider.phase.name,
        #     detection.triggered_cells,
        #     cmd.command.name,
        # )
        return cmd

    # ----------------------------------------------------------------
    # Step 3: 列追従（内部メソッド）
    # ----------------------------------------------------------------

    def _run_row_following(self, depth: np.ndarray) -> ControlCommand:
        """RowFollower (FTG-i) でコマンドを生成する."""
        cmd, theta = self._follower.compute_command(depth)

        if cmd is None:
            rospy.logwarn_throttle(0.5, "[CropNav] RowFollower returned None → STOP")
            return ControlCommand(CommandType.STOP)

        # デバッグ画像ログ（描画はワーカースレッドで実行）
        # depth.copy() でメインループの次フレーム書き換えを防ぐ
        # self.log_image_lazy(
        #     lambda: self._follower.get_debug_images()[0],
        #     tag="roi_depth",
        # )
        # self.log_image_lazy(
        #     lambda: self._follower.get_debug_images()[1],
        #     tag="1d_range",
        # )
        # self.log_image_lazy(
        #     lambda: self._follower.get_debug_images()[2],
        #     tag="gap_select",
        # )
        # d = depth.copy()
        # self.log_image_lazy(
        #     lambda d=d: self._follower._render_roi(
        #         self._follower._build_1d_range(d)[2]
        #     ),
        #     tag="roi_depth",
        # )
        # self.log_image_lazy(
        #     lambda d=d: self._follower._render_1d_heatmap(
        #         self._follower._build_1d_range(d)[0]
        #     ),
        #     tag="1d_range",
        # )
        # self.log_image_lazy(
        #     lambda d=d: self._follower._render_gap_debug(
        #         self._follower._build_1d_range(d)[1]
        #     ),
        #     tag="gap_select",
        # )

        rospy.loginfo_throttle(
            0.5, "[CropNav] RowFollowing theta=%.3f cmd=%s", theta, cmd.command.name
        )
        return cmd

    # ----------------------------------------------------------------
    # 描画ヘルパー
    # ----------------------------------------------------------------

    def _render_detection_grid(
        self,
        detection:  DetectionResult,
        cell_size:  int = 80,
    ) -> Optional[np.ndarray]:
        """9セル検知グリッドを BGR 画像として返す.

        色コード:
            緑 : 安全（閾値以上）
            赤 : 障害物検知（triggered）
            灰 : データ欠損
        中央に中央値距離[mm] をテキスト表示.
        """
        try:
            import cv2
        except ImportError:
            return None

        row_labels = ("u", "m", "l")
        col_labels = ("l", "c", "r")

        img = np.zeros((cell_size * 3, cell_size * 3, 3), dtype=np.uint8)

        for ri, rl in enumerate(row_labels):
            for ci, cl in enumerate(col_labels):
                name             = rl + cl
                median_d, p_valid = detection.cell_stats.get(name, (np.nan, 0.0))
                is_triggered     = name in detection.triggered_cells

                # セル塗りつぶし色
                if not np.isfinite(median_d) or p_valid < 0.1:
                    color = (80, 80, 80)          # 灰: 欠損
                elif is_triggered:
                    color = (0, 0, 200)            # 赤: 障害物
                else:
                    color = (0, 180, 0)            # 緑: 安全

                y0 = ri * cell_size
                y1 = y0 + cell_size
                x0 = ci * cell_size
                x1 = x0 + cell_size
                img[y0:y1, x0:x1] = color

                # セル枠
                cv2.rectangle(img, (x0, y0), (x1 - 1, y1 - 1), (200, 200, 200), 1)

                # 距離テキスト
                d_str = f"{median_d:.0f}" if np.isfinite(median_d) else "N/A"
                cv2.putText(
                    img, d_str,
                    (x0 + 8, y0 + cell_size // 2 + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA,
                )
                # セル名
                cv2.putText(
                    img, name,
                    (x0 + 4, y0 + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (220, 220, 220), 1, cv2.LINE_AA,
                )

        return img
