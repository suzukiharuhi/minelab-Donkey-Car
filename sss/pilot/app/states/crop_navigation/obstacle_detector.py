"""障害物検知器 (ObstacleDetector).

depth image から ROI を 3×3=9 セルに分割し, 各セルの中央値距離を算出する.
いずれかのセルの中央値が閾値未満 (または有効画素率が低い) 場合に
障害物ありと判定する.

設計方針:
    - 純粋なデータ処理クラス（状態を持たない）
    - BaseState には依存せず, 単独テスト可能
    - cell_stats を DetectionResult に格納して呼び出し元に渡す
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import rospy

from .config import DetectionConfig


# ---------------------------------------------------------------------------
# 検知結果データクラス
# ---------------------------------------------------------------------------

@dataclass
class DetectionResult:
    """障害物検知の結果.

    Attributes:
        has_obstacle: いずれかのセルで障害物が検知されたか
        cell_stats:   {セル名: (中央値[mm], 有効画素率)} の辞書
        triggered_cells: 障害物ありと判定されたセル名リスト
    """
    has_obstacle: bool
    cell_stats: Dict[str, Tuple[float, float]]
    triggered_cells: List[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# 検知器クラス
# ---------------------------------------------------------------------------

class ObstacleDetector:
    """ROI 9分割による障害物検知器.

    depth image (2D, 単位 mm) を受け取り:
        1. ROI を 3×3 = 9 セルに分割
        2. 各セルの中央値距離と有効画素率を算出
        3. 閾値と比較 → has_obstacle を決定
    して DetectionResult を返す.

    最適化:
        - 有効マスクを画像全体で一括計算し, 各セルスライスで再利用
        - bool sum() で中間配列生成を回避
    """

    # セル名テーブル（行: upper/middle/lower, 列: left/center/right）
    _CELL_NAMES: Tuple[Tuple[str, str, str], ...] = (
        ("ul", "uc", "ur"),
        ("ml", "mc", "mr"),
        ("ll", "lc", "lr"),
    )

    def __init__(self, config: Optional[DetectionConfig] = None) -> None:
        self.config = config or DetectionConfig()

    # ----------------------------------------------------------------
    # パブリックインタフェース
    # ----------------------------------------------------------------

    def detect(self, depth: np.ndarray) -> DetectionResult:
        """depth image から障害物を検知する.

        Args:
            depth: 2D depth 画像 (単位 mm, float32 推奨)

        Returns:
            DetectionResult
        """
        cell_stats = self._compute_cell_stats(depth)
        triggered  = self._check_threshold(cell_stats)
        has_obstacle = len(triggered) > 0

        if has_obstacle:
            rospy.loginfo_throttle(
                0.5,
                "[Detector] Obstacle in cells: %s",
                triggered,
            )

        return DetectionResult(
            has_obstacle=has_obstacle,
            cell_stats=cell_stats,
            triggered_cells=triggered,
        )

    # ----------------------------------------------------------------
    # セル統計量の計算
    # ----------------------------------------------------------------

    def _compute_cell_stats(
        self, depth: np.ndarray
    ) -> Dict[str, Tuple[float, float]]:
        """ROI を 9 セルに分割し (中央値[mm], 有効画素率) を返す.

        Returns:
            {セル名: (median_mm, valid_rate)}
            データ欠損の場合は (nan, 0.0)
        """
        cfg = self.config
        h, w = depth.shape[:2]

        # ---- ROI 境界を計算 ----
        r0_roi = max(0, min(int(h * cfg.v_min_frac), h))
        r1_roi = max(0, min(int(h * cfg.v_max_frac), h))
        c0_roi = max(0, min(int(w * cfg.u_min_frac), w))
        c1_roi = max(0, min(int(w * cfg.u_max_frac), w))

        if r1_roi <= r0_roi or c1_roi <= c0_roi:
            # ROI が取れない場合, 全セルを欠損扱い
            return {
                name: (np.nan, 0.0)
                for row in self._CELL_NAMES
                for name in row
            }

        roi_h = r1_roi - r0_roi
        roi_w = c1_roi - c0_roi

        # ---- 有効マスクを ROI 内で一括計算（最重要最適化）----
        roi_valid = (
            np.isfinite(depth[r0_roi:r1_roi, c0_roi:c1_roi])
            & (depth[r0_roi:r1_roi, c0_roi:c1_roi] >= cfg.depth_min_mm)
            & (depth[r0_roi:r1_roi, c0_roi:c1_roi] <= cfg.depth_max_mm)
        )
        roi_depth = depth[r0_roi:r1_roi, c0_roi:c1_roi]

        # ---- ROI を 3×3 に等分割するセル境界 ----
        row_splits = [
            (roi_h * i // 3, roi_h * (i + 1) // 3)
            for i in range(3)
        ]
        col_splits = [
            (roi_w * j // 3, roi_w * (j + 1) // 3)
            for j in range(3)
        ]

        stats: Dict[str, Tuple[float, float]] = {}
        for ri, (rr0, rr1) in enumerate(row_splits):
            for ci, (cc0, cc1) in enumerate(col_splits):
                name  = self._CELL_NAMES[ri][ci]
                total = (rr1 - rr0) * (cc1 - cc0)

                if total == 0:
                    stats[name] = (np.nan, 0.0)
                    continue

                cell_valid = roi_valid[rr0:rr1, cc0:cc1]
                n_valid    = int(cell_valid.sum())

                if n_valid == 0:
                    stats[name] = (np.nan, 0.0)
                    continue

                p_valid  = n_valid / total
                median_d = float(np.median(roi_depth[rr0:rr1, cc0:cc1][cell_valid]))
                stats[name] = (median_d, p_valid)

        return stats

    # ----------------------------------------------------------------
    # 閾値チェック
    # ----------------------------------------------------------------

    def _check_threshold(
        self, cell_stats: Dict[str, Tuple[float, float]]
    ) -> List[str]:
        """各セルを閾値と比較し, 障害物ありとみなすセル名リストを返す.

        判定条件（いずれかに該当すれば障害物）:
            1. 有効画素率 < p_min  → データ欠損 → 保守的に障害物
            2. 中央値が NaN/Inf    → 保守的に障害物
            3. 中央値 < obstacle_threshold_mm → 障害物あり
        """
        cfg = self.config
        triggered: List[str] = []

        for name, (median_d, p_valid) in cell_stats.items():
            if p_valid < cfg.p_min:
                triggered.append(name)
                continue
            if not np.isfinite(median_d):
                triggered.append(name)
                continue
            if median_d < cfg.obstacle_threshold_mm:
                triggered.append(name)

        return triggered
