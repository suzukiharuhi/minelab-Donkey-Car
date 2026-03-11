"""深度画像に基づく障害物検知・回避状態 (ObstacleAvoidance).

ROI を 3×3 (9セル) に分割し, 各セルの代表距離（中央値）と有効画素率を算出.
セルごとの危険度を連続値で推定し, 左・中央・右リスクへ統合.
コマンド生成は「連続制御（NORMAL）」と「固定フレーム回避シーケンス（BACK→TURN）」を
フェーズで切り替える単一ロジックとして実装する.

チャタリング抑制の設計方針:
  - NORMAL フェーズでは ch3 を前進方向（>= 1500）のみに使用し, 後退しない.
  - 後退は BACK フェーズの固定コミットのみで行う.
  - 回避トリガは多数決（vote_window 中 votes_needed 票）で抑制する.

インタフェース:
    - BaseState を継承
    - enter()            : 状態開始時に呼ばれる
    - execute()          : 毎フレーム, depth image から制御コマンドを生成
    - check_transition() : 常に None を返す（状態維持）
    - exit()             : 状態終了時に呼ばれる
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rospy

from pilot.app.core.commands import CommandType, ControlCommand
from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.utils.data_models import ArUcoMarker, CameraData, IMUData


# ---------------------------------------------------------------------------
# 安全層フェーズ定義
# ---------------------------------------------------------------------------

class AvoidPhase(Enum):
    """回避フェーズ."""
    NORMAL = auto()   # 通常走行（連続制御）
    BACK   = auto()   # 固定フレーム後退
    TURN   = auto()   # 固定フレーム旋回


# ---------------------------------------------------------------------------
# 設定クラス
# ---------------------------------------------------------------------------

@dataclass
class ObstacleAvoidanceConfig:
    """ObstacleAvoidance の全パラメータ."""

    # ---- ROI 設定（画像サイズに対する割合）----
    v_min_frac: float = 0.30   # 高さ方向: 上端からの割合
    v_max_frac: float = 0.65
    u_min_frac: float = 0.15   # 幅方向: 左端からの割合
    u_max_frac: float = 0.85

    # ---- 深度の有効範囲 [mm] ----
    depth_min_mm: float = 10.0    # これ未満は外れ値として除外
    depth_max_mm: float = 10000.0  # これ超は遠方として除外

    # ---- 危険度変換パラメータ [mm] ----
    d_stop: float = 300.0    # この距離以下は危険度 1.0
    d_safe: float = 800.0   # この距離以上は危険度 0.0

    # ---- 有効画素率の信頼閾値 ----
    p_min: float = 0.10   # これ未満のセルは危険度を 1.0 にする

    # ---- 行列方向の重み（下段 > 中段 > 上段）----
    w_upper: float = 1.0
    w_middle: float = 2.0
    w_lower: float = 3.0

    # ---- 基本層ゲイン ----
    kt: float = 1.2    # 旋回ゲイン（左右リスク差 → 旋回 t）

    # ---- 回避トリガしきい値 ----
    r_avoid_on: float = 0.80   # 回避シーケンス開始しきい値（多数決の投票判定に使用）

    # ---- 多数決（チャタリング抑制）----
    votes_needed: int = 2      # 直近 vote_window フレーム中の回避票数
    vote_window:  int = 3

    # ---- 回避シーケンス フレーム数 ----
    back_frames: int = 10       # BACK フェーズ継続フレーム数（約0.2秒@30fps）
    turn_frames: int = 10       # TURN フェーズ継続フレーム数（約0.25秒@30fps）

    # ---- PWM 設定 ----
    pwm_center:   int = 1500
    pwm_min:      int = 1100
    pwm_max:      int = 1900

    ch3_forward_min: int = 1700  # NORMAL フェーズでの前進 ch3 下限（1500未満にしない）

    # ch1（旋回）振幅 — t=±1 のときの PWM オフセット
    A1: int = 300
    # ch3（推進）振幅 — s=1 のときの PWM オフセット
    A3: int = 300

    # 安全層 PWM
    A_back: int  = 200   # 後退 ch3 = pwm_center - A_back
    A_turn: int  = 300   # 旋回 ch1 = pwm_center ± A_turn


# ---------------------------------------------------------------------------
# メインクラス
# ---------------------------------------------------------------------------

class ObstacleAvoidanceState(BaseState):
    """深度画像に基づく 9セル危険度推定による障害物回避状態."""

    # セル名（row: upper/middle/lower, col: left/center/right）
    _CELL_NAMES: Tuple[str, ...] = (
        "ul", "uc", "ur",
        "ml", "mc", "mr",
        "ll", "lc", "lr",
    )

    def __init__(self) -> None:
        super().__init__()
        self._STATE = "obstacle_avoidance"
        self.config = ObstacleAvoidanceConfig()

        # 回避フェーズ管理
        self._phase: AvoidPhase = AvoidPhase.NORMAL
        self._phase_counter: int = 0     # 現フェーズの経過フレーム数
        self._avoid_votes: List[int] = []  # 多数決バッファ（0=通常, 1=回避）
        self._turn_dir: int = 1          # 旋回方向（+1=右, -1=左）

        # 直前リスク（デバッグ・ロギング用）
        self._last_risks: Dict[str, float] = {}

        self._frame_index: int = 0

    # ----------------------------------------------------------------
    # BaseState 実装
    # ----------------------------------------------------------------

    def needs_pose_estimation(self) -> bool:
        """depth のみ使用するため ArUco 姿勢推定は不要."""
        return False

    def enter(self) -> None:
        rospy.loginfo("[ObstAvoid] Entered -----------------")
        self._phase = AvoidPhase.NORMAL
        self._phase_counter = 0
        self._avoid_votes = []
        self._turn_dir = 1
        self._frame_index = 0

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker],) -> ControlCommand:
        """depth image から ObstacleAvoidance を実行して ControlCommand を返す."""

        depth = camera_data.depth_image
        if depth is None:
            rospy.logwarn_throttle(1.0, "[ObstAvoid] depth_image is None → STOP")
            return ControlCommand(CommandType.STOP)
        if depth.ndim != 2:
            rospy.logwarn_throttle(1.0, "[ObstAvoid] depth_image is not 2D → STOP")
            return ControlCommand(CommandType.STOP)

        # # ---- 1. ROI 抽出 ----
        # roi = self._extract_roi(depth)
        # if roi is None:
        #     rospy.logwarn_throttle(1.0, "[ObstAvoid] ROI extraction failed → STOP")
        #     return ControlCommand(CommandType.STOP)

        # ---- 2. 9セル → 代表距離・有効画素率 ----
        rois = self._compute_cell_stats(depth)   # {cell_name: (median_mm, valid_rate)}

        # ---- 3. 距離 → 危険度 ----
        risks = {name: self._dist_to_risk(d, p) for name, (d, p) in rois.items()}
        self._last_risks = risks

        # ---- 4. 左・中央・右リスク統合 ----
        R_left, R_center, R_right = self._aggregate_risks(risks)

        # ---- 5. デバッグ画像ログ ----
        roi_vis = self._render_roi(depth)
        if roi_vis is not None:
            self.log_image(image=roi_vis, tag="roi_depth")
            
        risk_vis = self._render_risk_grid(risks)
        if risk_vis is not None:
            self.log_image(image=risk_vis, tag="risk_grid")

        # ---- 6. 回避トリガ多数決 ----
        avoid_vote = 1 if R_center >= self.config.r_avoid_on else 0
        self._avoid_votes.append(avoid_vote)
        if len(self._avoid_votes) > self.config.vote_window:
            self._avoid_votes.pop(0)
        voted_avoid = sum(self._avoid_votes) >= self.config.votes_needed

        # ---- 7. コマンド生成（連続制御 ＋ 回避シーケンス統合）----
        cmd = self._generate_command(R_left, R_center, R_right, voted_avoid)

        rospy.loginfo(
            "[ObstAvoid] RL=%.2f RC=%.2f RR=%.2f phase=%s cmd=%s",
            R_left, R_center, R_right, self._phase.name, cmd.command.name,
        )
        self.log_metric(
            metrics={
                "R_left": R_left,
                "R_center": R_center,
                "R_right": R_right,
                "phase": self._phase.name,
            },
            tag="execute",
        )

        self._frame_index += 1
        return cmd

    def check_transition(
        self,
        imu_data: Optional[IMUData],
        camera_data: Optional[CameraData],
        markers: List[ArUcoMarker],
    ) -> Optional[StateType]:
        """状態遷移判定（本状態では常に None を返して状態維持）."""
        return None

    def exit(self) -> None:
        rospy.loginfo("[ObstAvoid] Exiting")

    # ----------------------------------------------------------------
    # ROI 抽出
    # ----------------------------------------------------------------

    def _extract_roi(self, depth: np.ndarray) -> Optional[np.ndarray]:
        """設定された割合で ROI を抽出する.

        Returns:
            ROI 配列（有効値は mm, 無効は NaN）または None
        """
        cfg = self.config
        h, w = depth.shape[:2]

        r0 = int(h * cfg.v_min_frac)
        r1 = int(h * cfg.v_max_frac)
        c0 = int(w * cfg.u_min_frac)
        c1 = int(w * cfg.u_max_frac)

        r0, r1 = max(0, r0), min(h, r1)
        c0, c1 = max(0, c0), min(w, c1)

        if r1 <= r0 or c1 <= c0:
            return None

        roi = depth[r0:r1, c0:c1].astype(np.float32)

        # 無効値 (0, NaN, 範囲外) を NaN に統一
        valid_mask = (
            np.isfinite(roi)
            & (roi >= cfg.depth_min_mm)
            & (roi <= cfg.depth_max_mm)
        )
        roi = np.where(valid_mask, roi, np.nan)
        return roi

    # ----------------------------------------------------------------
    # 9セル解析
    # ----------------------------------------------------------------

    def _compute_cell_stats(self, depth: np.ndarray) -> Dict[str, Tuple[float, float]]:
        """depth から指定 ROI 9 領域を抽出し, 各セルの (中央値[mm], 有効画素率) を返す.

        最適化:
          - 有効画素マスク (isfinite & > 0) を画像全体で 1 回だけ計算し
            9 セルへのスライスで再利用（9 回分の反復処理を削減）.
          - セル内の有効画素数は boolean mask の sum() で求め
            中間の 1D 配列生成を最小化する.
        """
        h, w = depth.shape[:2]

        # ---- 有効マスクを全画像で一括計算（最重要最適化）----
        valid_mask = np.isfinite(depth) & (depth > 0)

        # ---- セル境界を定数テーブルとして定義 ----
        #  行: [h//9*1, h//9*3] / [h//9*3, h//9*6] / [h//9*6, h//9*8]
        #  列: [w//12*2, w//12*4] / [w//12*4, w//12*8] / [w//12*8, w//12*10]
        row_bounds = (
            (h // 10 * 1, h // 10 * 3),
            (h // 10 * 4, h // 10 * 6),
            (h // 10 * 7, h // 10 * 9),
        )
        col_bounds = (
            (w // 10 * 2,  w // 10 * 4),
            (w // 10 * 5,  w // 10 * 7),
            (w // 10 * 8,  w // 10 * 10),
        )
        cell_names = (
            ("ul", "uc", "ur"),
            ("ml", "mc", "mr"),
            ("ll", "lc", "lr"),
        )

        stats: Dict[str, Tuple[float, float]] = {}
        for ri, (r0, r1) in enumerate(row_bounds):
            for ci, (c0, c1) in enumerate(col_bounds):
                name  = cell_names[ri][ci]
                total = (r1 - r0) * (c1 - c0)
                if total == 0:
                    stats[name] = (np.nan, 0.0)
                    continue

                # valid_mask スライスはビューなのでコピー不要
                cell_valid = valid_mask[r0:r1, c0:c1]
                n_valid    = int(cell_valid.sum())   # bool sum: 中間配列不要

                if n_valid == 0:
                    stats[name] = (np.nan, 0.0)
                    continue

                p_valid  = n_valid / total
                # 有効画素だけを 1 回のブール索引で抽出して median を計算
                median_d = float(np.median(depth[r0:r1, c0:c1][cell_valid]))
                stats[name] = (median_d, p_valid)

        return stats

    # ----------------------------------------------------------------
    # 距離 → 危険度
    # ----------------------------------------------------------------

    def _dist_to_risk(self, d_mm: float, p_valid: float) -> float:
        """距離[mm] と有効画素率から危険度 r ∈ [0, 1] を返す.

        有効画素率が低い（欠損が多い）場合は保守的に 1.0 とする.
        """
        cfg = self.config

        # 欠損が多いセルは保守的対応
        if p_valid < cfg.p_min:
            return 1.0

        # d が NaN の場合も保守的対応
        if not np.isfinite(d_mm):
            return 1.0

        denom = cfg.d_safe - cfg.d_stop
        if denom <= 0:
            denom = 1.0

        r_raw = (cfg.d_safe - d_mm) / denom
        return float(np.clip(r_raw, 0.0, 1.0))

    # ----------------------------------------------------------------
    # リスク統合
    # ----------------------------------------------------------------

    def _aggregate_risks(
        self, risks: Dict[str, float]
    ) -> Tuple[float, float, float]:
        """9セル危険度を左・中央・右リスクへ重み付き統合する.

        重みは下段 > 中段 > 上段（cfg.w_lower > cfg.w_middle > cfg.w_upper）.

        Returns:
            (R_left, R_center, R_right) — 正規化済み [0, 1]
        """
        cfg = self.config
        wu, wm, wl = cfg.w_upper, cfg.w_middle, cfg.w_lower
        weight_sum = wu + wm + wl

        def _col(key_u: str, key_m: str, key_l: str) -> float:
            val = (
                wu * risks.get(key_u, 1.0)
                + wm * risks.get(key_m, 1.0)
                + wl * risks.get(key_l, 1.0)
            ) / weight_sum
            return float(np.clip(val, 0.0, 1.0))

        R_left   = _col("ul", "ml", "ll")
        R_center = _col("uc", "mc", "lc")
        R_right  = _col("ur", "mr", "lr")

        return R_left, R_center, R_right

    # ----------------------------------------------------------------
    # コマンド生成（連続制御 ＋ 回避シーケンス統合）
    # ----------------------------------------------------------------

    def _generate_command(
        self,
        R_left: float,
        R_center: float,
        R_right: float,
        voted_avoid: bool,
    ) -> ControlCommand:
        """フェーズに応じてコマンドを生成する.

        NORMAL: 前進方向のみの連続制御（ch3 >= 1500, チャタリングなし）
                voted_avoid=True になった瞬間に BACK へ遷移.
        BACK  : 固定フレーム後退（ch3=1300, ch1=1500）.
                back_frames 経過後 TURN へ遷移.
        TURN  : 固定フレーム旋回（ch3=1500, ch1=1200 or 1800）.
                turn_frames 経過後 NORMAL へ遷移（vote バッファをリセット）.
        """
        cfg = self.config

        # ================================================================
        # BACK フェーズ
        # ================================================================
        if self._phase == AvoidPhase.BACK:
            self._phase_counter += 1
            if self._phase_counter >= cfg.back_frames:
                self._turn_dir = 1 if R_left >= R_right else -1  # 危険が小さい側へ
                self._transition_phase(AvoidPhase.TURN)
            pwm_ch3 = int(np.clip(cfg.pwm_center - cfg.A_back, cfg.pwm_min, cfg.pwm_max))
            return ControlCommand(CommandType.MOVE_BACKWARD, pwm_ch3)

        # ================================================================
        # TURN フェーズ
        # ================================================================
        if self._phase == AvoidPhase.TURN:
            self._phase_counter += 1
            if self._phase_counter >= cfg.turn_frames:
                self._transition_phase(AvoidPhase.NORMAL)
                # NORMAL 復帰時に vote バッファをリセット（即再トリガを防ぐ）
                self._avoid_votes = []
            pwm_ch1 = int(np.clip(
                cfg.pwm_center + self._turn_dir * cfg.A_turn,
                cfg.pwm_min, cfg.pwm_max,
            ))
            if self._turn_dir > 0:
                return ControlCommand(CommandType.TURN_RIGHT, pwm_ch1)
            else:
                return ControlCommand(CommandType.TURN_LEFT, pwm_ch1)

        # ================================================================
        # NORMAL フェーズ
        # ================================================================

        # --- 回避トリガ判定 → BACK へ遷移 ---
        if voted_avoid:
            rospy.logwarn(
                "[ObstAvoid] Avoid triggered (RC=%.2f) → BACK", R_center
            )
            self._transition_phase(AvoidPhase.BACK)
            pwm_ch3 = int(np.clip(cfg.pwm_center - cfg.A_back, cfg.pwm_min, cfg.pwm_max))
            return ControlCommand(CommandType.MOVE_BACKWARD, pwm_ch3)

        # --- 連続制御（前進方向のみ, ch3 は 1500 未満に下げない）---
        # ch1: 1500 + A1 * t  （右リスク > 左リスク → 左旋回 t<0, 左リスク > 右リスク → 右旋回 t>0）
        # ch3: R_center=0 → 最大前進(1800), R_center=1 → 停止(1500) ← 後退しない
        t = float(np.clip(cfg.kt * (R_left - R_right), -1.0, 1.0))
        s = float(np.clip(1.0 - R_center, 0.0, 1.0))  # [0,1], R_center高→s小

        ch1 = int(np.clip(cfg.pwm_center + cfg.A1 * t, cfg.pwm_min, cfg.pwm_max))
        # ch3: s を [0.5, 1] にリマップして前進のみの範囲へ写像
        #   s=1 → ch3=pwm_center+A3=1800（最大前進）
        #   s=0 → ch3=pwm_center=1500（停止）← 後退しない
        ch3_raw = cfg.pwm_center + cfg.A3 * s  # [1500, 1800]
        ch3 = int(np.clip(ch3_raw, cfg.ch3_forward_min, cfg.pwm_max))

        tt = 0.15  # 旋回ありとみなす |t| 閾値
        if ch3 > cfg.pwm_center:
            # 前進 + 旋回
            if t > tt:
                return ControlCommand(CommandType.FORWARD_RIGHT, ch1)
            elif t < -tt:
                return ControlCommand(CommandType.FORWARD_LEFT, ch1)
            else:
                return ControlCommand(CommandType.MOVE_FORWARD, ch3)
        else:
            # ch3 == pwm_center（停止）: ch1 旋回のみ
            if t > tt:
                return ControlCommand(CommandType.TURN_RIGHT, ch1)
            elif t < -tt:
                return ControlCommand(CommandType.TURN_LEFT, ch1)
            else:
                return ControlCommand(CommandType.STOP)

    def _transition_phase(self, new_phase: AvoidPhase) -> None:
        """フェーズ遷移ヘルパー（カウンタリセット）."""
        rospy.loginfo(
            "[ObstAvoid] Phase: %s → %s", self._phase.name, new_phase.name
        )
        self._phase = new_phase
        self._phase_counter = 0

    # ----------------------------------------------------------------
    # 描画ヘルパー
    # ----------------------------------------------------------------

    def _render_roi(self, roi: np.ndarray) -> Optional[np.ndarray]:
        """ROI depth 画像をカラーマップ変換して返す."""
        valid = np.isfinite(roi) & (roi > 0)
        if not np.any(valid):
            h, w = roi.shape[:2]
            return np.zeros((h, w, 3), dtype=np.uint8)
        v_min = float(np.min(roi[valid]))
        v_max = float(np.max(roi[valid]))
        if v_max <= v_min:
            v_max = v_min + 1e-6
        vis = np.zeros(roi.shape, dtype=np.float32)
        vis[valid] = (roi[valid] - v_min) / (v_max - v_min) * 255.0
        vis = np.clip(vis, 0, 255).astype(np.uint8)
        return cv2.applyColorMap(vis, cv2.COLORMAP_JET)

    def _render_risk_grid(
        self, risks: Dict[str, float], cell_size: int = 80
    ) -> Optional[np.ndarray]:
        """9セル危険度グリッドを画像として返す.

        危険度が高い（1.0）ほど赤, 低い（0.0）ほど青で塗りつぶし.
        中央に危険度の数値を表示する.
        """
        row_labels = ("u", "m", "l")
        col_labels = ("l", "c", "r")

        h_img = cell_size * 3
        w_img = cell_size * 3
        img = np.zeros((h_img, w_img, 3), dtype=np.uint8)

        for ri, rl in enumerate(row_labels):
            for ci, cl in enumerate(col_labels):
                name = rl + cl
                r_val = float(risks.get(name, 0.0))
                # 危険度 → BGR（低=青, 高=赤）
                blue  = int((1.0 - r_val) * 255)
                red   = int(r_val * 255)
                color = (blue, 0, red)

                y0 = ri * cell_size
                y1 = y0 + cell_size
                x0 = ci * cell_size
                x1 = x0 + cell_size
                img[y0:y1, x0:x1] = color

                # セル枠
                cv2.rectangle(img, (x0, y0), (x1 - 1, y1 - 1), (200, 200, 200), 1)

                # 数値ラベル
                label = f"{r_val:.2f}"
                org = (x0 + cell_size // 2 - 20, y0 + cell_size // 2 + 5)
                cv2.putText(
                    img, label, org,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA,
                )
                # セル名
                name_org = (x0 + 4, y0 + 16)
                cv2.putText(
                    img, name, name_org,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA,
                )

        return img
