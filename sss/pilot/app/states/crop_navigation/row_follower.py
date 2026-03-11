"""作物列追従器 (RowFollower / FTG-i).

FTG-i (Follow The Gap improved) アルゴリズムを実装し,
depth image から作物列の中央方向を求めて ControlCommand を生成する.

処理フロー:
    1. _build_1d_range()     : depth image → 1D レンジプロファイル (K サンプル)
    2. _extract_gaps()       : 1D レンジから自由空間ギャップを抽出
    3. _select_best_heading(): 評価関数 U(θ) で最良ギャップ方位を選択
    4. _heading_to_command() : 方位 → ControlCommand (PWM 値)

設計方針:
    - θ_prev のみ内部状態として保持（ステートフル）
    - reset() を enter() 時に呼び出すこと
    - BaseState には依存せず単独テスト可能
    - 描画ヘルパーは get_debug_images() でまとめて取得可能
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np
import rospy

from pilot.app.core.commands import CommandType, ControlCommand
from .config import RowFollowerConfig


class RowFollower:
    """FTG-i による作物列中央走行器.

    depth image (2D) を受け取り, 最良ギャップ方位に向かう ControlCommand を返す.
    前フレームの舵角 θ_prev を保持し, 舵角変化ペナルティに使用する.
    """

    def __init__(self, config: Optional[RowFollowerConfig] = None) -> None:
        self.config       = config or RowFollowerConfig()
        self._theta_prev  = 0.0   # 前フレームの選択角度
        # compute_command() の計算結果キャッシュ（重複計算防止）
        # get_debug_images() はこのキャッシュを使う
        self._debug_cache: Optional[Tuple] = None

    # ----------------------------------------------------------------
    # パブリックインタフェース
    # ----------------------------------------------------------------

    def reset(self) -> None:
        """状態をリセットする（enter() 時に呼ぶこと）."""
        self._theta_prev  = 0.0
        self._debug_cache = None

    def compute_command(
        self, depth: np.ndarray
    ) -> Tuple[Optional[ControlCommand], float]:
        """depth image から行走行コマンドを生成する.

        計算した中間結果（d_1d, roi, gaps, gap_infos）は _debug_cache に保存する．
        get_debug_images() はこのキャッシュを再利用するため _build_1d_range() の
        重複実行が発生しない．

        Returns:
            (ControlCommand または None, theta_best)
            None の場合は呼び出し側で STOP など代替コマンドを使うこと.
        """
        self._debug_cache = None  # 計算失敗時に古いキャッシュが残らないようクリア

        d_1d, _min_dist, roi = self._build_1d_range(depth)
        if d_1d is None or d_1d.size == 0:
            return None, 0.0

        gaps = self._extract_gaps(d_1d)
        if not gaps:
            rospy.logwarn_throttle(0.5, "[RowFollower] no free gaps detected")
            return None, 0.0

        theta_best, idx_best, gap_infos = self._select_best_heading(d_1d, gaps)
        self._theta_prev = theta_best

        # キャッシュ保存（配列はコピーして独立させる）
        self._debug_cache = (
            d_1d.copy(),
            roi.copy() if roi is not None else None,
            gaps,
            gap_infos,
            theta_best,
            idx_best,
        )

        cmd = self._heading_to_command(theta_best)
        return cmd, theta_best

    def get_debug_images(
        self,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """デバッグ用画像をキャッシュから生成する．

        compute_command() が計算した結果（d_1d, roi, gaps）を再利用するため，
        _build_1d_range() / _extract_gaps() / _select_best_heading() の
        再実行は発生しない．

        compute_command() を呼ぶ前に呼び出した場合は (None, None, None) を返す．

        Returns:
            (roi_vis, heatmap_1d, gap_vis)
        """
        if self._debug_cache is None:
            return None, None, None

        d_1d, roi, gaps, gap_infos, theta_best, idx_best = self._debug_cache

        roi_vis = self._render_roi(roi) if roi is not None else None
        heatmap = self._render_1d_heatmap(d_1d) if d_1d is not None else None
        gap_vis = (
            self._render_gap_debug(d_1d, gaps, gap_infos, theta_best, idx_best)
            if gaps else None
        )
        return roi_vis, heatmap, gap_vis

    # ----------------------------------------------------------------
    # 1. depth → 1D レンジ変換
    # ----------------------------------------------------------------

    def _build_1d_range(
        self, depth: np.ndarray,
    ) -> Tuple[Optional[np.ndarray], float, Optional[np.ndarray]]:
        """depth image から 1D レンジプロファイルを生成する.

        ROI を切り出し, 水平 K サンプルの列中央値を取って 1D 配列を作る.

        Returns:
            d_1d:     shape=(K,) の 1D 距離[mm] 配列（NaN 含む場合あり）
            min_dist: ROI 内の最近距離[mm]（有効値がなければ inf）
            roi:      切り出した ROI depth 画像（不変スライス）
        """
        cfg = self.config
        h, w = depth.shape[:2]

        # ROI 境界
        r0 = max(0, min(int(h * cfg.v_min_frac), h))
        r1 = max(0, min(int(h * cfg.v_max_frac), h))
        c0 = max(0, min(int(w * cfg.u_min_frac), w))
        c1 = max(0, min(int(w * cfg.u_max_frac), w))

        if r1 <= r0 or c1 <= c0:
            return None, float("inf"), None

        roi = depth[r0:r1, c0:c1]
        roi = np.where(np.isfinite(roi) & (roi > 0.0), roi, np.nan)

        # 最近距離（下位 1% を除去して外れ値を抑制）
        if np.any(np.isfinite(roi)):
            try:
                flat = roi[np.isfinite(roi)].ravel()
                p1   = np.percentile(flat, 1.0)
                min_dist = float(np.min(flat[flat >= p1]))
            except Exception:
                min_dist = float("inf")
        else:
            min_dist = float("inf")

        # K 列を等間隔サンプリングして列中央値を計算
        num_cols      = roi.shape[1]
        K             = min(cfg.num_samples, num_cols)
        sample_idx    = np.linspace(0, num_cols - 1, K).astype(np.int32)
        roi_sampled   = roi[:, sample_idx]                  # (H, K)
        d_1d          = np.nanmedian(roi_sampled, axis=0)   # (K,)

        # rospy.loginfo_throttle(
        #     2.0,
        #     "[RowFollower] ROI=(%d:%d,%d:%d) K=%d min=%.1f med=%.1f max=%.1f",
        #     r0, r1, c0, c1, d_1d.size,
        #     float(np.nanmin(d_1d)), float(np.nanmedian(d_1d)), float(np.nanmax(d_1d)),
        # )
        return d_1d, min_dist, roi

    # ----------------------------------------------------------------
    # 2. ギャップ抽出
    # ----------------------------------------------------------------

    def _extract_gaps(
        self, d_1d: np.ndarray
    ) -> List[Tuple[int, int, int]]:
        """1D レンジから自由空間ギャップを抽出する.

        Returns:
            [(start_idx, end_idx, center_idx), ...] （end は inclusive）
        """
        cfg   = self.config
        d_safe = np.nan_to_num(d_1d, nan=0.0, posinf=0.0, neginf=0.0)
        free   = d_safe >= cfg.d_free

        K         = d_safe.size
        min_width = max(cfg.min_gap_width_pixels,
                        int(cfg.min_gap_width_ratio * K))

        gaps: List[Tuple[int, int, int]] = []
        i = 0
        while i < K:
            if not free[i]:
                i += 1
                continue
            start = i
            while i + 1 < K and free[i + 1]:
                i += 1
            end = i
            if end - start + 1 >= min_width:
                gaps.append((start, end, (start + end) // 2))
            i += 1

        return gaps

    # ----------------------------------------------------------------
    # 3. 方位選択（評価関数 U(θ)）
    # ----------------------------------------------------------------

    def _select_best_heading(
        self,
        d_1d: np.ndarray,
        gaps: List[Tuple[int, int, int]],
    ) -> Tuple[float, int, List[Dict]]:
        """評価関数 U(θ) = Σ w_i * score_i を最大にする方位を選択する.

        4 項:
            (A) Clearance   : ギャップ中心の距離が大きいほど良い
            (B) Centering   : 画像中央に近いほど良い
            (C) Goal heading: 現状ゴール無し（w_goal=0）
            (D) Steer smooth: 前回 θ_prev との差が小さいほど良い

        Returns:
            theta_best: 正規化方位 [-1, 1]
            idx_best:   1D 配列インデックス
            gap_infos:  各ギャップの評価詳細リスト
        """
        cfg   = self.config
        K     = d_1d.size
        mid   = (K - 1) / 2.0

        theta_best = 0.0
        idx_best   = int(mid)
        u_best     = -float("inf")
        gap_infos: List[Dict] = []

        for start, end, center in gaps:
            d_c = d_1d[center]
            if not np.isfinite(d_c) or d_c <= 0.0:
                continue

            theta = (center - mid) / max(mid, 1.0)   # [-1, 1]

            # (A) Clearance: 距離が大きいほど良い
            clearance = float(np.clip(d_c / cfg.clearance_norm, 0.0, 1.0))

            # (B) Centering: 画像中央に近いほど良い
            centering = 1.0 - float(np.clip(abs(center - mid) / max(mid, 1.0), 0.0, 1.0))

            # (C) Goal heading: ε_goal = |θ - 0| （目標は直進）
            goal_score = 1.0 - float(np.clip(abs(theta), 0.0, 1.0))

            # (D) Steer smooth: 前回方位との差を抑制
            steer_score = 1.0 - float(np.clip(abs(theta - self._theta_prev), 0.0, 1.0))

            u = (
                cfg.w_clearance    * clearance
                + cfg.w_centering  * centering
                + cfg.w_goal       * goal_score
                + cfg.w_steer_smooth * steer_score
            )

            gap_infos.append({
                "start": start, "end": end, "center": center,
                "theta": float(theta),
                "clearance": clearance, "centering": centering,
                "goal": goal_score, "steer": steer_score,
                "u": float(u),
            })

            if u > u_best:
                u_best     = u
                theta_best = float(theta)
                idx_best   = int(center)

        return theta_best, idx_best, gap_infos

    # ----------------------------------------------------------------
    # 4. 方位 → ControlCommand 変換
    # ----------------------------------------------------------------

    def _heading_to_command(self, theta: float) -> ControlCommand:
        """正規化方位 θ から ControlCommand を生成する.

        θ=0 が直進中央, θ>0 が右寄り, θ<0 が左寄り.
        デッドゾーン内は直進（MOVE_FORWARD）.
        デッドゾーン外は比例制御で PWM を算出する.
        """
        cfg = self.config
        error = float(np.clip(theta, -1.0, 1.0))
        e = 0.0 - error   # 中央（θ=0）を追いかける誤差
        print("theta:", theta, "error:", error, "e:", e)   # --- IGNORE ---

        # デッドゾーン内 → 直進
        if abs(e) <= cfg.theta_dead_zone:
            return ControlCommand(CommandType.MOVE_FORWARD, cfg.pwm_forward)

        Kp = 250.0   # 比例ゲイン（要調整）
        if e < 0: #左に壁，右旋回 (e < 0 なので Kp*e は負)
            pwm = int(cfg.pwm_u_min - Kp * e)
        else: #右に壁，左旋回（e > 0 なので Kp*e は正）
            pwm = int(cfg.pwm_l_min - Kp * e)

        pwm = max(cfg.pwm_min, min(cfg.pwm_max, pwm))  # クリップ

        if pwm < cfg.pwm_center:
            return ControlCommand(CommandType.FORWARD_LEFT, pwm)
        elif pwm > cfg.pwm_center:
            return ControlCommand(CommandType.FORWARD_RIGHT, pwm)
        else:
            return ControlCommand(CommandType.MOVE_FORWARD, cfg.pwm_forward)

    # ----------------------------------------------------------------
    # 描画ヘルパー
    # ----------------------------------------------------------------

    def _render_roi(self, roi: np.ndarray) -> Optional[np.ndarray]:
        """ROI depth 画像をカラーマップ (JET) 変換した BGR 画像を返す."""
        try:
            import cv2
        except ImportError:
            return None

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

    def _render_1d_heatmap(
        self,
        d_1d:   np.ndarray,
        d_min:  float = 100.0,   # [mm] 正規化下限
        d_max:  float = 5000.0,  # [mm] 正規化上限
        height: int   = 80,
    ) -> Optional[np.ndarray]:
        """1D レンジプロファイルをヒートマップ画像 (height × K) として返す."""
        try:
            import cv2
        except ImportError:
            return None

        if d_1d is None or d_1d.size == 0:
            return None

        vals = d_1d.astype(np.float32)
        mask = np.isfinite(vals) & (vals > 0.0)
        if not np.any(mask):
            return None

        denom = float(d_max - d_min)
        if denom <= 0:
            return None

        norm = np.zeros_like(vals)
        norm[mask] = np.clip((vals[mask] - d_min) / denom, 0.0, 1.0)

        row      = (norm * 255.0).astype(np.uint8)[np.newaxis, :]   # (1, K)
        img_gray = np.repeat(row, height, axis=0)                    # (height, K)
        return cv2.applyColorMap(img_gray, cv2.COLORMAP_JET)

    def _render_gap_debug(
        self,
        d_1d:       np.ndarray,
        gaps:       List[Tuple[int, int, int]],
        gap_infos:  List[Dict],
        theta_best: float,
        idx_best:   int,
    ) -> Optional[np.ndarray]:
        """ギャップ抽出・方位選択結果を 1D ヒートマップ上に描画して返す.

        - 各ギャップ区間を矩形で囲む（色: U 値に応じた青→赤グラデーション）
        - 選択されたギャップを白枠で強調
        - 右上に θ_best を表示
        """
        try:
            import cv2
        except ImportError:
            return None

        img = self._render_1d_heatmap(d_1d)
        if img is None or not gap_infos:
            return img

        height = img.shape[0]
        u_arr = np.array([info["u"] for info in gap_infos], dtype=np.float32)
        u_min = float(np.min(u_arr))
        u_max = float(np.max(u_arr))
        if u_max <= u_min:
            u_max = u_min + 1e-6

        # 各ギャップを矩形で描画
        # 色は U 値に応じた青→赤グラデーション, 中央が緑、U が大きいほど赤寄り、小さいほど青寄り
        for info in gap_infos:
            s, e2, c = int(info["start"]), int(info["end"]), int(info["center"])
            u_n = float(np.clip((info["u"] - u_min) / (u_max - u_min), 0.0, 1.0))
            color = (int(255 * u_n), int(255 * (1.0 - u_n)), 0)

            cv2.rectangle(img, (s, 0), (e2, height - 1), color, 1)
            cv2.putText(
                img, f"U={info['u']:.2f}", (c, height // 2),
                cv2.FONT_HERSHEY_PLAIN, 0.7, color, 1, cv2.LINE_AA,
            )

        # 選択ギャップを白枠で強調
        for info in gap_infos:
            if int(info["center"]) == idx_best:
                cv2.rectangle(
                    img,
                    (int(info["start"]), 0),
                    (int(info["end"]), height - 1),
                    (255, 255, 255), 2,
                )
                break

        # θ_best を描画
        cv2.putText(
            img, f"theta={theta_best:.3f}", (5, 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA,
        )
        return img
