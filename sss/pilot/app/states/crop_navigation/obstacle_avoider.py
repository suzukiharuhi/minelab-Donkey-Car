"""障害物回避器 (ObstacleAvoider).

ObstacleDetector が算出した cell_stats（9セルの中央値・有効画素率）を受け取り,
以下のステップで ControlCommand を生成する.

    1. 距離 → 危険度  : _dist_to_risk()
    2. 列リスク統合   : _aggregate_risks() → R_left, R_center, R_right
    3. 回避フェーズ管理: NORMAL → BACK → TURN の遷移で回避コマンド生成

設計方針:
    - フェーズ状態を内部に保持（ステートフル）
    - reset() を enter() 時に呼び出すこと
    - BaseState には依存せず単独テスト可能
"""
from __future__ import annotations

from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import numpy as np
import rospy

from pilot.app.core.commands import CommandType, ControlCommand
from .config import AvoidanceConfig


# ---------------------------------------------------------------------------
# 回避フェーズ定義
# ---------------------------------------------------------------------------

class AvoidPhase(Enum):
    """障害物回避フェーズ.

    NORMAL: 通常走行（連続制御）
    BACK:   固定フレーム後退（チャタリング抑制）
    TURN:   固定フレーム旋回（危険が少ない側へ）
    """
    NORMAL = auto()
    BACK   = auto()
    TURN   = auto()


# ---------------------------------------------------------------------------
# 回避器クラス
# ---------------------------------------------------------------------------

class ObstacleAvoider:
    """危険度ベースの障害物回避器.

    cell_stats を受け取り, NORMAL/BACK/TURN フェーズを管理しながら
    ControlCommand を返す.

    フェーズ遷移:
        NORMAL ──(voted_avoid)──▶ BACK ──(back_frames)──▶ TURN ──(turn_frames)──▶ NORMAL
    """

    def __init__(self, config: Optional[AvoidanceConfig] = None) -> None:
        self.config = config or AvoidanceConfig()
        self._phase:         AvoidPhase  = AvoidPhase.NORMAL
        self._phase_counter: int         = 0
        self._avoid_votes:   List[int]   = []
        self._turn_dir:      int         = 1   # +1=右, -1=左

    # ----------------------------------------------------------------
    # パブリックインタフェース
    # ----------------------------------------------------------------

    @property
    def phase(self) -> AvoidPhase:
        return self._phase

    def reset(self) -> None:
        """状態をリセットする（enter() 時に呼ぶこと）."""
        self._phase         = AvoidPhase.NORMAL
        self._phase_counter = 0
        self._avoid_votes   = []
        self._turn_dir      = 1

    def compute_command(
        self,
        cell_stats: Dict[str, Tuple[float, float]],
    ) -> ControlCommand:
        """cell_stats から回避コマンドを生成する.

        Args:
            cell_stats: {セル名: (中央値[mm], 有効画素率)} — DetectionResult.cell_stats と同形式

        Returns:
            ControlCommand
        """
        # Step 1: 距離 → 危険度
        risks = {
            name: self._dist_to_risk(d_mm, p_valid)
            for name, (d_mm, p_valid) in cell_stats.items()
        }

        # Step 2: 列リスク統合
        R_left, R_center, R_right = self._aggregate_risks(risks)

        # Step 3: 多数決で回避トリガ判定
        vote = 1 if R_center >= self.config.r_avoid_on else 0
        self._avoid_votes.append(vote)
        if len(self._avoid_votes) > self.config.vote_window:
            self._avoid_votes.pop(0)
        voted_avoid = sum(self._avoid_votes) >= self.config.votes_needed

        rospy.loginfo_throttle(
            1.0,
            "[Avoider] RL=%.2f RC=%.2f RR=%.2f phase=%s voted=%s",
            R_left, R_center, R_right, self._phase.name, voted_avoid,
        )

        # Step 4: コマンド生成（フェーズ切り替え込み）
        return self._generate_command(R_left, R_center, R_right, voted_avoid)

    # ----------------------------------------------------------------
    # Step 1: 距離 → 危険度
    # ----------------------------------------------------------------

    def _dist_to_risk(self, d_mm: float, p_valid: float) -> float:
        """距離[mm] と有効画素率から危険度 r ∈ [0, 1] を返す.

        有効画素率が低い（欠損が多い）場合は保守的に 1.0 とする.
        危険度は線形補間: d_stop 以下 → 1.0, d_safe 以上 → 0.0.
        """
        cfg = self.config

        if p_valid < cfg.p_min:
            return 1.0
        if not np.isfinite(d_mm):
            return 1.0

        denom = cfg.d_safe - cfg.d_stop
        denom = denom if denom > 0 else 1.0
        return float(np.clip((cfg.d_safe - d_mm) / denom, 0.0, 1.0))

    # ----------------------------------------------------------------
    # Step 2: 列リスク統合（左・中央・右）
    # ----------------------------------------------------------------

    def _aggregate_risks(
        self, risks: Dict[str, float]
    ) -> Tuple[float, float, float]:
        """9セル危険度を左・中央・右リスクへ重み付き統合する.

        重みは下段 > 中段 > 上段（前方路面に近いほど重要）.

        Returns:
            (R_left, R_center, R_right) — [0, 1] に正規化済み
        """
        cfg = self.config
        wu, wm, wl = cfg.w_upper, cfg.w_middle, cfg.w_lower
        ws = wu + wm + wl

        def _col(key_u: str, key_m: str, key_l: str) -> float:
            return float(np.clip(
                (wu * risks.get(key_u, 1.0)
                 + wm * risks.get(key_m, 1.0)
                 + wl * risks.get(key_l, 1.0)) / ws,
                0.0, 1.0,
            ))

        R_left   = _col("ul", "ml", "ll")
        R_center = _col("uc", "mc", "lc")
        R_right  = _col("ur", "mr", "lr")
        return R_left, R_center, R_right

    # ----------------------------------------------------------------
    # Step 3: フェーズ管理 & コマンド生成
    # ----------------------------------------------------------------

    def _generate_command(
        self,
        R_left:      float,
        R_center:    float,
        R_right:     float,
        voted_avoid: bool,
    ) -> ControlCommand:
        """フェーズに応じてコマンドを生成する.

        NORMAL:
            voted_avoid=False → 連続制御（ch3 >= 1500, 後退しない）.
            voted_avoid=True  → BACK フェーズへ遷移.
        BACK:
            back_frames 経過後 TURN へ遷移（旋回方向は危険が少ない側）.
        TURN:
            turn_frames 経過後 NORMAL へ遷移（vote バッファをリセット）.
        """
        cfg = self.config

        # ================================================================
        # BACK フェーズ
        # ================================================================
        if self._phase == AvoidPhase.BACK:
            self._phase_counter += 1
            if self._phase_counter >= cfg.back_frames:
                # 左右リスクが低い方向へ旋回
                self._turn_dir = 1 if R_left >= R_right else -1
                self._transition(AvoidPhase.TURN)
            pwm = int(np.clip(cfg.pwm_center - cfg.A_back, cfg.pwm_min, cfg.pwm_max))
            return ControlCommand(CommandType.MOVE_BACKWARD, pwm)

        # ================================================================
        # TURN フェーズ
        # ================================================================
        if self._phase == AvoidPhase.TURN:
            self._phase_counter += 1
            if self._phase_counter >= cfg.turn_frames:
                self._transition(AvoidPhase.NORMAL)
                self._avoid_votes = []   # 即再トリガを防ぐ
            pwm = int(np.clip(
                cfg.pwm_center + self._turn_dir * cfg.A_turn,
                cfg.pwm_min, cfg.pwm_max,
            ))
            cmd_type = CommandType.TURN_RIGHT if self._turn_dir > 0 else CommandType.TURN_LEFT
            return ControlCommand(cmd_type, pwm)

        # ================================================================
        # NORMAL フェーズ
        # ================================================================

        # 回避トリガ → BACK へ遷移
        if voted_avoid:
            rospy.logwarn("[Avoider] Avoid triggered (RC=%.2f) → BACK", R_center)
            self._transition(AvoidPhase.BACK)
            pwm = int(np.clip(cfg.pwm_center - cfg.A_back, cfg.pwm_min, cfg.pwm_max))
            return ControlCommand(CommandType.MOVE_BACKWARD, pwm)

        # 連続制御（前進方向のみ, ch3 は pwm_center 未満に下げない）
        #   t: 左右リスク差 → 旋回量 [-1, 1]
        #   s: 中央リスクから前進量 [0, 1]
        t = float(np.clip(cfg.kt * (R_left - R_right), -1.0, 1.0))
        s = float(np.clip(1.0 - R_center, 0.0, 1.0))

        ch1 = int(np.clip(cfg.pwm_center + cfg.A1 * t, cfg.pwm_min, cfg.pwm_max))
        ch3 = int(np.clip(cfg.pwm_center + cfg.A3 * s, cfg.ch3_forward_min, cfg.pwm_max))

        tt = 0.05   # 旋回ありとみなす |t| 閾値
        if ch3 > cfg.pwm_center:
            if t > tt:
                return ControlCommand(CommandType.FORWARD_RIGHT, ch1)
            elif t < -tt:
                return ControlCommand(CommandType.FORWARD_LEFT, ch1)
            else:
                return ControlCommand(CommandType.MOVE_FORWARD, ch3)
        else:
            if t > tt:
                return ControlCommand(CommandType.TURN_RIGHT, ch1)
            elif t < -tt:
                return ControlCommand(CommandType.TURN_LEFT, ch1)
            else:
                return ControlCommand(CommandType.STOP)

    def _transition(self, new_phase: AvoidPhase) -> None:
        """フェーズ遷移ヘルパー（ログ出力 & カウンタリセット）."""
        rospy.loginfo("[Avoider] Phase: %s → %s", self._phase.name, new_phase.name)
        self._phase         = new_phase
        self._phase_counter = 0
