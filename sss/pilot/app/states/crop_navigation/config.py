"""crop_navigation パッケージの設定クラス群.

各コンポーネント（ObstacleDetector / ObstacleAvoider / RowFollower）に対応する
設定クラスと, それらをまとめた CropNavigationConfig を定義する.
"""
from __future__ import annotations

from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# 1. 障害物検知設定
# ---------------------------------------------------------------------------

@dataclass
class DetectionConfig:
    """ObstacleDetector の設定パラメータ.

    ROI を 3×3=9 セルに分割し, 各セルの中央値距離で障害物を判定する.
    """

    # ---- ROI 設定（画像サイズに対する割合）----
    v_min_frac: float = 0.30   # 高さ方向: 上端からの割合
    v_max_frac: float = 0.65
    u_min_frac: float = 0.15   # 幅方向: 左端からの割合
    u_max_frac: float = 0.85

    # ---- 深度の有効範囲 [mm] ----
    depth_min_mm: float = 10.0
    depth_max_mm: float = 10000.0

    # ---- 有効画素率の信頼閾値 ----
    # これ未満のセルはデータ欠損とみなし, 保守的に障害物ありと判定する
    p_min: float = 0.10

    # ---- 障害物判定距離閾値 [mm] ----
    # いずれかのセルの中央値がこれ未満 → has_obstacle=True
    obstacle_threshold_mm: float = 800.0


# ---------------------------------------------------------------------------
# 2. 障害物回避設定
# ---------------------------------------------------------------------------

@dataclass
class AvoidanceConfig:
    """ObstacleAvoider の設定パラメータ.

    距離→危険度変換, フェーズ管理（NORMAL/BACK/TURN）, PWM 設定を含む.
    """

    # ---- 危険度変換 [mm] ----
    d_stop: float = 300.0    # この距離以下は危険度 1.0
    d_safe: float = 800.0    # この距離以上は危険度 0.0

    # ---- 有効画素率の信頼閾値 ----
    p_min: float = 0.10

    # ---- 列方向の重み（上段＞下段＞中段）----
    w_upper:  float = 3.0
    w_middle: float = 2.0
    w_lower:  float = 1.0

    # ---- 旋回ゲイン ----
    kt: float = 1.2   # 左右リスク差 → 旋回 t

    # ---- 回避トリガしきい値 ----
    r_avoid_on: float = 0.80

    # ---- 多数決（チャタリング抑制）----
    votes_needed: int = 2
    vote_window:  int = 3

    # ---- 回避シーケンス フレーム数 ----
    back_frames: int = 10
    turn_frames: int = 10

    # ---- PWM 設定 ----
    pwm_center:      int = 1500
    pwm_min:         int = 1100
    pwm_max:         int = 1900
    ch3_forward_min: int = 1700   # NORMAL フェーズの前進 ch3 下限（後退しない）

    A1: int = 300   # ch1（旋回）振幅; t=±1 のときのオフセット
    A3: int = 300   # ch3（推進）振幅; s=1 のときのオフセット

    A_back: int = 200   # 後退 ch3 オフセット (pwm_center - A_back)
    A_turn: int = 300   # 旋回 ch1 オフセット (pwm_center ± A_turn)


# ---------------------------------------------------------------------------
# 3. 作物列追従設定（FTG-i）
# ---------------------------------------------------------------------------

@dataclass
class RowFollowerConfig:
    """RowFollower（FTG-i）の設定パラメータ.

    depth image から 1D レンジプロファイルを生成し, ギャップ方位を選択する.
    """

    # ---- ROI 設定（画像サイズに対する割合）----
    v_min_frac: float = 0.30
    v_max_frac: float = 0.60
    u_min_frac: float = 0.20
    u_max_frac: float = 0.80

    # ---- 1D レンジのサンプル数 K ----
    num_samples: int = 300

    # ---- 1D メディアンフィルタカーネルサイズ ----
    median_filter_ksize: int = 3

    # ---- 自由空間判定しきい値 [mm] ----
    d_free: float = 3000.0

    # ---- ギャップ最小幅 ----
    min_gap_width_pixels: int = 10
    min_gap_width_ratio:  float = 0.05   # K に対する割合

    # ---- U(θ) 評価関数の重み ----
    w_clearance:    float = 0.8
    w_centering:    float = 0.9
    w_goal:         float = 0.0   # ゴール無し → 0
    w_steer_smooth: float = 2.0   # 大きいほど舵角変化を嫌う

    # ---- Clearance 正規化用スケール ----
    clearance_norm: float = 2.0

    # ---- デッドゾーン ----
    theta_dead_zone: float = 0.1   # |θ| <= これをほぼ直進とみなす

    # ---- PWM 設定 ----
    pwm_center:  int = 1500
    pwm_u_min:   int = 1600   # 右前進 PWM 下限（1650 以上推奨）
    pwm_l_min:   int = 1400   # 左前進 PWM 上限（1350 以下推奨）
    pwm_min:     int = 1100
    pwm_max:     int = 1900
    pwm_forward: int = 1750


# ---------------------------------------------------------------------------
# 4. 統合設定
# ---------------------------------------------------------------------------

@dataclass
class CropNavigationConfig:
    """CropNavigationState の統合設定コンテナ.

    未指定の場合はそれぞれデフォルト値で初期化される.
    """
    detection:    DetectionConfig    = field(default_factory=DetectionConfig)
    avoidance:    AvoidanceConfig    = field(default_factory=AvoidanceConfig)
    row_follower: RowFollowerConfig  = field(default_factory=RowFollowerConfig)
