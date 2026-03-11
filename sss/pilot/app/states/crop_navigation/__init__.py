"""crop_navigation パッケージ.

作物列走行 (FTG-i) + 障害物回避を統合した状態クラスを提供する.

主な公開クラス:
    CropNavigationState  : BaseState を継承した状態クラス（メインエントリ）
    CropNavigationConfig : 全サブコンポーネントの設定コンテナ
    DetectionConfig      : 障害物検知設定
    AvoidanceConfig      : 障害物回避設定
    RowFollowerConfig    : FTG-i 列追従設定

サブコンポーネント（単独テスト可能）:
    ObstacleDetector     : ROI 9分割 → 障害物判定
    ObstacleAvoider      : 危険度ベース回避コマンド生成
    RowFollower          : FTG-i ギャップ方位選択 → コマンド生成

使用例::

    from pilot.app.states.crop_navigation import CropNavigationState

    state = CropNavigationState()
    # または設定をカスタマイズ:
    from pilot.app.states.crop_navigation import (
        CropNavigationState, CropNavigationConfig, DetectionConfig
    )
    cfg = CropNavigationConfig(detection=DetectionConfig(obstacle_threshold_mm=600.0))
    state = CropNavigationState(config=cfg)
"""

from .crop_navigation_state import CropNavigationState, NavigationMode

from .config import (
    CropNavigationConfig,
    DetectionConfig,
    AvoidanceConfig,
    RowFollowerConfig,
)

from .obstacle_detector import ObstacleDetector, DetectionResult
from .obstacle_avoider  import ObstacleAvoider, AvoidPhase
from .row_follower      import RowFollower

__all__ = [
    # 状態クラス（メインエントリ）
    "CropNavigationState",
    "NavigationMode",

    # 設定クラス
    "CropNavigationConfig",
    "DetectionConfig",
    "AvoidanceConfig",
    "RowFollowerConfig",

    # サブコンポーネント
    "ObstacleDetector",
    "DetectionResult",
    "ObstacleAvoider",
    "AvoidPhase",
    "RowFollower",
]
