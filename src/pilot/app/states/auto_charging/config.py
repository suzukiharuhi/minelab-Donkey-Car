from dataclasses import dataclass

@dataclass
class ChargingConfig:
    # thresholds [cm]
    approach_point_distance: float = 50.0  # ステーションから目標点Pまでの距離
    docking_threshold: float = 20.0         # marker0 z to dock
    
    miss_limit: int = 5 # 途中でマーカーを見失った時の許容回数

    retry_back_ticks: int = 60             # ~2s at 30Hz
    turn_ticks_small: int = 20             # small nudge turns
