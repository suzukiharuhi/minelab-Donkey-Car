from dataclasses import dataclass
import math

@dataclass
class TurnDecision:
    """旋回指令案

    Attributes:
        done:      旋回完了か
        direction: 'left' | 'right' | None (done=True なら None)
        slow:      True なら低速旋回を推奨
        diff_deg:  初期姿勢から現在までの累積旋回角度（正規化済み）
        remaining_deg: 目標相対角度 - diff （残り角度、符号は方向に依存）
    """
    done: bool
    direction: str | None
    slow: bool
    diff_deg: float
    remaining_deg: float


class TurnController:
    """相対角度旋回コントローラ (度単位)。

    Params:
        padding_deg:        停止遅れ見込み (早め停止余裕)。order.py の PADDING に対応
        slow_threshold_deg: 残り角度がこの値未満で低速化
        min_turn_deg:       この角度未満なら旋回不要 (ノイズ扱い)
        small_diff_zero_deg: 初期差がこの閾値未満なら diff=0 とみなす
    """

    def __init__(
        self,
        padding_deg: float = 5.0,
        slow_threshold_deg: float = 30.0,
        min_turn_deg: float = 3.0,
        small_diff_zero_deg: float = 3.0,
    ):
        self.padding_deg = padding_deg
        self.slow_threshold_deg = slow_threshold_deg
        self.min_turn_deg = min_turn_deg
        self.small_diff_zero_deg = small_diff_zero_deg

        # 内部状態
        self.active: bool = False
        self.initial_yaw_deg: float = 0.0
        self.theta_deg: float = 0.0  # 目標相対角度 (正: 左, 負: 右)
        self.effective_padding_deg: float = 0.0

    # ---- lifecycle ----
    def start(self, theta_deg: float, current_yaw_deg: float):
        """相対角度 theta_deg の旋回開始。
        Args:
            theta_deg: 正なら左旋回、負なら右旋回
            current_yaw_deg: 開始時の絶対 yaw
        """
        self.initial_yaw_deg = current_yaw_deg % 360.0
        self.theta_deg = theta_deg
        # 有効パディングは相対角度の 50% を上限
        self.effective_padding_deg = min(self.padding_deg, abs(self.theta_deg) * 0.5)
        self.active = True

    def is_active(self) -> bool:
        return self.active

    def finished(self) -> bool:
        return not self.active

    # ---- core step ----
    def step(self, current_yaw_deg: float) -> TurnDecision:
        """現在 yaw (deg) に基づき次指令を決定
        Returns:
            TurnDecision: 完了/方向/速度段階など
        """
        if not self.active:
            return TurnDecision(done=True, direction=None, slow=False, diff_deg=0.0, remaining_deg=0.0)

        current_yaw_deg = current_yaw_deg % 360.0

        # 小さな差分を 0 とみなす angle_diff ロジック (order.py 準拠)
        diff = self._angle_diff_for_signed_direction(current_yaw_deg)

        # 終了判定: diff >= 目標 - effective_padding (符号別)
        if self._should_finish(diff):
            self.active = False
            remaining = self.theta_deg - diff
            return TurnDecision(done=True, direction=None, slow=False, diff_deg=diff, remaining_deg=remaining)

        remaining = self.theta_deg - diff
        # 低速判定
        slow = abs(remaining) < self.slow_threshold_deg
        direction = 'left' if self.theta_deg > 0 else 'right'
        return TurnDecision(done=False, direction=direction, slow=slow, diff_deg=diff, remaining_deg=remaining)

    # ---- internal helpers ----
    def _angle_diff_for_signed_direction(self, current_yaw_deg: float) -> float:
        """初期角からの累積旋回角度 diff を、目標符号に合わせて正方向で返す。
        左旋回 (theta>0): diff = angle_diff(current, initial)
        右旋回 (theta<0): diff = angle_diff(initial, current)
        最初の数度はノイズ扱い (small_diff_zero_deg)。
        """
        if self.theta_deg == 0:
            return 0.0
        if self.theta_deg > 0:
            raw = self._angle_diff(current_yaw_deg, self.initial_yaw_deg)
        else:
            raw = self._angle_diff(self.initial_yaw_deg, current_yaw_deg)
        if abs(raw) < self.small_diff_zero_deg:
            return 0.0
        return raw

    def _angle_diff(self, current: float, reference: float) -> float:
        """order.py の angle_diff と同等処理 (3度未満なら0扱いは外部で)。"""
        diff = (current - reference + 360.0) % 360.0
        return diff

    def _should_finish(self, diff: float) -> bool:
        """パディングを考慮した終了判定。min_turn_deg 未満なら即完了。"""
        if abs(self.theta_deg) < self.min_turn_deg:
            return True
        # 左旋回: diff >= theta - padding, 右旋回: diff >= |theta| - padding
        target_mag = abs(self.theta_deg)
        threshold = target_mag - self.effective_padding_deg
        return diff >= threshold
