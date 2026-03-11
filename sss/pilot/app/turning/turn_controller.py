from dataclasses import dataclass
from typing import Optional


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
    direction: Optional[str]
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
        # 3度未満は0扱い（order.py 準拠）
        if abs(raw) < self.small_diff_zero_deg:
            return 0.0
        # ラップ誤差対策: 360度に極めて近い値は 0 とみなす（例: 359.x）
        if abs(360.0 - raw) < self.small_diff_zero_deg:
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


@dataclass
class SmallTurnDecision:
    """小角度用パルス旋回の指令

    Attributes:
        done:        旋回完了か
        direction:   'left' | 'right' | None（待機 or 完了）
        slow:        低速推奨か（小角度なので通常 True の想定）
        error_deg:   現在 yaw から見た目標までの符号付き誤差 (target - current)
        phase:       'pulse'(旋回) | 'wait'(停止待機) | 'done'(完了)
    """
    done: bool
    direction: Optional[str]
    slow: bool
    error_deg: float
    phase: str


class SmallAngleTurnController:
    """小角度調整用のパルス旋回コントローラ

    - current_yaw と now_sec を受け取り、小パルス or 停止の指示を返す。
    - 内部状態として「パルス出力中」「停止待機中」を持ち、
      指定時間経過後にフェーズを切り替える。
    - 停止フェーズ中に目標角±tolerance_deg に入っていれば完了とみなす。

    想定利用:
        ctrl = SmallAngleTurnController()
        ctrl.start(target_yaw_deg, current_yaw_deg, now_sec)
        while not ctrl.finished():
            decision = ctrl.step(now_sec, current_yaw_deg)
            # decision.direction に応じて turn_left/turn_right/stop を発行
    """

    def __init__(self, pulse_sec: float = 0.1, wait_sec: float = 0.1, tolerance_deg: float = 0.5) -> None:
        self.pulse_sec = pulse_sec # パルス出力時間[s]
        self.wait_sec = wait_sec # 停止待機時間[s]
        self.tolerance_deg = tolerance_deg # 許容誤差[度]

        self.active: bool = False # 旋回タスクがアクティブか
        self.target_yaw_deg: float = 0.0 # 目標絶対 yaw [deg]
        self._phase: str = "idle"  # 'idle' | 'pulse' | 'wait' | 'done'
        self._phase_start_time: float = 0.0 # フェーズ開始時刻 [s]

    def start(self, target_yaw_deg: float, current_yaw_deg: float, now_sec: float) -> None:
        """小角度パルス旋回の開始"""
        del current_yaw_deg  # 現状は保持不要だが将来拡張に備えて引数として受ける
        self.target_yaw_deg = target_yaw_deg % 360.0
        self.active = True
        self._phase = "pulse"
        self._phase_start_time = now_sec

    def is_active(self) -> bool:
        return self.active

    def finished(self) -> bool:
        return not self.active

    def _signed_error(self, current_yaw_deg: float) -> float:
        """target - current の最短差分（左回りで増加, 右回りで減少）。

        - current, target は 0〜360 に正規化した角度
        - 誤差は wrap-around を考慮して [-180, 180] に畳み込み
          （正→左回りで近づく／負→右回りで近づく）
        """
        cur = current_yaw_deg % 360.0
        tgt = self.target_yaw_deg % 360.0

        diff = tgt - cur  # -360〜360
        if diff > 180.0:
            diff -= 360.0     # 例: tgt=355, cur=5 → 350→ -10°
        elif diff < -180.0:
            diff += 360.0     # 例: tgt=5, cur=350 → -345→ +15°

        return diff

    def step(self, now_sec: float, current_yaw_deg: float) -> SmallTurnDecision:
        """現在時刻と yaw に基づき、小パルス or 停止指令を返す。"""
        if not self.active:
            return SmallTurnDecision(done=True, direction=None, slow=True, error_deg=0.0, phase="done")

        error = self._signed_error(current_yaw_deg) # target - current
        print(f"SmallAngleTurnController: error={error:.2f}°, phase={self._phase}")

        # PULSE 中以外で許容誤差内なら完了
        if abs(error) <= self.tolerance_deg:
            self.active = False
            self._phase = "done"
            return SmallTurnDecision(done=True, direction=None, slow=True, error_deg=error, phase="done")

        # フェーズ別処理
        if self._phase == "pulse":
            elapsed = now_sec - self._phase_start_time
            if elapsed >= self.pulse_sec:
                # 規定時間だけパルスを出し切ったので STOP + 待機フェーズへ
                self._phase = "wait"
                self._phase_start_time = now_sec
                return SmallTurnDecision(done=False, direction=None, slow=True, error_deg=error, phase="wait")

            # パルス継続中: 誤差の符号に応じて左右どちらかを指示
            direction = "left" if error > 0.0 else "right"
            return SmallTurnDecision(done=False, direction=direction, slow=True, error_deg=error, phase="pulse")

        if self._phase == "wait":
            elapsed = now_sec - self._phase_start_time
            if elapsed >= self.wait_sec:
                # 待機完了。まだ誤差が残っていれば次のパルスへ。
                if abs(error) <= self.tolerance_deg:
                    self.active = False
                    self._phase = "done"
                    return SmallTurnDecision(done=True, direction=None, slow=True, error_deg=error, phase="done")

                self._phase = "pulse"
                self._phase_start_time = now_sec
                direction = "left" if error > 0.0 else "right"
                return SmallTurnDecision(done=False, direction=direction, slow=True, error_deg=error, phase="pulse")

            # まだ待機中: STOP を継続
            return SmallTurnDecision(done=False, direction=None, slow=True, error_deg=error, phase="wait")

        # 想定外フェーズ: 安全側で STOP
        self._phase = "wait"
        self._phase_start_time = now_sec
        return SmallTurnDecision(done=False, direction=None, slow=True, error_deg=error, phase="wait")


@dataclass
class PidTurnDecision:
    """PIDベース旋回用の指令案

    Attributes:
        done:        目標角に収束したか
        direction:   'left' | 'right' | None
        speed_norm:  出力[1100~1900]
        error_deg:   target_yaw - current_yaw の符号付き誤差（[-180,180]）
    """
    done: bool
    direction: Optional[str]
    speed_norm: int
    error_deg: float


class PidTurnController:
    """絶対 yaw 目標に対する PID 旋回コントローラ
    - start(theta_deg, current_yaw_deg, now_sec) で「相対角度 theta_deg」だけ回すタスクを開始
        - 例: theta_deg=+180 → 左に 180°、theta_deg=-90 → 右に 90°
    - step(now_sec, current_yaw_deg) ごとに PidTurnDecision を返す。
        - direction: 'left' / 'right' / None
        - speed_norm: 1100〜1900 の PWM 出力値
    - [-180,180] 度の誤差を算出，オーバーシュート防止のため徐々に減速
    - オーバーシュートした場合，逆方向へ回して補正．徐々に収束させる
    - finished() で完了判定
    """
    def __init__(self, kp: float = 0.04, ki: float = 0.0, kd: float = 0.00, tolerance_deg: float = 3.0, integral_limit: float = 200.0, u_max: float = 1.0,) -> None:
        self.kp = kp # 比例ゲイン
        self.ki = ki # 積分ゲイン
        self.kd = kd # 微分ゲイン
        self.tolerance_deg = tolerance_deg # 許容誤差[度]
        self.integral_limit = integral_limit # 積分飽和防止用リミット[度・秒]
        self.u_max = max(1e-3, u_max) # 出力上限（0禁止）

        self.active: bool = False # 旋回タスクがアクティブか

        self._prev_yaw: Optional[float] = None # 前回 yaw [deg]
        self._rotated_deg: float = 0.0 # 累積旋回角度 [deg]
        self._target_theta: float = 0.0 # 目標相対角度 [deg]

        self._last_error: float = 0.0 # 前回誤差[度]
        self._integral: float = 0.0 # 積分値
        self._last_time: Optional[float] = None # 前回ステップ時刻[s]
    
    def start(self, theta_deg: float, current_yaw: float, now_sec: float) -> None:
        """相対角度 theta_deg の PID 旋回開始"""
        self._target_theta = theta_deg
        self._rotated_deg = 0.0
        self._prev_yaw = current_yaw

        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = now_sec
        self._yaw_accum = 0.0

        self.active = True
    
    def is_active(self) -> bool:
        return self.active

    def finished(self) -> bool:
        return not self.active

    def finish(self, error: float) -> PidTurnDecision:
        self.active = False
        return PidTurnDecision(done=True, direction=None, speed_norm=1500, error_deg=error)

    @staticmethod
    def _yaw_error(current_yaw_deg: float, prev_yaw_deg: float) -> float:
        """
        yaw の差分（current - prev）を、
        回転方向を保ったまま [-180, 180] に正規化する。

        正の値: 左旋回
        負の値: 右旋回
        """
        diff = current_yaw_deg - prev_yaw_deg

        if diff > 180.0:
            diff -= 360.0
        elif diff < -180.0:
            diff += 360.0

        return diff
    
    def step(self, now_sec: float, current_yaw: float) -> PidTurnDecision:
        """"""
        if not self.active:
            return self.finish(0.0)
        
        # 回転量を積算
        delta_yaw = self._yaw_error(current_yaw, self._prev_yaw) # 前回からの変化量
        self._yaw_accum += delta_yaw

        # 3. 一定以上たまったら反映
        if abs(self._yaw_accum) >= 0.3:
            self._rotated_deg += self._yaw_accum
            self._yaw_accum = 0.0
        # # IMUノイズ除去
        # if abs(delta_yaw) < 0.1:   # ← 実機で調整（0.3〜0.8）
        #     delta_yaw = 0.0
        # self._rotated_deg += delta_yaw # 累積旋回角度更新
        self._prev_yaw = current_yaw # 前回 yaw 更新

        # 残り回転量（これが制御誤差）
        error = self._target_theta - self._rotated_deg # target - current
        print(f"current_yaw={current_yaw:.2f}, delta_yaw={delta_yaw:.2f}, rotated_deg={self._rotated_deg:.2f}, error={self._target_theta}-{self._rotated_deg:.2f}={error:.2f}")

        # 終了判定
        if abs(error) <= self.tolerance_deg:
            return self.finish(error)
        
        # 時間差分 dt
        dt = max(1e-3, now_sec - self._last_time) if self._last_time else 0.0
        # 積分値
        self._integral += error * dt
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral)) # 積分飽和防止
        # 微分値
        derivative = 0.0 if dt == 0.0 else (error - self._last_error) / dt

        # PID 制御量 u
        u = self.kp * error + self.ki * self._integral + self.kd * derivative
        u = max(-self.u_max, min(self.u_max, u))
        # 状態更新
        self._last_error = error
        self._last_time = now_sec

        direction = "left" if u > 0.0 else "right"
        delta = abs(u) / self.u_max  # 0.0 ~ 1.0
        if direction == "left":
            speed_norm = int(1350 - delta * (1350 - 1200))
        else:
            speed_norm = int(1700 + delta * (1800 - 1700))
        return PidTurnDecision(done=False, direction=direction, speed_norm=speed_norm, error_deg=error)


