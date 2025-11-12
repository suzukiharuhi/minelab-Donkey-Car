from typing import Tuple
from pilot.app.core.commands import ControlCommand, SpeedLevel


def _normalize_angle_deg(angle: float) -> float:
    """Wrap angle to [-180, 180)."""
    a = (angle + 180.0) % 360.0 - 180.0
    # handle -180 edge (optional)
    return a if a != -180.0 else 180.0


def turn_towards(current_yaw_deg: float, target_yaw_deg: float, speed: SpeedLevel = SpeedLevel.SLOW, deadband_deg: float = 3.0) -> Tuple[ControlCommand, bool]:
    """
    Compute a turn command to rotate from current yaw to target yaw.

    Args:
        current_yaw_deg: 現在のヨー角[deg]（0-360 または -180〜180 いずれでも可）
        target_yaw_deg: 目標ヨー角[deg]
        speed: 旋回速度のレベル
        deadband_deg: 許容誤差[deg]以内なら完了とみなす

    Returns:
        (ControlCommand, done):
            - done=True のとき、停止コマンドを返す
            - done=False のとき、回転コマンド（左/右）を返す

    Note:
        yaw の正方向と左右回転の対応は実機依存です。もし挙動が逆なら、左右の分岐を反転してください。
    """
    err = _normalize_angle_deg(target_yaw_deg - current_yaw_deg)
    if abs(err) <= deadband_deg:
        return ControlCommand.stop(), True

    # 正の誤差 -> 右旋回 / 負の誤差 -> 左旋回（必要に応じて反転）
    if err > 0:
        return ControlCommand.turn_right(speed), False
    else:
        return ControlCommand.turn_left(speed), False
