"""RC-override mapper (ROS-free).

Maps ``ControlCommand`` → ``(ch1_pwm, ch3_pwm)`` pairs that are fed into
``OverrideRCIn`` by the node layer.  Mirrors the PWM table in
``sss/mavros/motor_output.py`` so both implementations stay consistent.
"""
from __future__ import annotations

from typing import Tuple

from minelab.app.core.commands import CommandType, ControlCommand, SpeedLevel

# PWM tables (same as sss/mavros/motor_output.py)
_THROTTLE_FWD: dict[str, int] = {
    "very_slow": 1600, "slow": 1670, "mid": 1800, "high": 1900
}
_THROTTLE_REV: dict[str, int] = {
    "very_slow": 1450, "slow": 1300, "mid": 1200, "high": 1100
}
_YAW_LEFT: dict[str, int] = {
    "very_slow": 1320, "slow": 1250, "mid": 1180, "high": 1100
}
_YAW_RIGHT: dict[str, int] = {
    "very_slow": 1720, "slow": 1750, "mid": 1820, "high": 1900
}

_NEUTRAL = 1500


class RCOverrideMapper:
    """Map a ``ControlCommand`` to RC channel PWM values.

    Only channels 1 (steering/yaw) and 3 (throttle) are used.

    Args:
        ch1_index:  0-based index of the steering channel (default 0).
        ch3_index:  0-based index of the throttle channel (default 2).
        num_channels:  Total RC channel count (default 18).
    """

    def __init__(
        self,
        ch1_index: int = 0,
        ch3_index: int = 2,
        num_channels: int = 18,
    ) -> None:
        self._ch1_idx = ch1_index
        self._ch3_idx = ch3_index
        self._num_channels = num_channels

    def map(self, cmd: ControlCommand) -> Tuple[int, int]:
        """Return ``(ch1_pwm, ch3_pwm)`` for the given command.

        The caller is responsible for building the full ``OverrideRCIn``
        message.
        """
        spd = cmd.speed.name.lower()
        ct = cmd.command

        ch1 = _NEUTRAL
        ch3 = _NEUTRAL

        if ct == CommandType.STOP or ct == CommandType.NO_OPERATION:
            pass
        elif ct == CommandType.MOVE_FORWARD:
            ch3 = _THROTTLE_FWD[spd]
            ch1 = 1535
        elif ct == CommandType.FORWARD_LEFT:
            ch3 = 1700
            ch1 = _YAW_LEFT[spd]
        elif ct == CommandType.FORWARD_RIGHT:
            ch3 = 1700
            ch1 = _YAW_RIGHT[spd]
        elif ct == CommandType.MOVE_BACKWARD:
            ch3 = _THROTTLE_REV[spd]
        elif ct == CommandType.BACKWARD_LEFT:
            ch3 = _THROTTLE_REV[spd]
            ch1 = 1600
        elif ct == CommandType.BACKWARD_RIGHT:
            ch3 = _THROTTLE_REV[spd]
            ch1 = 1400
        elif ct == CommandType.TURN_LEFT:
            ch1 = _YAW_LEFT[spd]
        elif ct == CommandType.TURN_RIGHT:
            ch1 = _YAW_RIGHT[spd]

        return ch1, ch3

    def build_channels(self, cmd: ControlCommand) -> list[int]:
        """Return a full channel list (length *num_channels*) for ``OverrideRCIn``."""
        ch1, ch3 = self.map(cmd)
        channels = [0] * self._num_channels
        channels[self._ch1_idx] = ch1
        channels[self._ch3_idx] = ch3
        return channels
