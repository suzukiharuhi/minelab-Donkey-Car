"""Control command types and data structures (ROS-free).

Mirrors and extends the existing ``sss/pilot/app/core/commands.py`` but
lives in the minelab package and has **no** ROS dependency.
"""
from __future__ import annotations

from enum import Enum, auto
from typing import Tuple


class CommandType(Enum):
    """Enumeration of all possible motion commands."""

    NO_OPERATION = auto()
    STOP = auto()

    MOVE_FORWARD = auto()
    FORWARD_LEFT = auto()
    FORWARD_RIGHT = auto()

    MOVE_BACKWARD = auto()
    BACKWARD_LEFT = auto()
    BACKWARD_RIGHT = auto()

    TURN_LEFT = auto()
    TURN_RIGHT = auto()


class SpeedLevel(Enum):
    """Discrete speed tiers mapped to PWM values in ``rc_override_mapper``."""

    VERY_SLOW = auto()
    SLOW = auto()
    MID = auto()
    HIGH = auto()


class ControlCommand:
    """Immutable control command with command type and speed level."""

    def __init__(
        self,
        command: CommandType,
        speed: SpeedLevel = SpeedLevel.MID,
    ) -> None:
        self.command = command
        self.speed = speed

    # ------------------------------------------------------------------
    # Factory helpers
    # ------------------------------------------------------------------

    @classmethod
    def no_operation(cls) -> "ControlCommand":
        return cls(CommandType.NO_OPERATION)

    @classmethod
    def stop(cls) -> "ControlCommand":
        return cls(CommandType.STOP)

    @classmethod
    def move_forward(cls, speed: SpeedLevel = SpeedLevel.MID) -> "ControlCommand":
        return cls(CommandType.MOVE_FORWARD, speed)

    @classmethod
    def forward_left(cls, speed: SpeedLevel = SpeedLevel.MID) -> "ControlCommand":
        return cls(CommandType.FORWARD_LEFT, speed)

    @classmethod
    def forward_right(cls, speed: SpeedLevel = SpeedLevel.MID) -> "ControlCommand":
        return cls(CommandType.FORWARD_RIGHT, speed)

    @classmethod
    def move_backward(cls, speed: SpeedLevel = SpeedLevel.MID) -> "ControlCommand":
        return cls(CommandType.MOVE_BACKWARD, speed)

    @classmethod
    def backward_left(cls, speed: SpeedLevel = SpeedLevel.MID) -> "ControlCommand":
        return cls(CommandType.BACKWARD_LEFT, speed)

    @classmethod
    def backward_right(cls, speed: SpeedLevel = SpeedLevel.MID) -> "ControlCommand":
        return cls(CommandType.BACKWARD_RIGHT, speed)

    @classmethod
    def turn_left(cls, speed: SpeedLevel = SpeedLevel.MID) -> "ControlCommand":
        return cls(CommandType.TURN_LEFT, speed)

    @classmethod
    def turn_right(cls, speed: SpeedLevel = SpeedLevel.MID) -> "ControlCommand":
        return cls(CommandType.TURN_RIGHT, speed)

    # ------------------------------------------------------------------
    # Serialisation helpers (used by nodes to publish over ROS topics)
    # ------------------------------------------------------------------

    def to_strings(self) -> Tuple[str, str]:
        """Return ``(command_str, speed_str)`` compatible with motor_output.py."""
        return self.command.name.lower(), self.speed.name.lower()

    def __repr__(self) -> str:
        return f"ControlCommand({self.command.name}, {self.speed.name})"
