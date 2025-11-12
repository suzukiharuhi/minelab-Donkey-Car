"""Control command types and data structures.

Separated from states/base_state.py to keep state definitions focused.
"""
from enum import Enum, auto


class CommandType(Enum):
    """制御コマンドタイプ"""
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
    """速度レベル"""
    SLOW = auto()
    MID = auto()
    HIGH = auto()


class ControlCommand:
    """制御コマンド"""

    def __init__(self, command: CommandType, speed: SpeedLevel = SpeedLevel.MID):
        """
        Args:
            command: コマンドタイプ
            speed: 速度レベル
        """
        self.command = command
        self.speed = speed

    @classmethod
    def no_operation(cls):
        """NOP"""
        return cls(CommandType.NO_OPERATION)

    @classmethod
    def stop(cls):
        """停止"""
        return cls(CommandType.STOP)

    @classmethod
    def move_forward(cls, speed: SpeedLevel = SpeedLevel.MID):
        """前進"""
        return cls(CommandType.MOVE_FORWARD, speed)
    
    @classmethod
    def forward_left(cls, speed: SpeedLevel = SpeedLevel.MID):
        """前進左折"""
        return cls(CommandType.FORWARD_LEFT, speed)

    @classmethod
    def forward_right(cls, speed: SpeedLevel = SpeedLevel.MID):
        """前進右折"""
        return cls(CommandType.FORWARD_RIGHT, speed)

    @classmethod
    def move_backward(cls, speed: SpeedLevel = SpeedLevel.MID):
        """後退"""
        return cls(CommandType.MOVE_BACKWARD, speed)

    @classmethod
    def backward_left(cls, speed: SpeedLevel = SpeedLevel.MID):
        """後退左折"""
        return cls(CommandType.BACKWARD_LEFT, speed)

    @classmethod
    def backward_right(cls, speed: SpeedLevel = SpeedLevel.MID):
        """後退右折"""
        return cls(CommandType.BACKWARD_RIGHT, speed)

    @classmethod
    def turn_left(cls, speed: SpeedLevel = SpeedLevel.MID):
        """左旋回"""
        return cls(CommandType.TURN_LEFT, speed)

    @classmethod
    def turn_right(cls, speed: SpeedLevel = SpeedLevel.MID):
        """右旋回"""
        return cls(CommandType.TURN_RIGHT, speed)


    def __repr__(self):
        return f"ControlCommand({self.command.name}, {self.speed.name})"
