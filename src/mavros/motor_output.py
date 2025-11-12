from dataclasses import dataclass
from typing import List, Optional
import rospy
from mavros_msgs.msg import OverrideRCIn

# サポートするコマンドと速度
VALID_COMMANDS = {
    "stop", "move_forward", "forward_left", "forward_right",
    "move_backward", "backward_left", "backward_right",
    "turn_left", "turn_right"
}
VALID_SPEEDS = {"slow", "mid", "high"}

# PWM値テーブル
THROTTLE_FWD = {"slow": 1700, "mid": 1800, "high": 1900}
THROTTLE_REV = {"slow": 1300, "mid": 1200, "high": 1100}
YAW_LEFT = {"slow": 1250, "mid": 1180, "high": 1100}
YAW_RIGHT = {"slow": 1750, "mid": 1820, "high": 1900}

@dataclass
class MotorState:
    channels: List[int]

    @classmethod
    def neutral(cls) -> "MotorState":
        # return cls([1500, 0, 1500, 0, 0, 0, 0, 0])
        return cls([1500] * 18)

class InvalidMotorCommand(Exception):
    pass

class InvalidSpeedLevel(Exception):
    pass

class MotorOutputController:
    """RCチャンネル PWM を生成・publishするクラス"""
    def __init__(self, topic: str = "/mavros/rc/override", queue_size: int = 10):
        self._publisher = rospy.Publisher(topic, OverrideRCIn, queue_size=queue_size)
        self._state = MotorState.neutral()  # 各チャンネルをニュートラルに設定
        self._last_command: Optional[str] = None  # 最後に適用したコマンド
        self._last_speed: Optional[str] = None  # 最後に適用した速度

    @staticmethod
    def _normalize_command(command: str) -> str:
        """コマンド文字列を正規化し、未知コマンドは例外送出"""
        if not isinstance(command, str):
            raise InvalidMotorCommand(f"Command must be a string, got {type(command)}")
        if command not in VALID_COMMANDS:
            raise InvalidMotorCommand(f"Unknown motor command: {command}")
        return command

    @staticmethod
    def _normalize_speed(speed: str) -> str:
        """速度文字列を正規化し、未知速度レベルは例外送出"""
        if not isinstance(speed, str):
            raise InvalidSpeedLevel(f"Speed must be a string, got {type(speed)}")
        if speed not in VALID_SPEEDS:
            raise InvalidSpeedLevel(f"Unknown speed level: {speed}")
        return speed

    def apply_command(self, command: str, speed: Optional[str] = None) -> None:
        """コマンドと速度を内部 state に反映"""
        # コマンドと速度を正規化
        cmd = self._normalize_command(command)
        if speed is None:
            speed = "mid"
        spd = self._normalize_speed(speed)

        # Ch1(index0), Ch3(index2)
        ch1, ch3 = 1500, 1500
        if cmd == "stop":
            pass
        elif cmd == "move_forward":
            ch3 = THROTTLE_FWD[spd]; ch1 = 1460
        elif cmd == "forward_left":
            ch3 = THROTTLE_FWD[spd]; ch1 = 1400
        elif cmd == "forward_right":
            ch3 = THROTTLE_FWD[spd]; ch1 = 1600
        elif cmd == "move_backward":
            ch3 = THROTTLE_REV[spd]
        elif cmd == "backward_left":
            ch3 = THROTTLE_REV[spd]; ch1 = 1600
        elif cmd == "backward_right":
            ch3 = THROTTLE_REV[spd]; ch1 = 1400
        elif cmd == "turn_left":
            ch1 = YAW_LEFT[spd]
        elif cmd == "turn_right":
            ch1 = YAW_RIGHT[spd]

        self._state.channels[0] = ch1
        self._state.channels[2] = ch3
        # 未使用は 0
        for i in range(8):
            if i not in (0, 2):
                self._state.channels[i] = 0
        self._last_command = cmd
        self._last_speed = spd

        print(f"Applied command: {cmd}, speed: {spd}")

    def build_msg(self) -> OverrideRCIn:
        msg = OverrideRCIn()
        msg.channels = list(self._state.channels)
        return msg

    def publish(self) -> None:
        msg = self.build_msg()
        self._publisher.publish(msg)
        print(f"Published: {msg.channels}")  # 最初の3チャンネルだけ表示

    def run_motor(self, command: str, speed: str) -> None:
        """publish motor command"""
        try:
            self.apply_command(command, speed)
        except (InvalidMotorCommand, InvalidSpeedLevel) as e:
            rospy.logwarn_throttle(5.0, f"Motor command ignored: {e}")
            return
        self.publish()

    def stop(self) -> None:
        self._state.channels[0] = 1500
        self._state.channels[2] = 1500
        self._last_command = "stop"
        self._last_speed = None
        self.publish()
