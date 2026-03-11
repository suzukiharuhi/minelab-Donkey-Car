import rospy
from typing import Optional, Any
from ros_ugv.msg import OperateCommand
from pilot.app.core.commands import ControlCommand as CtrlObj, CommandType, SpeedLevel


class MotorCommandPublisher:
    """OperateCommand.msg をシンプルに publish するクラス（command, speed の2引数）

        - publish(command, speed) で使用
            - command: ControlCommand | CommandType | str
            - speed:   SpeedLevel | str | None（None の場合は ControlCommand から補完 or 'mid'）
        - 送信前に小文字文字列に正規化
        - 前回と同じ (command,speed) ならログを抑制
    """

    def __init__(self, topic: str = '/control/command'):
        """Publisher 初期化

        Args:
            topic: 出力トピック名
        """
        self.pub = rospy.Publisher(topic, OperateCommand, queue_size=10)
        self._last_key = None  # 直前に送った (command:speed) を保持し重複ログ抑制

    @staticmethod
    def _normalize_speed(speed: Optional[Any]) -> str:
        """Speed を文字列に正規化

        - SpeedLevel -> "slow"/"mid"/"high"/...
        - int/float   -> PWM 値("1100"〜"1900")としてそのまま文字列化（範囲外はクリップ）
        - str         -> 小文字化してそのまま
        """
        if speed is None:
            return 'mid'

        # 数値(PWM)指定: 1100〜1900 にクリップして文字列化
        if isinstance(speed, (int, float)):
            val = int(speed)
            if val < 1100:
                val = 1100
            if val > 1900:
                val = 1900
            return str(val)

        if isinstance(speed, str):
            return speed.lower()
        if isinstance(speed, SpeedLevel):
            return speed.name.lower()
        # 想定外型は文字列化して使う
        return str(speed).lower()

    @staticmethod
    def _normalize_command(command: Any) -> str:
        """コマンドを小文字文字列に正規化"""
        if isinstance(command, CtrlObj):
            return command.command.name.lower()
        if isinstance(command, CommandType):
            return command.name.lower()
        if isinstance(command, str):
            return command.lower()
        return 'stop'

    def publish(self, command: Any, speed: Optional[Any] = None) -> None:
        """OperateCommand.msg を送信

        Args:
            command: ControlCommand | CommandType | str
            speed: SpeedLevel | str | None
        """
        if command is None:
            return

        # ControlCommand の場合は speed 未指定ならオブジェクトの値で補完
        if isinstance(command, CtrlObj) and speed is None:
            speed_to_use = command.speed
        else:
            speed_to_use = speed

        command_str = self._normalize_command(command)
        speed_str = self._normalize_speed(speed_to_use)
        key = f"{command_str}:{speed_str}"
        if key != self._last_key:
            self._last_key = key
            print(f"publish: {key}")

        msg = OperateCommand()
        msg.command = command_str
        msg.speed = speed_str
        self.pub.publish(msg)
