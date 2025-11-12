from typing import Optional
import rospy
from ros_ugv.msg import OperateCommand


class ControlCommandSubscriber:
    """/control/command を購読するサブスクライバ
    購読データ：
        - command: 最新のコマンド文字列
        - speed: 最新の速度の文字列（slow/mid/high）
    """

    def __init__(self, topic: str = '/control/command') -> None:
        self.topic = topic
        self._sub: Optional[rospy.Subscriber] = None
        self._is_active: bool = False
        self._command: Optional[str] = None
        self._speed: Optional[str] = None
        self._prev_command: Optional[str] = None
        self._prev_speed: Optional[str] = None

    def start(self) -> None:
        """購読開始"""
        if not self._is_active:
            # publisherは ros_ugv/OperateCommand をpublish
            self._sub = rospy.Subscriber(self.topic, OperateCommand, self._callback)
            self._is_active = True
            rospy.loginfo(f"ControlCommand subscriber started: {self.topic}")

    def stop(self) -> None:
        """購読停止"""
        if self._is_active and self._sub is not None:
            self._sub.unregister()
            self._sub = None
            self._is_active = False
            rospy.loginfo("ControlCommand subscriber stopped")

    def _callback(self, msg: OperateCommand) -> None:
        """購読コールバック"""
        # 直前値を保存してから更新
        self._prev_command = self._command
        self._prev_speed = self._speed

        # publisher側のフィールドに合わせて格納
        self._command = msg.command
        self._speed = msg.speed
        # print(f"Received command: {self._command}, speed: {self._speed}")

    @property
    def command(self) -> Optional[str]:
        """最新の command を取得"""
        return self._command

    @property
    def speed(self) -> Optional[str]:
        """最新の speed を取得"""
        return self._speed

    def changed(self) -> bool:
        """command または speed のどちらかが変われば True"""
        return (self._command != self._prev_command) or (self._speed != self._prev_speed)

    @property
    def is_active(self) -> bool:
        """サブスクライバが開始されているかどうか"""
        return self._is_active
