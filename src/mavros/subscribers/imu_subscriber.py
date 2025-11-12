from typing import Optional
import rospy
from sensor_msgs.msg import Imu


class IMUSubscriber:
    """IMU 購読クラス

    - start()/stop() で購読の開始/終了を制御
    - 初回受信で imu_initialized=True
    - 最新の Imu メッセージを保持
    """

    def __init__(self, topic: str = '/mavros/imu/data') -> None:
        self.topic = topic
        self._sub: Optional[rospy.Subscriber] = None
        self.is_active: bool = False
        self._latest: Optional[Imu] = None
        self.imu_initialized: bool = False

    def start(self) -> None:
        if not self.is_active:
            self._sub = rospy.Subscriber(self.topic, Imu, self._callback)
            self.is_active = True
            rospy.loginfo(f"IMU Subscriber started: {self.topic}")

    def stop(self) -> None:
        if self.is_active and self._sub is not None:
            self._sub.unregister()
            self._sub = None
            self.is_active = False
            self.imu_initialized = False
            rospy.loginfo("IMU Subscriber stopped")

    def _callback(self, msg: Imu) -> None:
        self._latest = msg
        if not self.imu_initialized:
            self.imu_initialized = True
            rospy.loginfo("IMU first message received. Initialization complete.")

    @property
    def latest(self) -> Optional[Imu]:
        return self._latest
