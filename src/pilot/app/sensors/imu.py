"""IMUセンサー管理"""
import rospy
from sensor_msgs.msg import Imu
from typing import Optional
from tf.transformations import euler_from_quaternion
from math import degrees

from . import BaseSensor
from pilot.app.utils.data_models import IMUData


class IMUSensor(BaseSensor):
    """IMUセンサークラス"""
    
    def __init__(self, topic: str = '/mavros/imu/data'):
        """
        Args:
            topic_name: IMUデータのトピック名
        """
        super().__init__()
        self.topic = topic
        self._subscriber: Optional[rospy.Subscriber] = None
        self._latest_data: Optional[IMUData] = None
        self.roll: Optional[float] = None
        self.pitch: Optional[float] = None
        self.yaw: Optional[float] = None
        # 初期化フラグ（最初のIMUメッセージ受信で True）
        self.imu_initialized: bool = False
        
    
    def start(self) -> None:
        """IMUセンサーの開始"""
        if not self._is_active:
            self._subscriber = rospy.Subscriber(self.topic, Imu, self._imu_callback)
            self._is_active = True
            rospy.loginfo(f"IMU Sensor started: {self.topic}")

    def stop(self) -> None:
        """IMUセンサーの停止"""
        if self._is_active and self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None
            self._is_active = False
            self.imu_initialized = False
            rospy.loginfo("IMU Sensor stopped")
    
    def _imu_callback(self, msg: Imu) -> None:
        """
        IMUデータのコールバック
        
        Args:
            msg: IMUメッセージ
        """
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        try:
            (roll_rad, pitch_rad, yaw_rad) = euler_from_quaternion(orientation_list)
            # Convert radians to degrees
            self.roll  = degrees(roll_rad)
            self.pitch = degrees(pitch_rad)
            self.yaw   = degrees(yaw_rad) + 180
        except Exception as e:
            rospy.logwarn(f"Failed to convert quaternion to Euler: {e}")
            self.roll = self.pitch = self.yaw = None

        self._latest_data = IMUData(
            orientation=(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ),
            euler_angles=(
                self.roll,
                self.pitch,
                self.yaw
            ),
            angular_velocity=(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ),
            linear_acceleration=(
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ),
            timestamp=rospy.get_time()
        )

        # 初回受信で初期化完了とみなす
        if not self.imu_initialized:
            self.imu_initialized = True
            rospy.loginfo("IMU first message received. Initialization complete.")

        # 受信時コールバックが設定されていれば呼び出す
        if self._callback is not None:
            try:
                self._callback(self._latest_data)
            except Exception as e:
                rospy.logwarn(f"IMU callback error: {e}")
    
    def get_latest_data(self) -> Optional[IMUData]:
        """最新データの取得
        
        Returns:
            最新のIMUData、データがない場合はNone
        """
        return self._latest_data
