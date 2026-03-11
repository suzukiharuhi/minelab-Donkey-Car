"""IMUセンサー管理"""
import rospy
from sensor_msgs.msg import BatteryState
from typing import Optional

from . import BaseSensor
from pilot.app.utils.data_models import BatteryData


class BatterySensor(BaseSensor):
    """バッテリーセンサークラス"""
    
    def __init__(self, topic: str = '/mavros/battery'):
        super().__init__()
        self.topic = topic
        self._subscriber: Optional[rospy.Subscriber] = None
        self._latest_data: Optional[BatteryData] = None
        self.battery_initialized = False
    
    def start(self) -> None:
        """バッテリーセンサーの開始"""
        if not self._is_active:
            self._subscriber = rospy.Subscriber(self.topic, BatteryState, self._callback)
            self._is_active = True
            rospy.loginfo(f"Battery Sensor started: {self.topic}")

    def stop(self) -> None:
        """バッテリーセンサーの停止"""
        if self._is_active and self._subscriber is not None:
            self._subscriber.unregister()
            self._subscriber = None
            self._is_active = False
            rospy.loginfo("Battery Sensor stopped")
    
    def _callback(self, msg: BatteryState) -> None:
        print(f"Battery callback: voltage={msg.voltage:.2f}V, percentage={msg.percentage*100:.1f}%")
        self.battery_initialized = True
        self._latest_data = BatteryData(
            voltage=msg.voltage,
            percentage=msg.percentage,
            timestamp=rospy.get_time()
        )
            
    def get_latest_data(self) -> Optional[BatteryData]:
        """最新データの取得
        
        Returns:
            最新のBatteryData、データがない場合はNone
        """
        return self._latest_data
