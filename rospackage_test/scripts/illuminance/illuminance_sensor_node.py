import rospy
import smbus
import time
from std_msgs.msg import Bool

class IlluminanceSensorNode:
    """
    照度センサの値を送信するノード 

    Publisher
    ・'pilot_node'に照度センサの値を送信 (/illuminance)

    """
    def __init__(self):
        rospy.init_node('illuminance_sensor_node')
        rospy.loginfo("illuminance_sensor_nodeを初期化")

        self.rate = rospy.Rate(1) # 50Hz

        self.sensor_address = 0x23
        self.check_num      = 10
        self.read_mode      = 0x10
        self.scale_factor   = 1.2
        self.sleep_interval = 1
        self.byte_count     = 2
        self.bus = smbus.SMBus(1)

        # Publisher
        self.illuminance_pub = rospy.Publisher('/illuminance', Bool, queue_size=10)

    # Read data from illuminance sensor
    def _read_lux(self) -> float:
        data = self.bus.read_i2c_block_data(self.sensor_address, self.read_mode, self.byte_count)
        lux = (data[0] << 8) + data[1]
        print(f'Illuminance: {lux} lx')
        return lux / self.scale_factor
    
        # Calculate average illuminance over CHECK_NUM readings
    def _get_average_illuminance(self) -> float:
        total_lux = 0
        for _ in range(self.check_num):
            total_lux += self._read_lux()
            time.sleep(self.sleep_interval)
        average_lux = total_lux / self.check_num
        return average_lux

    # Check if the average illuminance is above a given threshold
    def is_above_threshold(self, threshold: float) -> bool:
        print('Checking illuminance sensor...')
        average_lux = self._get_average_illuminance()
        self.illuminance_pub.publish(self.imu_data)

