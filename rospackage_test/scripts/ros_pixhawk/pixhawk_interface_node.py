#!/usr/bin/env python3

import rospy
import datetime
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from motor_output import MotorOutput

class PixhawkInterfaceNode:
    """
    Pixhawk(MAVROS)からIMUデータを取得するノード 

    Publisher
    ・'pilot_node'にIMUデータを流す
    ・PixhawkにRC入力値を送信 (MotorOutput)
    Subscriber
    ・PixhawkからIMUデータを受信 /mavros/imu/data (imu_callback)
    ・'pilot_node'からモーター制御指令を受信 /control/command (command_callback)
    """
    def __init__(self):
        self.imu_data = None
        self.imu_initialized = False
        self.command = None

        rospy.init_node('pixhawk_interface_node')
        rospy.loginfo("pixhawk_interface_nodeを初期化")

        self.imu_pub = rospy.Publisher('/imu/forwarded', Imu, queue_size=10)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/control/command', String, self.command_callback)
        self.rate = rospy.Rate(50)
        
    def imu_callback(self, data):
        """
        Calculate roll, pitch, yaw angle from geomagnetic data
        """
        self.imu_data = data
        self.imu_initialized = True
    
    def command_callback(self, data):
        """"""
        self.command = data.data

    def run(self):
        try:
            motor_controller = MotorOutput() # RC値入力用インスタンス

            # IMUデータを受信するまで待機
            while not self.imu_initialized and not rospy.is_shutdown():
                rospy.loginfo_throttle(5, "Waiting for IMU data...")
                self.rate.sleep()

            while not rospy.is_shutdown():
                # 'pilot_node'にIMUデータをpublish
                if self.imu_data is not None:
                    self.imu_pub.publish(self.imu_data)
                # RC制御
                motor_controller.run_motor(self.command)

                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.logwarn("ノードがCtrl+Cで停止されました")
        finally:
            rospy.loginfo("終了 -----------------------------------")


if __name__ == '__main__':
    PixhawkInterfaceNode().run()
