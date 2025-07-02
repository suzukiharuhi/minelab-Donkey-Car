from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from math import degrees, sqrt
import rospy

class IMUHandler:
    def __init__(self):
        self.roll = self.pitch = self.yaw = self.acc = 0
        self.initialized = False
        rospy.Subscriber('/imu/forwarded', Imu, self.imu_callback)

    def imu_callback(self, data):
        #rospy.loginfo("-- get IMU data")
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.roll  = degrees(roll)
        self.pitch = degrees(pitch)
        self.yaw   = degrees(yaw) + 180

        ax = data.linear_acceleration.x
        ay = data.linear_acceleration.y
        az = data.linear_acceleration.z
        self.acc = sqrt(ax**2 + ay**2 + az**2)

        self.initialized = True

    def get_yaw(self):
        return self.yaw

    def get_acc(self):
        return self.acc
    

