from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import euler_from_quaternion
from math import degrees

class IMU:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.acc = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.imu_initialized = False

    def imu_callback(self, data):
        """
        Calculate roll, pitch, yaw angle from geomagnetic data
        """
        # get guaternion(x,y,z,w)
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # convert from quaternion to Euler angles(roll, pitch, yaw)
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # Convert radians to degrees
        self.roll  = degrees(roll)
        self.pitch = degrees(pitch)
        self.yaw   = degrees(yaw) + 180

        # linear acceleration (m/s^2)
        self.ax = data.linear_acceleration.x
        self.ay = data.linear_acceleration.y
        self.az = data.linear_acceleration.z
        # 加速度の大きさを計算
        self.acc = (self.ax**2 + self.ay**2)**0.5

        self.imu_initialized = True


    def get_yaw(self):
        """
        Returns current orientation as (roll, pitch, yaw) in degrees.
        """
        return self.yaw
    
    def get_acc(self):
        return self.acc
        #return self.ax, self.ay, self.az