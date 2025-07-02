#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RealSenseFrameGetter:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = self.depth_image = None
        self.color_initialized = self.depth_initialized = False

        rospy.init_node('frame_getter_node', anonymous=True)
        self.rate = rospy.Rate(10) # 10Hz

        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.color_initialized = True
        except Exception as e:
            rospy.logerr(f"Failed to convert color image: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_initialized = True
        except Exception as e:
            rospy.logerr(f"Failed to convert depth image: {e}")

    def get_frames(self):
        """
        最新のカラー画像と深度画像を返します。
        Returns:
            color_image (np.ndarray or None): BGR画像
            depth_image (np.ndarray or None): 16bit深度画像
        """
        return self.color_image, self.depth_image

    def run(self):
        while not(self.color_initialized and self.depth_initialized):
            #rospy.loginfo("Waiting for IMU data...")
            self.rate.sleep()
        while not rospy.is_shutdown():
            color, depth = self.get_frames()
            if color is not None and depth is not None:
                rospy.loginfo("Frames received.")
            else:
                print("no frames")
            self.rate.sleep()

if __name__ == '__main__':
    RealSenseFrameGetter().run()
