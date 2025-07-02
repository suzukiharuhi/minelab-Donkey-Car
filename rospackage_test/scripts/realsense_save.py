#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

class RealSenseSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None

        rospy.init_node('realsense_saver')
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)

        rospy.loginfo("Waiting for both color and depth images...")
        rospy.Timer(rospy.Duration(1.0), self.save_images, oneshot=True)

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        # 深度は16bitのまま取得
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def save_images(self, event):
        if self.color_image is not None and self.depth_image is not None:
            save_dir = os.path.expanduser("/home/pi/catkin_ws/src/rospackage_test/scripts/images")
            os.makedirs(save_dir, exist_ok=True)

            # カラー画像はそのまま保存
            cv2.imwrite(os.path.join(save_dir, "color.png"), self.color_image)

            # 深度画像の16bitデータを8bitに変換（アルファ値は環境に応じて調整してください）
            depth_8bit = cv2.convertScaleAbs(self.depth_image, alpha=0.1)

            # 8bit深度画像を保存（グレースケール）
            cv2.imwrite(os.path.join(save_dir, "depth_8bit.png"), depth_8bit)

            # カラーマップ（ヒートマップ）を付けて保存（視認性向上）
            depth_color = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
            cv2.imwrite(os.path.join(save_dir, "depth_color.png"), depth_color)

            rospy.loginfo("Saved images (color, depth_8bit, depth_color) to {}".format(save_dir))
            rospy.signal_shutdown("Done")
        else:
            rospy.logwarn("Images not received yet. Waiting again...")
            rospy.Timer(rospy.Duration(1.0), self.save_images, oneshot=True)

if __name__ == '__main__':
    RealSenseSaver()
    rospy.spin()
