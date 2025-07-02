#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import degrees, sqrt
import time
import cv2
import threading

from constant import *
from sub_imu import IMUHandler
from sub_realsense import RealSenseHandler
from aruco_detector import MarkerDetector
from state import State

class Pilot:
    """
    'pixhawk_interface_node'からIMUデータを受信, モーター制御指令を送信するノード 
    RealSenseのフレーム取得は別のスレッドをたて行う(get_reanselse())

    Publisher
    ・'pixhawk_interface_node'にモーター制御指令コマンドを送信 (/control/comand)
    Subscriber
    ・'pixhawk_interface_node'からIMUデータを受信 (IMUHandler)
    ・RealSenseからcolor, depthフレームを受信 (RealSenseHandler)
    """
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = self.depth_image = None

        rospy.init_node('pilot_node')
        rospy.loginfo("pilot_nodeを初期化")

        self.rate = rospy.Rate(50) # 50Hz
        self.msg = String()

        # 指令を送るPublisher
        self.command_pub = rospy.Publisher('/control/command', String, queue_size=10)

        self.imu_handler = IMUHandler()
        self.realsense_hanlder = RealSenseHandler()

        self.color = self.depth = self.marker_corners = self.list_marker_ids = None
        self.marker_detector = MarkerDetector()
        self.realsense_thread = threading.Thread(target=self.get_realsense, daemon=False)

    def publish_message(self, command):
        self.msg.data = command
        self.command_pub.publish(self.msg)

    def get_realsense(self):
        try:
            rospy.loginfo("[RealSenseスレッド] 開始")
            # realsenseのフレームを受信するまで待機
            while not(self.realsense_hanlder.color_initialized and self.realsense_hanlder.depth_initialized) and not rospy.is_shutdown():
                rospy.loginfo_throttle(5, "Waiting for RealSense data...")
                self.rate.sleep()

            while not rospy.is_shutdown():
                self.color, self.depth = self.realsense_hanlder.get_frames()
                self.marker_corners, self.list_marker_ids = self.marker_detector.get_marker(self.color)
        finally:
            rospy.loginfo("[RealSenseスレッド] 終了")

    def angle_diff(self, current, reference):
        if abs(current - reference) < 0.5:
            return 0
        diff = (current - reference + 360) % 360
        return diff

    def run(self):
        try:
            self.realsense_thread.start()

            # IMUデータを受信するまで待機
            while not self.imu_handler.initialized and not rospy.is_shutdown():
                rospy.loginfo_throttle(5, "Waiting for IMU data...")
                self.rate.sleep()
            # realsenseのフレームを受信するまで待機
            while not(self.realsense_hanlder.color_initialized and self.realsense_hanlder.depth_initialized) and not rospy.is_shutdown():
                rospy.loginfo_throttle(5, "Waiting for RealSense data...")
                self.rate.sleep()

            time.sleep(3)

            state = State.IDEL
            next_state = None 
            pause_sent = False
            target_yaw_degrees = None
            theta_p = 0

            while not rospy.is_shutdown():         
                current_yaw = self.imu_handler.get_yaw()

                if state == State.STOP:
                    current_time = time.time()
                    if not pause_sent:
                        self.publish_message("stop")
                        pause_sent = True
                        stop_end_time = current_time + PAUSE_TIME
                    if current_time >= stop_end_time:
                        pause_sent = False      # フラグをリセット
                        state = next_state      # 次の動作状態へ遷移
                        rospy.loginfo(f"state -> {state}")
                    continue  # 停止期間内は他の処理を行わない
                elif state == State.IDEL:
                    if 10 in self.list_marker_ids:
                        state = State.APPROACH_MARKER_10
                        rospy.loginfo("state -> APPROACH_MARKER_10")
                    elif 0 in self.list_marker_ids:
                        state = State.APPROACH_MARKER_0
                        rospy.loginfo("state -> APPROACH_MARKER_0")
                    else:
                        self.publish_message("move_forward")
                elif state == State.APPROACH_MARKER_10:
                    x, z = self.marker_detector.aruco_distance(10, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                    #rospy.logdebug(f"Marker 10: x={x:.2f}, z={z:.2f}")
                    if x is not None and z is not None:
                        if z <= DISTANCE_THRESHOLD_MARKER_10:
                            state = State.STOP
                            next_state = State.TURN_180
                            rospy.loginfo("state -> TURN_180")
                            continue
                        else:
                            if x < -1 * 10:
                                self.publish_message("slow_turn_left")
                            elif x > 10:
                                self.publish_message("slow_turn_right")
                            else: # -〇 < x < 〇
                                self.publish_message("move_forward")                        
                elif state == State.TURN_180:
                    theta = 180
                    if target_yaw_degrees is None:
                        initial_yaw_degrees = current_yaw % 360
                        target_yaw_degrees  = (initial_yaw_degrees + theta) % 360  # Target angle after the turn
                        effective_padding = min(PADDING, abs(theta) * 0.5)  # 50%以下に制限
                        rospy.logdebug(f"Start: {initial_yaw_degrees:.2f}, Target: {target_yaw_degrees:.2f}")
                    if theta >= 0:
                        diff = self.angle_diff(current_yaw, initial_yaw_degrees)
                        if diff >= theta - effective_padding:
                            state = State.STOP
                            next_state = State.IDEL
                            rospy.loginfo("state -> IDEL")
                            continue
                        else:
                            self.publish_message("turn_left")
                    elif theta < 0:
                        diff = self.angle_diff(initial_yaw_degrees, current_yaw) 
                        if diff >= abs(theta) - effective_padding:
                            state = State.STOP
                            next_state = State.IDEL
                            rospy.loginfo("state -> IDEL")
                            continue
                        else:
                            self.publish_message("turn_right")

                elif state == State.APPROACH_MARKER_0:
                    x, z = self.marker_detector.aruco_distance(0, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                    #rospy.logdebug(f"Marker 10: x={x:.2f}, z={z:.2f}")
                    if x is not None and z is not None:
                        if z <= DISTANCE_THRESHOLD_MARKER_10:
                            state = State.STOP
                            next_state = State.CALUCULATE_P
                            rospy.loginfo("state -> CALUCULATE_P")
                            continue
                        else:
                            if x < -1 * 10:
                                self.publish_message("slow_turn_left")
                            elif x > 10:
                                self.publish_message("slow_turn_right")
                            else: # -〇 < x < 〇
                                self.publish_message("move_forward")
                elif state == State.CALUCULATE_P:
                    if 3 in self.list_marker_ids and 4 in self.list_marker_ids:
                        x3, z3 = self.marker_detector.aruco_distance(3, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        x4, z4 = self.marker_detector.aruco_distance(4, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        point_a, point_b, point_m, point_p = draw_p.find_P(x3, z3, x4, z4, DISTANCE_POINT_P)
                        theta_p = draw_p.grayscale_image(point_a, point_b, point_m, point_p) *  (-1)
                        state = State.TURN_TO_P
                    else:
                        rospy.loginfo("目標点P算出のためのマーカーが認識できません")
                elif state == State.TURN_TO_P:
                    theta = theta_p
                    if target_yaw_degrees is None:
                        initial_yaw_degrees = current_yaw % 360
                        target_yaw_degrees  = (initial_yaw_degrees + theta) % 360  # Target angle after the turn
                        effective_padding = min(PADDING, abs(theta) * 0.5)  # 50%以下に制限
                        rospy.logdebug(f"Start: {initial_yaw_degrees:.2f}, Target: {target_yaw_degrees:.2f}")
                    if theta >= 0:
                        diff = self.angle_diff(current_yaw, initial_yaw_degrees)
                        if diff >= theta - effective_padding:
                            state = State.STOP
                            next_state = State.MOVE_TO_P
                            rospy.loginfo("state -> MOVE_TO_P")
                            continue
                        else:
                            self.publish_message("turn_left")
                    elif theta < 0:
                        diff = self.angle_diff(initial_yaw_degrees, current_yaw) 
                        if diff >= abs(theta) - effective_padding:
                            state = State.STOP
                            next_state = State.MOVE_TO_P
                            rospy.loginfo("state -> MOVE_TO_P")
                            continue
                        else:
                            self.publish_message("turn_right")
                elif state == State.MOVE_TO_P:
                    pass
                elif state == State.FACE_FORWARD:
                    pass
                elif state == State.POSITION_CHECK:
                    pass
                elif state == State.DOKING_STATION:
                    pass
                                
                self.rate.sleep()

        except rospy.ROSInterruptException:
            rospy.logwarn("ノードがCtrl+Cで停止されました")
        finally:
            rospy.loginfo("[メイン] 終了 -----------------------------------")

if __name__ == '__main__':
    Pilot().run()
