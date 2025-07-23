#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
import time
import threading

from config.constant import *
from subscriber.sub_imu import IMUHandler
from subscriber.sub_realsense import RealSenseHandler
from aruco.aruco_detector import MarkerDetector
from state import State
from draw_p import GoalPointDrawer

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
        rospy.init_node('pilot_node')
        rospy.loginfo("pilot_nodeを初期化")
        self.rate = rospy.Rate(50) # 50Hz

        # 指令を送るPublisher
        self.command_pub = rospy.Publisher('/control/command', String, queue_size=10)
        self.msg = String()

        self.color = self.depth = self.marker_corners = self.list_marker_ids = None
        self.bridge = CvBridge()
        self.color_image = self.depth_image = None
        self.current_command = None

        # インスタンス初期化
        self.imu_handler = IMUHandler()
        self.realsense_hanlder = RealSenseHandler()
        self.marker_detector = MarkerDetector()
        self.gpd = GoalPointDrawer()

        # Realsense取得用スレッド
        self.realsense_thread = threading.Thread(target=self.get_realsense, daemon=False)

        rospy.on_shutdown(self.on_shutdown)  # シャットダウン時の処理を登録

    # publisher
    def publish_message(self, command):
        self.msg.data = command
        self.command_pub.publish(self.msg)
        if self.current_command != command:
            rospy.loginfo(f"{self.current_command} -> {command}")
        self.current_command = command
    # realsenseのフレーム取得
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
        if abs(current - reference) < 2:
            return 0
        diff = (current - reference + 360) % 360
        return diff
    
    def adjust_angle(self, x_distance, z_distance):
        if z_distance > 150:
            threshold = 20
        else:
            threshold = 10
        if x_distance < (-1 * threshold):
            self.publish_message("slow_turn_left")
        elif x_distance > threshold:
            self.publish_message("slow_turn_right")
        else: # -〇 < x < 〇
            self.publish_message("move_forward")   
       
    def on_shutdown(self):
        print("[on_shutdown] STOPコマンドを送信中...")
        stop_msg = String()
        stop_msg.data = "stop"
        rate = rospy.Rate(10)
        for _ in range(30):  # 10Hzで3秒送信
            self.command_pub.publish(stop_msg)
            rate.sleep()
        print("[on_shutdown] 完了")

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

            while not rospy.is_shutdown():         
                current_yaw = self.imu_handler.get_yaw()

                if state == State.STOP:
                    current_time = time.time()
                    self.publish_message("stop")
                    if not pause_sent:
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
                    if 10 in self.list_marker_ids:
                        x, z = self.marker_detector.aruco_distance(10, 0.15, self.marker_corners, self.list_marker_ids)
                        if z <= DISTANCE_THRESHOLD_MARKER_10:
                            state = State.STOP
                            next_state = State.TURN_180
                            rospy.loginfo("state -> TURN_180")
                            continue
                        else:
                            self.adjust_angle(x, z)
                    else:
                        #state10の間，10を認識できなければ停止
                        self.publish_message("stop")  
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
                            target_yaw_degrees = None
                            continue
                        else:
                            self.publish_message("turn_left")
                    elif theta < 0:
                        diff = self.angle_diff(initial_yaw_degrees, current_yaw) 
                        if diff >= abs(theta) - effective_padding:
                            state = State.STOP
                            next_state = State.IDEL
                            rospy.loginfo("state -> IDEL")
                            target_yaw_degrees = None
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
                            self.adjust_angle(x, z)
                    else:
                        self.publish_message("stop")  
                elif state == State.CALUCULATE_P:
                    if 3 in self.list_marker_ids and 4 in self.list_marker_ids:
                        x3, z3 = self.marker_detector.aruco_distance(3, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        x4, z4 = self.marker_detector.aruco_distance(4, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        point_a, point_b, point_m, point_p = self.gpd.find_goal_point(x3, z3, x4, z4, DISTANCE_POINT_P)
                        theta_p = self.gpd.draw(point_a, point_b, point_m, point_p) *  (-1)
                        state = State.TURN_TO_P
                    else:
                        rospy.loginfo("目標点P算出のためのマーカーが認識できません")
                        self.publish_message("stop")  
                        # 両方見えるように旋回．バック
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
                    for marker_id in [3, 4]:
                        if marker_id in self.list_marker_ids:
                            _, marker_distance = self.marker_detector.aruco_distance(marker_id, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                            break
                    else:
                        self.publish_message("stop")
                    if marker_distance <= DISTANCE_POINT_P:
                        state = State.STOP
                        next_state = State.FACE_FORWARD
                        rospy.loginfo("state -> FACE_FORWARD")
                        continue
                    else:
                        self.publish_message("move_forward")
                    # 連続で3も4も見つからないときリトライ
                elif state == State.FACE_FORWARD:

                    if 0 in self.list_marker_ids and 3 in self.list_marker_ids and 4 in self.list_marker_ids:
                        x0, _ = self.marker_detector.aruco_distance(0, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        x3, _ = self.marker_detector.aruco_distance(3, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        x4, _ = self.marker_detector.aruco_distance(4, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        if -3 <= x0 <= 3:
                            state = State.STOP
                            next_state = State.POSITION_CHECK
                            continue
                        elif abs(x3) < abs(x4):
                            self.publish_message("slow_turn_right")
                        else:
                            self.publish_message("slow_turn_left")
                    elif 3 in self.list_marker_ids:
                        self.publish_message("slow_turn_right")
                    elif 4 in self.list_marker_ids:
                        self.publish_message("slow_turn_left")
                        
                elif state == State.POSITION_CHECK:
                    if 3 in self.list_ids and 4 in self.list_ids and 0 in self.list_ids:
                        x3, z3 = self.marker_detector.aruco_distance(3, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        x4, z4 = self.marker_detector.aruco_distance(4, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        x0, z0 = self.marker_detector.aruco_distance(0, MARKER_SIZE_ACTUAL, self.marker_corners, self.list_marker_ids)
                        if abs(z3-z4) < 1.5 and abs(x0) < 1.5:
                            state = State.STOP
                            next_state = State.DOKING_STATION
                            continue
                        else:
                            if abs(z3-z4) >= 1.5:
                                print("not z")
                            elif abs(x0) >= 1.5:
                                print("not x")
                            state = State.STOP
                            next_state = State.RETRY
                    else:
                        state = State.STOP
                        next_state = State.RETRY

                elif state == State.DOKING_STATION:
                    if 7 in self.list_ids:
                        x7, z7 = self.marker_detector.aruco_distance(0, MARKER_SIZE_SMALL, self.marker_corners, self.list_marker_ids)
                        if z7 <= DISTANCE_THRESHOLD_MARKER_7:
                            state = State.STOP
                            next_state = State.CHARGING
                        else:
                            self.publish_message("slow_move_forward")
                    else:
                        self.publish_message("stop")
                # elif state == State.CHARGING:
                #     if illminance.is_above_threshold(threshold=20):
                #         print("charging OK   OOOOOOOOOOO")
                #     else:
                #         print("Charging complete not confirmed. Continuing operation.")

                elif state == State.RETRY:
                    pass
                                
                self.rate.sleep()
            self.publish_message("stop")
        except rospy.ROSInterruptException:
            rospy.logwarn("ノードがCtrl+Cで停止されました")
        finally:
            for _ in range(100):
                self.publish_message("stop")
                self.rate.sleep()
            rospy.loginfo("[メイン] 終了 --------------------------------------------------------")

if __name__ == '__main__':
    Pilot().run()
