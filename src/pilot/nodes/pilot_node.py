#!/usr/bin/env python3
import rospy
from typing import Any, Tuple

import os
import sys
import rospy
import rospkg

# rospkg でパッケージの src を PYTHONPATH に追加（rosrun 対応）
try:
    pkg_path = rospkg.RosPack().get_path('ros_ugv')
    src_path = os.path.join(pkg_path, 'src') # トップレベルを src に設定
    if src_path not in sys.path:
        sys.path.insert(0, src_path)
except Exception as e:
    rospy.logwarn("PYTHONPATH setup failed: %s", e)


from pilot.app.core.state_machine import StateMachine
from pilot.app.states.state_type import StateType
from pilot.app.states.idle_state import IdleState
from pilot.app.states.approach_marker import ApproachMarkerState
from pilot.app.states.follow_row_center import FollowRowCenterState
from pilot.app.states.auto_charging import (
    AutoChargingState,
    ApproachEntryPointState,
    AdjustPositionState,
    DockingState,
    ChargingState,
    RetryingState,
)
from pilot.app.core.cmd_publisher import MotorCommandPublisher
from pilot.app.sensors.imu import IMUSensor
from pilot.app.sensors.realsense import CameraSensor
from pilot.app.perception.aruco_detector import ArucoDetector

"""
| pilot_node.py                    |
|----------------------------------|
| ① 各センサーを初期化               |
| ② 状態機械(StateMachine)を構築     |
| ③ メインループでセンサー読み込み    |
| ④ 状態に応じて制御コマンド生成     |
| ⑤ コマンドをPixhawkへ送信         |
"""

def build_state_machine() -> StateMachine:
    idle = IdleState()
    approach = ApproachMarkerState()

    follow_row = FollowRowCenterState()

    auto_chg = AutoChargingState()
    entry_p = ApproachEntryPointState()
    adjust = AdjustPositionState()
    docking = DockingState()
    charging = ChargingState()
    retrying = RetryingState()
    sm = StateMachine(
        state_map={
            StateType.IDLE: idle,
            StateType.APPROACH_MARKER: approach,
            StateType.FOLLOW_ROW_CENTER: follow_row,
            StateType.AUTO_CHARGING: auto_chg,
            StateType.APPROACH_ENTRY_POINT: entry_p,
            StateType.ADJUST_POSITION: adjust,
            StateType.DOCKING: docking,
            StateType.CHARGING: charging,
            StateType.RETRYING: retrying,
        },
        initial_state_type=StateType.IDLE
    )
    return sm

def main():
    rospy.init_node('pilot_node', anonymous=False)

    loop_hz = rospy.get_param('~loop_hz', 50)
    control_topic = rospy.get_param('~control_topic', '/control/command')

    # --- センサー初期化/初期化待ち --- #
    # IMU センサー
    imu = IMUSensor(topic='/mavros/imu/data')
    imu.start()
    while not imu.imu_initialized and not rospy.is_shutdown():
        rospy.loginfo_throttle(1, "Waiting for IMU initialization...")
        rospy.sleep(0.1)
    # Realsense カメラセンサー
    camera = CameraSensor(color_topic='/camera/color/image_raw', depth_topic='/camera/depth/image_rect_raw')
    camera.start()
    while (not camera.rgb_initialized or not camera.depth_initialized) and not rospy.is_shutdown():
        rospy.loginfo_throttle(1, "Waiting for Camera initialization...")
        rospy.sleep(0.1)
    # Aruco マーカー検出器
    aruco = ArucoDetector()

    # --- モーター制御 publisher --- #
    motor_pub = MotorCommandPublisher(topic=control_topic)

    # --- 状態機械 ---
    sm = build_state_machine()

    rate = rospy.Rate(loop_hz)

    while not rospy.is_shutdown():
        try:
            # --- センサー読み込み ---
            imu_data = imu.get_latest_data()
            cam_data = camera.get_latest_data()

            # 現在状態の要求に応じてマーカーの姿勢推定を行うかを切り替える（最適化）
            need_pose = False
            try:
                need_pose = sm.current_state.needs_pose_estimation()
            except Exception as e:
                rospy.logwarn_throttle(1.0, f"needs_pose_estimation check failed: {e}")
            # --- マーカー検出 ---
            markers = aruco.detect(cam_data, estimate_pose=need_pose)
            # tests = aruco.save_color_image(cam_data)

            # --- 状態機械で制御コマンド生成 ---
            cmd = sm.process(imu_data, cam_data, markers)

            # --- モーターに送信 (command, speed の2引数) ---
            motor_pub.publish(cmd, cmd.speed)
        except Exception as e:
            rospy.logerr('pilot_node loop error: %s', e)
        rate.sleep()

if __name__ == '__main__':
    main()