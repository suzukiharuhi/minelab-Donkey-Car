#!/usr/bin/env python3
import os
import sys
import cv2
import numpy as np
import rospy
import rospkg

# rospkg でパッケージの src を PYTHONPATH に追加（rosrun 対応）
try:
    pkg_path = rospkg.RosPack().get_path('ros_ugv')
    src_path = os.path.join(pkg_path, 'src')
    if src_path not in sys.path:
        sys.path.insert(0, src_path)
except Exception as e:
    rospy.logwarn("PYTHONPATH setup failed: %s", e)

from pathlib import Path

from pilot.app.core.cmd_publisher import MotorCommandPublisher
from pilot.app.core.commands import ControlCommand
from pilot.app.core.state_machine import StateMachine
from pilot.app.logging import LoggingConfig, SessionLogger
from pilot.app.perception.aruco_detector import ArucoDetector
from pilot.app.sensors.imu import IMUSensor
from pilot.app.sensors.battery import BatterySensor
from pilot.app.sensors.realsense import CameraSensor
from pilot.app.states.approach_marker import ApproachMarkerState
from pilot.app.states.auto_charging import (
    AdjustPositionState,
    ApproachEntryPointState,
    AutoChargingState,
    ChargingState,
    DockingState,
    RetryingState,
)
from pilot.app.states.crop_navigation import CropNavigationState
from pilot.app.states.idle_state import IdleState
from pilot.app.states.rotate180 import Rotate180State
from pilot.app.states.state_type import StateType
from pilot.app.states.withdrawal import WithdrawalState


class PilotNode:
    """
    UGV パイロットノード．

    責務:
        ① 各センサーの初期化・初期化待ち
        ② SessionLogger の初期化
        ③ 状態機械 (StateMachine) の構築
        ④ メインループでセンサー読み込み → 制御コマンド生成 → 送信
    """

    def __init__(self) -> None:
        rospy.init_node('pilot_node', anonymous=False)

        # --- ROSパラメータ ---
        self._loop_hz: int = rospy.get_param('~loop_hz', 50)
        self._control_topic: str = rospy.get_param('~control_topic', '/control/command')
        self._session_tag: str = rospy.get_param('~session_tag', 'field_run')
        self._log_root: str = rospy.get_param('~log_root', '/home/pi/ugv_ws/logging')

        # --- 各コンポーネント初期化 ---
        self._imu = self._init_imu()
        # self._battery = self._init_battery_sensor()
        self._camera = self._init_camera()
        self._aruco = ArucoDetector()
        self._motor_pub = MotorCommandPublisher(topic=self._control_topic)
        self._logger = self._init_logger()
        self._sm = self._build_state_machine()

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("[PilotNode] Initialization complete.")

    # ------------------------------------------------------------------
    # 初期化ヘルパー
    # ------------------------------------------------------------------

    def _init_imu(self) -> IMUSensor:
        """IMUセンサーを初期化し，データ受信を待つ．"""
        imu = IMUSensor(topic='/mavros/imu/data')
        imu.start()
        while not imu.imu_initialized and not rospy.is_shutdown():
            rospy.loginfo_throttle(1, "Waiting for IMU initialization...")
        rospy.loginfo("[PilotNode] IMU ready.")
        return imu
    
    def _init_battery_sensor(self) -> BatterySensor:
        """バッテリーセンサーを初期化し，データ受信を待つ．"""
        battery = BatterySensor(topic='/mavros/battery')
        battery.start()
        while not battery.battery_initialized and not rospy.is_shutdown():
            rospy.loginfo_throttle(1, "Waiting for Battery sensor initialization...")
        rospy.loginfo("[PilotNode] Battery sensor ready.")
        return battery

    def _init_camera(self) -> CameraSensor:
        """RealSenseカメラを初期化し，RGB・深度データ受信を待つ．"""
        camera = CameraSensor(
            color_topic='/camera/color/image_raw',
            depth_topic='/camera/depth/image_rect_raw',
        )
        camera.start()
        while (not camera.rgb_initialized or not camera.depth_initialized) \
                and not rospy.is_shutdown():
            rospy.loginfo_throttle(1, "Waiting for Camera initialization...")
        rospy.loginfo("[PilotNode] Camera ready.")
        return camera

    def _init_logger(self) -> SessionLogger:
        """SessionLogger を初期化し，セッションを開始する．"""
        config = LoggingConfig(
            log_root=Path(self._log_root),
            save_images=True,
            save_csv=True,
            verbose=False,
        )
        logger = SessionLogger.initialize(config)
        logger.start_session(session_tag=self._session_tag)
        return logger

    def _build_state_machine(self) -> StateMachine:
        """状態機械を構築して返す．"""
        sm = StateMachine(
            state_map={
                StateType.IDLE:                 IdleState(),
                StateType.APPROACH_MARKER:      ApproachMarkerState(),
                StateType.ROTATE180:            Rotate180State(),
                StateType.WITHDRAWAL:           WithdrawalState(),
                # StateType.FTGI:                 FTGIState(),
                # StateType.OBSTACLE_AVOIDANCE:   ObstacleAvoidanceState(),
                StateType.CROP_NAVIGATION:      CropNavigationState(),
                StateType.AUTO_CHARGING:        AutoChargingState(),
                StateType.APPROACH_ENTRY_POINT: ApproachEntryPointState(),
                StateType.ADJUST_POSITION:      AdjustPositionState(),
                StateType.DOCKING:              DockingState(),
                StateType.CHARGING:             ChargingState(),
                StateType.RETRYING:             RetryingState(),
            },
            initial_state_type=StateType.CROP_NAVIGATION,  # デバッグ用にCROP_NAVIGATIONから開始
        )
        rospy.loginfo("[PilotNode] StateMachine built.")
        return sm

    # ------------------------------------------------------------------
    # メインループ
    # ------------------------------------------------------------------

    def run(self) -> None:
        """メインループを起動する．"""
        rate = rospy.Rate(self._loop_hz)
        rospy.loginfo(f"[PilotNode] Main loop start at {self._loop_hz} Hz.")

        while not rospy.is_shutdown():
            try:
                print(f"time: {rospy.get_time():.2f}")
                self._step()
                self._frame_id = self._logger.next_frame()
            except Exception as e:
                rospy.logerr("[PilotNode] loop error: %s", e)
            rate.sleep()

    def _step(self) -> None:
        """1ループ分の処理"""
        # --- センサー読み込み ---
        imu_data = self._imu.get_latest_data()
        # battery_data = self._battery.get_latest_data()
        cam_data = self._camera.get_latest_data()

        # --- ポーズ推定が必要か確認（最適化） ---
        need_pose = self._check_needs_pose()

        # --- マーカー検出 ---
        markers = self._aruco.detect(cam_data, estimate_pose=need_pose)

        # --- 状態機械で制御コマンド生成 ---
        cmd = self._sm.process(imu_data, cam_data, markers)

        # 安全ガード: cmd が None の場合は停止コマンドにフォールバック
        if cmd is None:
            rospy.logwarn_throttle(1.0, "[PilotNode] StateMachine returned None → STOP fallback")
            cmd = ControlCommand.stop()

        # --- ロギング ---
        if cam_data is not None:
            pass
            # self._logger.log_image(cam_data.color_image, tag="rgb", state="", subdir="raw_images")
            # # depth_norm = cv2.normalize(cam_data.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            # # depth_norm = depth_norm.astype(np.uint8)
            # # depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            # self._logger.log_image(cam_data.depth_image, tag="depth", state="", subdir="raw_images")
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                temp_str = f.read()
            self._logger.log_metric(
                state="",
                tag="internal",
                metrics={
                    "cpu_temp": float(temp_str) / 1000.0,
                    # "battery_voltage": battery_data.voltage if battery_data else None,
                    # "battery_percentage": battery_data.percentage if battery_data else None,
                }
            )
        except:
            pass
        
        # --- モーターへ送信 ---
        self._motor_pub.publish(cmd, cmd.speed)

    def _check_needs_pose(self) -> bool:
        """現在状態がポーズ推定を必要とするか確認する．"""
        try:
            return self._sm.current_state.needs_pose_estimation()
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[PilotNode] needs_pose_estimation check failed: {e}")
            return False

    # ------------------------------------------------------------------
    # シャットダウン
    # ------------------------------------------------------------------

    def _on_shutdown(self) -> None:
        """ROSシャットダウン時のクリーンアップ．"""
        rospy.loginfo("[PilotNode] Shutting down...")
        self._logger.end_session(success=True, note="normal shutdown")
        rospy.loginfo("[PilotNode] Session saved.")


# ----------------------------------------------------------------------
# エントリーポイント
# ----------------------------------------------------------------------

def main() -> None:
    node = PilotNode()
    node.run()


if __name__ == '__main__':
    main()