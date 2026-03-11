import rospy
import subprocess
import datetime
from typing import Optional, List

from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType
from pilot.app.core.commands import ControlCommand
from pilot.app.utils.data_models import ArUcoMarker, IMUData, CameraData
from pilot.app.sensors.illuminance import BH1750Sensor, is_charging
import pilot.config.config as config

from stepper.stepper import StepperDriver
from .session_logger import get_logger

class ChargingState(BaseState):
    """充電状態（ドッキング後の停止と充電検証）"""
    def __init__(self):
        super().__init__()
        self._checked: bool = False
        self._result_ok: bool = False
        self._sensor: Optional[BH1750Sensor] = None
        self._sensors_stopped: bool = False
        self._motor: Optional[StepperDriver] = None
        self._failure_logged: bool = False
        # 設定（launchやパラメータサーバから変更可能）
        # self._sensor_nodes_to_kill: List[str] = rospy.get_param('/charging/sensor_nodes', [
        #     '/camera/realsense2_camera',  # 実際のノード名に合わせて変更
        #     '/rplidarNode'
        # ])
        # self._resume_after_sec: Optional[int] = rospy.get_param('/charging/resume_after_sec', None)
        # self._resume_at_time: Optional[str] = rospy.get_param('/charging/resume_at_time', None)  # "HH:MM" 24h

    def needs_pose_estimation(self) -> bool:
        return False

    def enter(self):
        """状態開始"""
        rospy.loginfo("[CHARGING] Entered ----------")
        rospy.sleep(1.0) # 車両停止待ち
        self._checked = False
        self._result_ok = False
        self._failure_logged = False
        # センサーセットアップ
        try:
            self._sensor = BH1750Sensor()
        except Exception as e:
            rospy.logwarn(f"[CHARGING] Illuminance sensor init failed: {e}")
            self._sensor = None
        self._sensors_stopped = False
        self._start_time = rospy.Time.now()
        try:
            get_logger().record_charging_attempt()
        except Exception as exc:
            rospy.logwarn(f"[CHARGING] logger record_charging_attempt failed: {exc}")

    def execute(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> ControlCommand:
        """
        状態動作:
        - 充電判定を行う
        - 充電中は停止を維持
        """
        if not self._checked:
            # 3秒観測で充電判定を実施
            if self._sensor is None:
                # センサーが利用できない
                rospy.logwarn("[CHARGING] Sensor unavailable → treat as not charging")
                return ControlCommand.stop()
            else:
                try:
                    self._result_ok = is_charging(self._sensor, threshold_lx=40.0, duration_sec=5.0, sample_rate_hz=5.0)
                except Exception as e:
                    rospy.logwarn(f"[CHARGING] is_charging failed: {e}")
                    self._result_ok = False
            self._checked = True
            rospy.loginfo(f"[CHARGING] Charging result: {'OK' if self._result_ok else 'NG'}")
            # # 充電成功ならセンサノード停止（省電力化）
            # if self._result_ok and not self._sensors_stopped:
            #     self._stop_sensor_nodes()
            #     self._sensors_stopped = True
        return ControlCommand.stop()

    def check_transition(self, imu_data: Optional[IMUData], camera_data: Optional[CameraData], markers: List[ArUcoMarker]) -> Optional[StateType]:
        """次状態への遷移判定
        遷移条件：
          - 充電成功(_result_ok)ならIDLEへ
          - 充電失敗ならRETRYINGへ
        """
        # 充電判定がまだなら遷移しない
        if not self._checked:
            return None
        # 充電失敗→リトライ
        if not self._result_ok:
            rospy.loginfo("[CHARGING] Charging NG → retry")
            if not self._failure_logged:
                try:
                    get_logger().record_charging_failure()
                except Exception as exc:
                    rospy.logwarn(f"[CHARGING] logger record_charging_failure failed: {exc}")
                self._failure_logged = True
            return StateType.RETRYING
        # 充電中は原則遷移しない（停止維持）。再開条件のみ許可。
        try:
            get_logger().record_charging_success()
            path = get_logger().finalize_session()
            if path:
                rospy.loginfo(f"[CHARGING] Session logged → {path}")
        except Exception as exc:
            rospy.logwarn(f"[CHARGING] logger finalize failed: {exc}")
        rospy.loginfo("[CHARGING] Charging OK → withdrawing from charger")
        return StateType.WITHDRAWAL
        
    def exit(self):
        """状態終了"""
        if self._result_ok:
            rospy.loginfo("[CHARGING] Charging successful.")
            self._motor = StepperDriver()
            if config.stepper == 1:
                print("stepper up")
                self._motor.up(7)
                self._motor.wait(1)
                config.stepper = 2
            elif config.stepper == 2:
                print("stepper down")
                self._motor.down(7)
                self._motor.wait(1)
                config.stepper = 1
            self._motor.close()

        rospy.loginfo("[CHARGING] Exiting")

    # --- 内部ユーティリティ ---
    def _stop_sensor_nodes(self) -> None:
        """センサーノードを停止して省電力化"""
        for name in self._sensor_nodes_to_kill:
            try:
                subprocess.call(['rosnode', 'kill', name])
                rospy.loginfo(f"[CHARGING] Stopped sensor node: {name}")
            except Exception as e:
                rospy.logwarn(f"[CHARGING] Failed to stop node {name}: {e}")

    def _should_resume(self) -> bool:
        """再開条件チェック（時間経過 or 指定時刻）。どちらか満たせばTrue"""
        # 経過秒
        if self._resume_after_sec is not None:
            elapsed = (rospy.Time.now() - self._start_time).to_sec()
            if elapsed >= int(self._resume_after_sec):
                return True
        # 指定時刻（毎日 HH:MM）
        if self._resume_at_time:
            try:
                hh, mm = map(int, self._resume_at_time.split(':'))
                now = datetime.datetime.now()
                if now.hour == hh and now.minute == mm:
                    return True
            except Exception:
                pass
        return False

