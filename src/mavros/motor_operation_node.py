#!/usr/bin/env python3
"""
Motor Operation Node

機能:
- /control/command (std_msgs/String) を購読
- /rc/override (mavros_msgs/OverrideRCIn) に出力
- 受信コマンドに応じて RC チャンネル値を適用

構成:
- IMUSubscriber: /mavros/imu/data の受信と初期化待ち
- CommandStore: /control/command の受信と差分検知
- MotorOutputController: RC Override の生成と publish
"""

from dataclasses import dataclass
import rospy
import signal
import sys

from motor_output import MotorOutputController
from subscribers.imu_subscriber import IMUSubscriber
from subscribers.command_subscriber import ControlCommandSubscriber


@dataclass
class MotorOperationConfig:
    loop_hz = rospy.get_param('~loop_hz', 50)
    control_topic = rospy.get_param('~control_topic', '/control/command')
    rc_override_topic = rospy.get_param('~rc_override_topic', '/mavros/rc/override')

class MotorOperationNode:
    def __init__(self):
        self.config = MotorOperationConfig()
        self.imu_sub = IMUSubscriber()
        self.cmd_sub = ControlCommandSubscriber(topic=self.config.control_topic)
        self.motor = MotorOutputController(topic=self.config.rc_override_topic)
        self.rate = rospy.Rate(self.config.loop_hz)

    def start(self):
        self.imu_sub.start()
        self.cmd_sub.start()

    def spin(self):
        rospy.loginfo("Entering main loop.")
        while not rospy.is_shutdown():
            # コマンド適用 / 差分ログ
            command = self.cmd_sub.command
            speed = self.cmd_sub.speed
            if command is not None and speed is not None:
                self.motor.run_motor(command, speed)
            else:
                self.motor.stop()
            self.rate.sleep()


def _on_shutdown(app: MotorOperationNode):
    print("Shutdown: sending stop commands...")
    try:
        for _ in range(20):
            app.motor.stop()
            rospy.sleep(0.1)
    except Exception as e:
        print("[Shutdown Error]", e)

def _sigint_handler(sig, frame):
    """Ctrl+C で呼ばれる handler"""
    print("\n[Ctrl+C] SIGINT received. Requesting shutdown...")
    rospy.signal_shutdown("SIGINT received")
    sys.exit(0)


def main():
    rospy.init_node("motor_operation_node")
    rospy.loginfo("motor_operation_node 起動 ----------------------------")
    rospy.on_shutdown(lambda: _on_shutdown(app)) # シャットダウン処理登録．ノード終了時に_on_shutdown(app)が実行される
    # Ctrl+C の安全処理
    signal.signal(signal.SIGINT, _sigint_handler)

    app = MotorOperationNode()
    # --- Subscribersの初期化 --- #
    app.start()
    # --- IMUとCommandの初期化待ち --- #
    while not app.imu_sub.is_active:
        rospy.loginfo_throttle(1, "Waiting for IMU initialization...")
    while not app.cmd_sub.is_active:
        rospy.loginfo_throttle(1, "Waiting for Command initialization...")

    try:
        app.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("Interrupted")
    finally:
        rospy.loginfo("終了 -----------------------------------")


if __name__ == "__main__":
    main()
