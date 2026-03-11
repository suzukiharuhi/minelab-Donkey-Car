#!/usr/bin/env python3
"""Vehicle I/O node (ROS-dependent).

Subscribes to ``/control/command`` and forwards commands to Pixhawk
via ``/mavros/rc/override`` (``mavros_msgs/OverrideRCIn``).

Uses ``RCOverrideMapper`` from the app layer (ROS-free) to translate
``ControlCommand`` objects (or the string-encoded variant published by
the pilot node) into PWM channel values.
"""
import os
import sys

import rospy
import rospkg
from std_msgs.msg import String

try:
    _pkg_path = rospkg.RosPack().get_path("ros_ugv")
    if _pkg_path not in sys.path:
        sys.path.insert(0, _pkg_path)
except Exception as _e:
    rospy.logwarn("rospkg path setup failed: %s", _e)

from minelab.app.actuators.rc_override_mapper import RCOverrideMapper
from minelab.app.core.commands import CommandType, ControlCommand, SpeedLevel

try:
    from mavros_msgs.msg import OverrideRCIn
    _MAVROS_AVAILABLE = True
except ImportError:
    _MAVROS_AVAILABLE = False
    rospy.logwarn(
        "[VehicleIONode] mavros_msgs not found – RC override publishing disabled"
    )

# Try ros_ugv OperateCommand first
try:
    from ros_ugv.msg import OperateCommand as _OperateCommandMsg
    _USE_OPERATE_CMD = True
except ImportError:
    _USE_OPERATE_CMD = False

_CMD_TYPE_MAP = {ct.name.lower(): ct for ct in CommandType}
_SPD_MAP = {sl.name.lower(): sl for sl in SpeedLevel}


class VehicleIONode:
    """Translate high-level commands to RC override PWM values."""

    def __init__(self) -> None:
        rospy.init_node("minelab_vehicle_io_node", anonymous=False)

        self._control_topic: str = rospy.get_param(
            "~control_topic", "/control/command"
        )
        self._rc_topic: str = rospy.get_param(
            "~rc_override_topic", "/mavros/rc/override"
        )
        self._loop_hz: int = rospy.get_param("~loop_hz", 50)

        self._mapper = RCOverrideMapper()
        self._latest_cmd: ControlCommand = ControlCommand.stop()

        # Publisher
        if _MAVROS_AVAILABLE:
            self._rc_pub = rospy.Publisher(
                self._rc_topic, OverrideRCIn, queue_size=5
            )

        # Subscriber – accept either OperateCommand or String
        if _USE_OPERATE_CMD:
            rospy.Subscriber(
                self._control_topic,
                _OperateCommandMsg,
                self._operate_cmd_callback,
                queue_size=5,
            )
        else:
            rospy.Subscriber(
                self._control_topic,
                String,
                self._string_cmd_callback,
                queue_size=5,
            )

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            "[VehicleIONode] Listening on %s, publishing to %s",
            self._control_topic,
            self._rc_topic,
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _operate_cmd_callback(self, msg) -> None:
        cmd_type = _CMD_TYPE_MAP.get(
            msg.command.lower(), CommandType.NO_OPERATION
        )
        speed = _SPD_MAP.get(msg.speed.lower(), SpeedLevel.MID)
        self._latest_cmd = ControlCommand(cmd_type, speed)

    def _string_cmd_callback(self, msg: String) -> None:
        parts = msg.data.strip().split()
        if not parts:
            return
        cmd_type = _CMD_TYPE_MAP.get(parts[0].lower(), CommandType.NO_OPERATION)
        speed = (
            _SPD_MAP.get(parts[1].lower(), SpeedLevel.MID)
            if len(parts) > 1
            else SpeedLevel.MID
        )
        self._latest_cmd = ControlCommand(cmd_type, speed)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        rate = rospy.Rate(self._loop_hz)
        rospy.loginfo("[VehicleIONode] Main loop at %d Hz", self._loop_hz)

        while not rospy.is_shutdown():
            try:
                self._apply_command(self._latest_cmd)
            except Exception as exc:
                rospy.logerr("[VehicleIONode] loop error: %s", exc)
            rate.sleep()

    def _apply_command(self, cmd: ControlCommand) -> None:
        if not _MAVROS_AVAILABLE:
            return
        channels = self._mapper.build_channels(cmd)
        msg = OverrideRCIn()
        msg.channels = channels
        self._rc_pub.publish(msg)

    def _on_shutdown(self) -> None:
        rospy.loginfo("[VehicleIONode] Shutdown – sending stop.")
        if _MAVROS_AVAILABLE:
            for _ in range(10):
                self._apply_command(ControlCommand.stop())
                rospy.sleep(0.05)


def main() -> None:
    node = VehicleIONode()
    node.run()


if __name__ == "__main__":
    main()
