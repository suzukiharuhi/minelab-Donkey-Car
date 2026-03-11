#!/usr/bin/env python3
"""Pilot node (ROS-dependent).

The brain of the UGV.  Subscribes to:
  - ``/minelab/depth_features``  (std_msgs/String, JSON)
  - ``/minelab/marker_info``     (std_msgs/String, JSON)
  - ``/mavros/imu/data``         (sensor_msgs/Imu)

Runs the state machine (app layer, ROS-free) and publishes:
  - ``/control/command``         (ros_ugv/OperateCommand or std_msgs/String)
  - ``/minelab/pilot_status``    (std_msgs/String, JSON – diagnostics)
"""
import json
import os
import sys
import time
from typing import Optional

import rospy
import rospkg
from sensor_msgs.msg import Imu
from std_msgs.msg import String

try:
    _pkg_path = rospkg.RosPack().get_path("ros_ugv")
    if _pkg_path not in sys.path:
        sys.path.insert(0, _pkg_path)
except Exception as _e:
    rospy.logwarn("rospkg path setup failed: %s", _e)

from minelab.app.core.commands import ControlCommand
from minelab.app.core.safety_guard import SafetyGuard
from minelab.app.core.state_machine import StateMachine
from minelab.app.states.approach_marker_state import ApproachMarkerState
from minelab.app.states.auto_charging_state import AutoChargingState
from minelab.app.states.crop_navigation_state import CropNavigationState
from minelab.app.states.docking_state import DockingState
from minelab.app.states.idle_state import IdleState
from minelab.app.states.state_type import StateType
from minelab.interfaces.data_models import DepthFeatures, MarkerInfo, PilotStatus, SensorSnapshot

# Try to import the package-specific OperateCommand; fall back to String
try:
    from ros_ugv.msg import OperateCommand as _OperateCommandMsg
    _USE_OPERATE_CMD = True
except ImportError:
    _USE_OPERATE_CMD = False


class PilotNode:
    """UGV pilot: state machine + sensor fusion + command generation.

    Only this file (and the other node files) may import ``rospy``.
    All control logic lives in ``minelab/app/``.
    """

    def __init__(self) -> None:
        rospy.init_node("minelab_pilot_node", anonymous=False)

        # --- Parameters ---
        self._loop_hz: int = rospy.get_param("~loop_hz", 50)
        self._control_topic: str = rospy.get_param(
            "~control_topic", "/control/command"
        )
        self._session_tag: str = rospy.get_param("~session_tag", "minelab_run")
        self._depth_timeout: float = rospy.get_param("~depth_timeout_s", 1.0)

        # --- Latest sensor values (updated by callbacks) ---
        self._latest_imu: SensorSnapshot = SensorSnapshot()
        self._latest_depth: Optional[DepthFeatures] = None
        self._latest_marker: Optional[MarkerInfo] = None

        # --- App-layer components (ROS-free) ---
        self._safety = SafetyGuard(depth_timeout_s=self._depth_timeout)
        self._sm = self._build_state_machine()

        # --- ROS publishers ---
        if _USE_OPERATE_CMD:
            self._cmd_pub = rospy.Publisher(
                self._control_topic, _OperateCommandMsg, queue_size=5
            )
        else:
            self._cmd_pub = rospy.Publisher(
                self._control_topic, String, queue_size=5
            )
        self._status_pub = rospy.Publisher(
            "/minelab/pilot_status", String, queue_size=5
        )

        # --- ROS subscribers ---
        rospy.Subscriber(
            rospy.get_param("~imu_topic", "/mavros/imu/data"),
            Imu,
            self._imu_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~depth_features_topic", "/minelab/depth_features"),
            String,
            self._depth_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            rospy.get_param("~marker_info_topic", "/minelab/marker_info"),
            String,
            self._marker_callback,
            queue_size=5,
        )

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("[PilotNode] Initialised (loop_hz=%d)", self._loop_hz)

    # ------------------------------------------------------------------
    # State-machine construction
    # ------------------------------------------------------------------

    def _build_state_machine(self) -> StateMachine:
        sm = StateMachine(
            state_map={
                StateType.IDLE: IdleState(),
                StateType.CROP_NAVIGATION: CropNavigationState(),
                StateType.APPROACH_MARKER: ApproachMarkerState(),
                StateType.DOCKING: DockingState(),
                StateType.AUTO_CHARGING: AutoChargingState(),
            },
            initial_state_type=StateType.IDLE,
        )
        rospy.loginfo("[PilotNode] StateMachine built (initial=IDLE)")
        return sm

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _imu_callback(self, msg: Imu) -> None:
        import math
        q = msg.orientation
        # Quaternion → yaw (simplified, z-axis)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self._latest_imu = SensorSnapshot(
            imu_yaw=yaw,
            imu_roll=0.0,
            imu_pitch=0.0,
            imu_angular_velocity_z=msg.angular_velocity.z,
            imu_timestamp=msg.header.stamp.to_sec(),
        )

    def _depth_callback(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            self._latest_depth = DepthFeatures(
                range_array=d.get("range_array", []),
                gap_index=d.get("gap_index", 0),
                gap_angle_rad=d.get("gap_angle_rad", 0.0),
                gap_width_px=d.get("gap_width_px", 0),
                confidence=d.get("confidence", 0.0),
                timestamp=d.get("timestamp", time.monotonic()),
            )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[PilotNode] depth JSON parse error: %s", exc)

    def _marker_callback(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            self._latest_marker = MarkerInfo(
                marker_id=d.get("marker_id", -1),
                visible=d.get("visible", False),
                distance_z_cm=d.get("distance_z_cm", 0.0),
                distance_x_cm=d.get("distance_x_cm", 0.0),
                yaw_error_deg=d.get("yaw_error_deg", 0.0),
                pitch_deg=d.get("pitch_deg", 0.0),
                center_px=tuple(d.get("center_px", [0, 0])),
                timestamp=d.get("timestamp", time.monotonic()),
            )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[PilotNode] marker JSON parse error: %s", exc)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        rate = rospy.Rate(self._loop_hz)
        rospy.loginfo("[PilotNode] Main loop started at %d Hz", self._loop_hz)

        while not rospy.is_shutdown():
            try:
                self._step()
            except Exception as exc:
                rospy.logerr("[PilotNode] loop error: %s", exc)
            rate.sleep()

    def _step(self) -> None:
        # Safety guard check
        stop = self._safety.check(
            self._latest_depth,
            self._latest_marker,
            now=time.monotonic(),
        )
        if stop is not None:
            cmd = stop
        else:
            cmd = self._sm.process(
                self._latest_imu,
                self._latest_depth,
                self._latest_marker,
            )
            if cmd is None:
                rospy.logwarn_throttle(
                    1.0, "[PilotNode] StateMachine returned None → STOP"
                )
                cmd = ControlCommand.stop()

        self._publish_command(cmd)
        self._publish_status(cmd)

    def _publish_command(self, cmd: ControlCommand) -> None:
        cmd_str, spd_str = cmd.to_strings()
        if _USE_OPERATE_CMD:
            msg = _OperateCommandMsg()
            msg.command = cmd_str
            msg.speed = spd_str
            self._cmd_pub.publish(msg)
        else:
            self._cmd_pub.publish(f"{cmd_str} {spd_str}")

    def _publish_status(self, cmd: ControlCommand) -> None:
        cmd_str, spd_str = cmd.to_strings()
        status = PilotStatus(
            state_name=self._sm.current_type.name,
            last_command_type=cmd_str,
            last_command_speed=spd_str,
            pilot_heartbeat=rospy.get_time(),
            depth_features_last_received=(
                self._latest_depth.timestamp if self._latest_depth else 0.0
            ),
            marker_info_last_received=(
                self._latest_marker.timestamp if self._latest_marker else 0.0
            ),
            session_tag=self._session_tag,
        )
        import dataclasses
        self._status_pub.publish(json.dumps(dataclasses.asdict(status)))

    # ------------------------------------------------------------------

    def _on_shutdown(self) -> None:
        rospy.loginfo("[PilotNode] Shutting down.")


def main() -> None:
    node = PilotNode()
    node.run()


if __name__ == "__main__":
    main()
