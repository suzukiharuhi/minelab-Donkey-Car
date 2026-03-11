#!/usr/bin/env python3
"""Logger node (ROS-dependent).

Subscribes to the minelab JSON topics and writes records to disk using
the ``SessionLogger`` from the app layer.

Topics:
  - ``/minelab/depth_features``  (std_msgs/String)
  - ``/minelab/marker_info``     (std_msgs/String)
  - ``/minelab/pilot_status``    (std_msgs/String)
"""
import json
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

from minelab.app.logging.session_logger import SessionLogger


class LoggerNode:
    """Subscribe to all minelab topics and persist records to disk."""

    def __init__(self) -> None:
        rospy.init_node("minelab_logger_node", anonymous=False)

        log_root: str = rospy.get_param("~log_root", "/tmp/minelab_logs")
        session_tag: str = rospy.get_param("~session_tag", "minelab_run")
        verbose: bool = rospy.get_param("~verbose", False)

        self._logger = SessionLogger(
            log_root=log_root,
            session_tag=session_tag,
            save_csv=True,
            verbose=verbose,
        )

        rospy.Subscriber(
            rospy.get_param("~depth_features_topic", "/minelab/depth_features"),
            String,
            self._depth_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~marker_info_topic", "/minelab/marker_info"),
            String,
            self._marker_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~pilot_status_topic", "/minelab/pilot_status"),
            String,
            self._status_callback,
            queue_size=10,
        )

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("[LoggerNode] Recording to %s/%s", log_root, session_tag)

    # ------------------------------------------------------------------

    def _depth_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._logger.log_event("depth_features", data, state="")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[LoggerNode] depth parse error: %s", exc)

    def _marker_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._logger.log_event("marker_info", data, state="")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[LoggerNode] marker parse error: %s", exc)

    def _status_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            state = data.pop("state_name", "")
            self._logger.log_metric(data, state=state, tag="pilot_status")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[LoggerNode] status parse error: %s", exc)

    # ------------------------------------------------------------------

    def run(self) -> None:
        rospy.spin()

    def _on_shutdown(self) -> None:
        rospy.loginfo("[LoggerNode] Closing session.")
        self._logger.close()


def main() -> None:
    node = LoggerNode()
    node.run()


if __name__ == "__main__":
    main()
