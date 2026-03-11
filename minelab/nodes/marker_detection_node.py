#!/usr/bin/env python3
"""Marker-detection node (ROS-dependent).

Subscribes to the colour camera topic, calls ``ArucoDetector`` (app
layer, ROS-free), and publishes detected markers as a JSON string on
``/minelab/marker_info`` (``std_msgs/String``).
"""
import json
import os
import sys

import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    _pkg_path = rospkg.RosPack().get_path("ros_ugv")
    if _pkg_path not in sys.path:
        sys.path.insert(0, _pkg_path)
except Exception as _e:
    rospy.logwarn("rospkg path setup failed: %s", _e)

from minelab.app.perception.aruco_detector import ArucoDetector


class MarkerDetectionNode:
    """Subscribe to colour image and publish ArUco detections as JSON."""

    def __init__(self) -> None:
        rospy.init_node("marker_detection_node", anonymous=False)

        self._color_topic: str = rospy.get_param(
            "~color_topic", "/camera/color/image_raw"
        )
        self._pub_topic: str = rospy.get_param(
            "~marker_topic", "/minelab/marker_info"
        )
        self._target_id: int = rospy.get_param("~target_marker_id", -1)
        self._estimate_pose: bool = rospy.get_param("~estimate_pose", True)

        self._detector = ArucoDetector()
        self._bridge = CvBridge()

        self._pub = rospy.Publisher(self._pub_topic, String, queue_size=5)
        rospy.Subscriber(
            self._color_topic,
            Image,
            self._color_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        rospy.loginfo(
            "[MarkerDetectionNode] Listening on %s, publishing to %s",
            self._color_topic,
            self._pub_topic,
        )

    # ------------------------------------------------------------------

    def _color_callback(self, msg: Image) -> None:
        try:
            color_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[MarkerDetectionNode] CvBridge error: %s", exc)
            return

        timestamp = msg.header.stamp.to_sec()
        markers = self._detector.detect(
            color_bgr,
            estimate_pose=self._estimate_pose,
            timestamp=timestamp,
        )

        # Filter to target marker if configured
        if self._target_id >= 0:
            markers = [m for m in markers if m.marker_id == self._target_id]

        if not markers:
            # Publish a "not visible" placeholder so pilot knows marker is absent
            payload = json.dumps(
                {
                    "marker_id": self._target_id,
                    "visible": False,
                    "distance_z_cm": 0.0,
                    "distance_x_cm": 0.0,
                    "yaw_error_deg": 0.0,
                    "pitch_deg": 0.0,
                    "center_px": [0, 0],
                    "timestamp": timestamp,
                }
            )
        else:
            m = markers[0]
            payload = json.dumps(
                {
                    "marker_id": m.marker_id,
                    "visible": m.visible,
                    "distance_z_cm": m.distance_z_cm,
                    "distance_x_cm": m.distance_x_cm,
                    "yaw_error_deg": m.yaw_error_deg,
                    "pitch_deg": m.pitch_deg,
                    "center_px": list(m.center_px),
                    "timestamp": m.timestamp,
                }
            )

        self._pub.publish(payload)

    # ------------------------------------------------------------------

    def run(self) -> None:
        rospy.spin()


def main() -> None:
    node = MarkerDetectionNode()
    node.run()


if __name__ == "__main__":
    main()
