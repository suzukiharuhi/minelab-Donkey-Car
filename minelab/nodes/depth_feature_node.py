#!/usr/bin/env python3
"""Depth-feature node (ROS-dependent).

Subscribes to ``/camera/depth/image_rect_raw``, runs
``DepthFeatureExtractor`` to extract the FTG-i gap result, and
publishes the result as a JSON string on ``/minelab/depth_features``
(``std_msgs/String``).  Optionally publishes a debug image.

Topic configuration is loaded from ``minelab/config/topics.yaml`` when
the parameter ``~config_path`` points to that file; otherwise defaults
are used.
"""
import json
import os
import sys

import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Resolve package root so minelab.* imports work under rosrun
try:
    _pkg_path = rospkg.RosPack().get_path("ros_ugv")
    _root = os.path.join(_pkg_path)
    if _root not in sys.path:
        sys.path.insert(0, _root)
except Exception as _e:
    rospy.logwarn("rospkg path setup failed: %s", _e)

from minelab.app.perception.depth_feature_extractor import DepthFeatureExtractor


class DepthFeatureNode:
    """Subscribe to depth image, extract features, publish JSON result."""

    def __init__(self) -> None:
        rospy.init_node("depth_feature_node", anonymous=False)

        # Parameters
        self._depth_topic: str = rospy.get_param(
            "~depth_topic", "/camera/depth/image_rect_raw"
        )
        self._pub_topic: str = rospy.get_param(
            "~features_topic", "/minelab/depth_features"
        )
        self._debug_topic: str = rospy.get_param(
            "~debug_image_topic", "/minelab/depth_debug"
        )
        self._publish_debug: bool = rospy.get_param("~publish_debug", False)

        roi_top: float = rospy.get_param("~roi_top_ratio", 0.3)
        roi_bottom: float = rospy.get_param("~roi_bottom_ratio", 0.7)
        obstacle_dist: float = rospy.get_param("~obstacle_dist_m", 0.8)
        min_gap: int = rospy.get_param("~min_gap_width_px", 20)
        hfov: float = rospy.get_param("~hfov_deg", 69.0)

        # App-layer extractor (ROS-free)
        self._extractor = DepthFeatureExtractor(
            roi_top_ratio=roi_top,
            roi_bottom_ratio=roi_bottom,
            obstacle_dist_m=obstacle_dist,
            min_gap_width_px=min_gap,
            hfov_deg=hfov,
        )

        self._bridge = CvBridge()

        # Publishers
        self._pub = rospy.Publisher(self._pub_topic, String, queue_size=5)
        if self._publish_debug:
            self._debug_pub = rospy.Publisher(self._debug_topic, Image, queue_size=2)

        # Subscriber
        rospy.Subscriber(
            self._depth_topic,
            Image,
            self._depth_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        rospy.loginfo(
            "[DepthFeatureNode] Listening on %s, publishing to %s",
            self._depth_topic,
            self._pub_topic,
        )

    # ------------------------------------------------------------------

    def _depth_callback(self, msg: Image) -> None:
        try:
            depth_m = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[DepthFeatureNode] CvBridge error: %s", exc)
            return

        timestamp = msg.header.stamp.to_sec()
        features = self._extractor.process(depth_m, timestamp=timestamp)

        payload = json.dumps(
            {
                "range_array": features.range_array,
                "gap_index": features.gap_index,
                "gap_angle_rad": features.gap_angle_rad,
                "gap_width_px": features.gap_width_px,
                "confidence": features.confidence,
                "timestamp": features.timestamp,
            }
        )
        self._pub.publish(payload)

        if self._publish_debug and hasattr(self, "_debug_pub"):
            self._publish_debug_image(depth_m, features)

    def _publish_debug_image(self, depth_m, features) -> None:
        import cv2
        import numpy as np

        h, w = depth_m.shape
        roi_top = int(h * self._extractor._roi_top)
        roi_bottom = int(h * self._extractor._roi_bottom)

        vis = cv2.normalize(depth_m, None, 0, 255, cv2.NORM_MINMAX).astype(
            "uint8"
        )
        vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

        # Draw ROI band
        cv2.rectangle(vis, (0, roi_top), (w - 1, roi_bottom), (0, 255, 0), 1)

        # Draw selected gap column
        if 0 <= features.gap_index < w:
            cv2.line(
                vis,
                (features.gap_index, roi_top),
                (features.gap_index, roi_bottom),
                (0, 0, 255),
                2,
            )

        try:
            debug_msg = self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            self._debug_pub.publish(debug_msg)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[DepthFeatureNode] debug publish error: %s", exc)

    # ------------------------------------------------------------------

    def run(self) -> None:
        rospy.spin()


def main() -> None:
    node = DepthFeatureNode()
    node.run()


if __name__ == "__main__":
    main()
