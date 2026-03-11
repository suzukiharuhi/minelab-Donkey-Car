"""ROS-independent dataclass interfaces for the minelab UGV system.

All classes here are plain Python dataclasses with no ROS imports.
They represent the canonical data shapes exchanged between app-layer
modules and serialised over ROS topics (as JSON strings on
std_msgs/String) inside the nodes layer.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class SensorSnapshot:
    """Snapshot of the latest raw sensor readings.

    Holds the most recently received values from IMU and camera
    without any processing.  Timestamps are float seconds (UNIX epoch
    or ROS time – consistent within a session).
    """

    # IMU
    imu_yaw: float = 0.0                         # [rad]
    imu_roll: float = 0.0                        # [rad]
    imu_pitch: float = 0.0                       # [rad]
    imu_angular_velocity_z: float = 0.0          # [rad/s]
    imu_timestamp: float = 0.0

    # Depth camera
    depth_frame_timestamp: float = 0.0
    depth_frame_available: bool = False

    # RGB camera
    color_frame_timestamp: float = 0.0
    color_frame_available: bool = False

    # Battery (optional)
    battery_voltage: Optional[float] = None      # [V]
    battery_percentage: Optional[float] = None   # 0.0–1.0


@dataclass
class DepthFeatures:
    """Output of the depth-image feature-extraction pipeline.

    Produced by ``DepthFeatureExtractor`` and published by
    ``depth_feature_node``.  Contains the 1-D range projection and the
    FTG-i gap selection result.
    """

    # 1-D range array (one value per horizontal pixel column in the ROI)
    range_array: List[float] = field(default_factory=list)

    # Index into range_array for the selected gap centre
    gap_index: int = 0

    # Steering angle derived from gap_index (positive = right) [rad]
    gap_angle_rad: float = 0.0

    # Width of the selected gap in pixels
    gap_width_px: int = 0

    # Confidence in [0, 1] – ratio of valid (non-zero) cells in the gap
    confidence: float = 0.0

    timestamp: float = 0.0


@dataclass
class MarkerInfo:
    """Information about a single detected ArUco marker.

    Produced by ``ArucoDetector`` (app layer) and published by
    ``marker_detection_node``.
    """

    marker_id: int = -1
    visible: bool = False

    # Distance from camera centre to marker [cm]
    distance_z_cm: float = 0.0     # forward distance
    distance_x_cm: float = 0.0     # lateral offset (positive = right)

    # Orientation errors [deg]
    yaw_error_deg: float = 0.0     # marker yaw relative to camera
    pitch_deg: float = 0.0

    # Pixel position of marker centre in the colour image
    center_px: Tuple[int, int] = (0, 0)

    timestamp: float = 0.0

    def __str__(self) -> str:
        return (
            f"MarkerInfo(id={self.marker_id}, visible={self.visible}, "
            f"z={self.distance_z_cm:.1f}cm, x={self.distance_x_cm:.1f}cm, "
            f"yaw={self.yaw_error_deg:.1f}deg)"
        )


@dataclass
class PilotStatus:
    """Diagnostic snapshot published by the pilot node.

    Useful for the logger node and remote monitoring.
    """

    state_name: str = "IDLE"
    last_command_type: str = "NO_OPERATION"
    last_command_speed: str = "MID"

    # Heartbeat timestamps (float seconds)
    pilot_heartbeat: float = 0.0
    depth_features_last_received: float = 0.0
    marker_info_last_received: float = 0.0

    # Running session tag (set from ROS param ~session_tag)
    session_tag: str = ""
