"""ArUco detector wrapper (ROS-free).

Wraps OpenCV's ``cv2.aruco`` detection and pose-estimation routines.
No ``rospy`` imports.  Uses the standard ``logging`` module.
"""
from __future__ import annotations

import logging
import os
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from cv2 import aruco

from minelab.interfaces.data_models import MarkerInfo

logger = logging.getLogger(__name__)

# Default marker dictionary
_DEFAULT_DICT = aruco.DICT_4X4_50

# Default camera-intrinsic search paths (relative to this file)
_MTX_PATH = os.path.join(os.path.dirname(__file__), "mtx.npy")
_DIST_PATH = os.path.join(os.path.dirname(__file__), "dist.npy")


class ArucoDetector:
    """Detect ArUco markers and estimate their pose.

    Args:
        dictionary:     ArUco dictionary constant (``aruco.DICT_*``).
        camera_matrix:  3×3 intrinsic matrix.  Loaded from ``mtx.npy``
                        if ``None``.
        dist_coeffs:    Distortion coefficients.  Loaded from ``dist.npy``
                        if ``None``.
        default_marker_size_m:  Marker side length [m] when not in *id_size_map*.
        id_size_map:    Per-ID marker sizes [m].
        offset_x_cm:    Horizontal offset between camera and vehicle centre [cm].
        smooth_alpha:   EMA smoothing factor for pose (0 = no update, 1 = raw).
    """

    def __init__(
        self,
        dictionary: int = _DEFAULT_DICT,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
        default_marker_size_m: float = 0.07,
        id_size_map: Optional[Dict[int, float]] = None,
        offset_x_cm: float = 3.0,
        smooth_alpha: float = 0.2,
    ) -> None:
        self._aruco_dict = aruco.getPredefinedDictionary(dictionary)
        self._params = aruco.DetectorParameters()
        self._default_size = default_marker_size_m
        self._id_size_map: Dict[int, float] = id_size_map or {
            0: 0.07, 3: 0.07, 4: 0.07,
            20: 0.15, 21: 0.15, 10: 0.15,
        }
        self._offset_x_cm = offset_x_cm
        self._smooth_alpha = smooth_alpha
        self._pose_ema: Dict[int, Dict[str, np.ndarray]] = {}

        # Sub-pixel corner refinement
        try:
            self._params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self._params.cornerRefinementWinSize = 5
            self._params.cornerRefinementMaxIterations = 30
            self._params.cornerRefinementMinAccuracy = 0.01
        except Exception:
            pass

        # Camera intrinsics
        self._camera_matrix = camera_matrix
        self._dist_coeffs = dist_coeffs
        if self._camera_matrix is None:
            self._load_camera_params()

        logger.info(
            "ArucoDetector initialised (dict=%s, default_size=%.3fm)",
            dictionary,
            default_marker_size_m,
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _load_camera_params(self) -> None:
        try:
            self._camera_matrix = np.load(_MTX_PATH)
            self._dist_coeffs = np.load(_DIST_PATH)
            logger.info("Camera parameters loaded from %s / %s", _MTX_PATH, _DIST_PATH)
        except Exception as exc:
            logger.warning("Could not load camera parameters: %s", exc)
            self._camera_matrix = None
            self._dist_coeffs = None

    @staticmethod
    def _rvec_to_euler(rvec: np.ndarray) -> Tuple[float, float, float]:
        """Convert Rodrigues rvec → (yaw, pitch, roll) in degrees."""
        R, _ = cv2.Rodrigues(rvec)
        yaw = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))
        pitch = float(np.degrees(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))))
        roll = float(np.degrees(np.arctan2(R[2, 1], R[2, 2])))
        return yaw, pitch, roll

    def _size_for_id(self, marker_id: int) -> float:
        return self._id_size_map.get(marker_id, self._default_size)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def detect(
        self,
        color_image: np.ndarray,
        estimate_pose: bool = True,
        timestamp: Optional[float] = None,
    ) -> List[MarkerInfo]:
        """Detect markers in *color_image* and return ``MarkerInfo`` objects.

        Args:
            color_image:    BGR uint8 image from the colour camera.
            estimate_pose:  If ``True`` and camera intrinsics are available,
                            compute distance/orientation.
            timestamp:      Frame acquisition time [s].  Defaults to now.

        Returns:
            List of ``MarkerInfo`` (one per detected marker).
        """
        t = timestamp if timestamp is not None else time.monotonic()

        if color_image is None:
            return []

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._params
        )

        results: List[MarkerInfo] = []
        if ids is None:
            return results

        for i, marker_id in enumerate(ids.flatten()):
            corner = corners[i].reshape(-1, 2)
            center_xy = tuple(corner.mean(axis=0).astype(int))

            info = MarkerInfo(
                marker_id=int(marker_id),
                visible=True,
                center_px=(int(center_xy[0]), int(center_xy[1])),
                timestamp=t,
            )

            if estimate_pose and self._camera_matrix is not None:
                size_m = self._size_for_id(info.marker_id)
                try:
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        np.array([corner], dtype=np.float32),
                        size_m,
                        self._camera_matrix,
                        self._dist_coeffs,
                    )
                    r = np.squeeze(rvecs[0])
                    t_vec = np.squeeze(tvecs[0])

                    # EMA smoothing
                    prev = self._pose_ema.get(info.marker_id)
                    if prev is not None:
                        a = self._smooth_alpha
                        r = (1 - a) * prev["r"] + a * r
                        t_vec = (1 - a) * prev["t"] + a * t_vec
                    self._pose_ema[info.marker_id] = {"r": r, "t": t_vec}

                    x_cm = float(t_vec[0] * 100.0) - self._offset_x_cm
                    z_cm = float(t_vec[2] * 100.0)
                    info.distance_x_cm = x_cm
                    info.distance_z_cm = z_cm

                    yaw, pitch, _ = self._rvec_to_euler(r)
                    info.yaw_error_deg = yaw
                    info.pitch_deg = pitch

                except Exception as exc:
                    logger.warning("Pose estimation failed (id=%d): %s", info.marker_id, exc)

            results.append(info)

        return results
