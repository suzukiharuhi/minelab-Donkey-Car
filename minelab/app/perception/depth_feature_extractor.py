"""Depth-image feature extractor – ROI → 1-D projection → FTG-i gap selection (ROS-free).

This module contains no ROS imports.  It operates on raw ``numpy`` arrays
and returns ``DepthFeatures`` dataclass instances.

Pipeline
--------
1. ``extract_roi``        – crop the depth image to the relevant strip
2. ``project_to_1d``      – reduce each column to a single distance value
3. ``select_gap_ftgi``    – simplified FTG-i: find the widest safe gap
4. ``DepthFeatureExtractor.process`` – thin orchestration wrapper
"""
from __future__ import annotations

import logging
import math
import time
from typing import List, Optional, Tuple

import numpy as np

from minelab.interfaces.data_models import DepthFeatures

logger = logging.getLogger(__name__)

# Default ROI expressed as fractions of image height
_ROI_TOP_RATIO = 0.3
_ROI_BOTTOM_RATIO = 0.7

# Minimum gap width (in pixel columns) to be considered navigable
_MIN_GAP_WIDTH_PX = 20

# Distance threshold below which a column is treated as "blocked" [m]
_OBSTACLE_DIST_M = 0.8


# ---------------------------------------------------------------------------
# Low-level functions (unit-testable without a class)
# ---------------------------------------------------------------------------

def extract_roi(
    depth_image: np.ndarray,
    top_ratio: float = _ROI_TOP_RATIO,
    bottom_ratio: float = _ROI_BOTTOM_RATIO,
) -> np.ndarray:
    """Return the horizontal strip of *depth_image* defined by the row ratios.

    Args:
        depth_image:   2-D float32 depth frame [m].
        top_ratio:     Fractional row index of ROI top    (0 = image top).
        bottom_ratio:  Fractional row index of ROI bottom (1 = image bottom).

    Returns:
        Cropped 2-D array (sub-array view).
    """
    h = depth_image.shape[0]
    r0 = max(0, int(h * top_ratio))
    r1 = min(h, int(h * bottom_ratio))
    return depth_image[r0:r1, :]


def project_to_1d(
    roi: np.ndarray,
    min_valid_m: float = 0.1,
    max_valid_m: float = 5.0,
) -> np.ndarray:
    """Reduce an ROI to a 1-D range array by taking the column-wise median.

    Invalid (zero / out-of-range) pixels are masked out before the median
    so that sparse depth sensors do not bias towards zero.

    Args:
        roi:          2-D float32 depth array (ROI).
        min_valid_m:  Lower bound of valid depth [m].
        max_valid_m:  Upper bound of valid depth [m].

    Returns:
        1-D float32 array of length ``roi.shape[1]``.
        Columns with no valid pixels are set to 0.
    """
    valid_mask = (roi > min_valid_m) & (roi < max_valid_m)
    # Replace invalid with NaN, then take nanmedian per column
    roi_nan = np.where(valid_mask, roi.astype(np.float32), np.nan)
    with np.errstate(all="ignore"):
        range_1d = np.nanmedian(roi_nan, axis=0)
    range_1d = np.nan_to_num(range_1d, nan=0.0).astype(np.float32)
    return range_1d


def select_gap_ftgi(
    range_1d: np.ndarray,
    obstacle_dist_m: float = _OBSTACLE_DIST_M,
    min_gap_width_px: int = _MIN_GAP_WIDTH_PX,
    image_center_px: Optional[int] = None,
    hfov_rad: float = math.radians(69.0),
) -> Tuple[int, int, float, float]:
    """Simplified FTG-i gap selection.

    Identifies all contiguous sequences of columns that are either farther
    than *obstacle_dist_m* or have no valid depth reading (treated as open).
    Among those, the **widest** gap is selected; ties are broken by proximity
    to *image_center_px*.

    Args:
        range_1d:         1-D depth array (one value per pixel column).
        obstacle_dist_m:  Columns ≥ this value are "free".
        min_gap_width_px: Gaps narrower than this are discarded.
        image_center_px:  Reference pixel for angle calculation.
                          Defaults to the array centre.
        hfov_rad:         Horizontal field-of-view [rad] used to convert
                          gap centre pixel to a steering angle.

    Returns:
        ``(gap_index, gap_width_px, gap_angle_rad, confidence)`` where
        *gap_index* is the pixel column of the gap centre.
    """
    n = len(range_1d)
    if n == 0:
        return 0, 0, 0.0, 0.0

    center = image_center_px if image_center_px is not None else n // 2

    # Build free/blocked boolean array
    free = (range_1d == 0.0) | (range_1d >= obstacle_dist_m)

    # Find contiguous gap runs
    gaps: List[Tuple[int, int]] = []  # (start, end) inclusive
    in_gap = False
    start = 0
    for i, is_free in enumerate(free):
        if is_free and not in_gap:
            in_gap = True
            start = i
        elif not is_free and in_gap:
            in_gap = False
            gaps.append((start, i - 1))
    if in_gap:
        gaps.append((start, n - 1))

    # Filter by minimum width
    gaps = [(s, e) for s, e in gaps if (e - s + 1) >= min_gap_width_px]

    if not gaps:
        # No usable gap – fall back to image centre
        return center, 0, 0.0, 0.0

    # Select widest gap (ties: prefer nearest to centre)
    best = max(gaps, key=lambda g: (g[1] - g[0], -(abs((g[0] + g[1]) // 2 - center))))
    gap_centre_px = (best[0] + best[1]) // 2
    gap_width = best[1] - best[0] + 1

    # Convert pixel offset to angle
    px_per_rad = n / hfov_rad
    gap_angle_rad = (gap_centre_px - center) / px_per_rad

    # Confidence: fraction of gap columns with valid readings
    gap_slice = range_1d[best[0]: best[1] + 1]
    valid_count = int(np.sum(gap_slice > 0))
    confidence = valid_count / gap_width if gap_width > 0 else 0.0

    return gap_centre_px, gap_width, float(gap_angle_rad), float(confidence)


# ---------------------------------------------------------------------------
# Orchestration class
# ---------------------------------------------------------------------------

class DepthFeatureExtractor:
    """High-level wrapper that runs the full depth-to-features pipeline.

    Args:
        roi_top_ratio:     Fractional top of ROI within the depth frame.
        roi_bottom_ratio:  Fractional bottom of ROI.
        obstacle_dist_m:   Free-space threshold [m].
        min_gap_width_px:  Minimum gap width to consider [px].
        hfov_deg:          Horizontal field-of-view of the depth camera [°].
    """

    def __init__(
        self,
        roi_top_ratio: float = _ROI_TOP_RATIO,
        roi_bottom_ratio: float = _ROI_BOTTOM_RATIO,
        obstacle_dist_m: float = _OBSTACLE_DIST_M,
        min_gap_width_px: int = _MIN_GAP_WIDTH_PX,
        hfov_deg: float = 69.0,
    ) -> None:
        self._roi_top = roi_top_ratio
        self._roi_bottom = roi_bottom_ratio
        self._obstacle_dist = obstacle_dist_m
        self._min_gap_width = min_gap_width_px
        self._hfov_rad = math.radians(hfov_deg)

    def process(
        self,
        depth_image: np.ndarray,
        timestamp: Optional[float] = None,
    ) -> DepthFeatures:
        """Run the full pipeline on *depth_image*.

        Args:
            depth_image:  2-D float32 depth frame [m].
            timestamp:    Frame acquisition time [s].  Defaults to now.

        Returns:
            Populated ``DepthFeatures`` dataclass.
        """
        t = timestamp if timestamp is not None else time.monotonic()

        roi = extract_roi(depth_image, self._roi_top, self._roi_bottom)
        range_1d = project_to_1d(roi)

        gap_idx, gap_w, gap_angle, conf = select_gap_ftgi(
            range_1d,
            obstacle_dist_m=self._obstacle_dist,
            min_gap_width_px=self._min_gap_width,
            hfov_rad=self._hfov_rad,
        )

        return DepthFeatures(
            range_array=range_1d.tolist(),
            gap_index=gap_idx,
            gap_angle_rad=gap_angle,
            gap_width_px=gap_w,
            confidence=conf,
            timestamp=t,
        )
