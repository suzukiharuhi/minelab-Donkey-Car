"""ROI helper utilities (ROS-free)."""
from __future__ import annotations

from typing import Tuple

import numpy as np


def crop_roi(
    image: np.ndarray,
    top: int,
    bottom: int,
    left: int = 0,
    right: int = -1,
) -> np.ndarray:
    """Return the sub-array ``image[top:bottom, left:right]``.

    Args:
        image:   2-D or 3-D numpy array (e.g. depth or colour frame).
        top:     First row (inclusive).
        bottom:  Last row (exclusive).
        left:    First column (inclusive).  Defaults to 0.
        right:   Last column (exclusive).  ``-1`` means full width.

    Returns:
        Sub-array view (no copy).
    """
    r = right if right != -1 else image.shape[1]
    return image[top:bottom, left:r]


def fractional_roi(
    image: np.ndarray,
    top_ratio: float = 0.3,
    bottom_ratio: float = 0.7,
    left_ratio: float = 0.0,
    right_ratio: float = 1.0,
) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
    """Crop *image* using fractional coordinates.

    Args:
        image:        Input 2-D or 3-D array.
        top_ratio:    Top of ROI as a fraction of image height.
        bottom_ratio: Bottom of ROI.
        left_ratio:   Left of ROI as a fraction of image width.
        right_ratio:  Right of ROI.

    Returns:
        ``(cropped_array, (top_px, bottom_px, left_px, right_px))``.
    """
    h, w = image.shape[:2]
    top = max(0, int(h * top_ratio))
    bottom = min(h, int(h * bottom_ratio))
    left = max(0, int(w * left_ratio))
    right = min(w, int(w * right_ratio))
    return image[top:bottom, left:right], (top, bottom, left, right)
