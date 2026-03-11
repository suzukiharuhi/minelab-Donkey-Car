"""Image logger (ROS-free).

Saves numpy arrays as image files to the session directory.
"""
from __future__ import annotations

import logging
import os
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class ImageLogger:
    """Save debug images to disk.

    Args:
        directory:  Target directory for saved images.
        prefix:     Filename prefix.
    """

    def __init__(self, directory: str, prefix: str = "frame") -> None:
        self._dir = Path(directory)
        self._dir.mkdir(parents=True, exist_ok=True)
        self._prefix = prefix
        self._counter: int = 0

    def save(
        self,
        image: np.ndarray,
        tag: str = "",
        ext: str = "jpg",
    ) -> Optional[str]:
        """Save *image* and return the file path, or ``None`` on error."""
        if image is None:
            return None
        ts = int(time.time() * 1000)
        name = f"{self._prefix}_{ts:013d}_{tag}.{ext}"
        path = str(self._dir / name)
        try:
            cv2.imwrite(path, image)
            self._counter += 1
        except Exception as exc:
            logger.warning("[ImageLogger] Failed to save %s: %s", path, exc)
            return None
        return path
