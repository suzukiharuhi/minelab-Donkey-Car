"""Safety guard – watchdog that issues STOP when sensor data times out (ROS-free).

The ``SafetyGuard`` is called once per pilot loop cycle.  If any
required data stream has not been updated within its configured
timeout, a STOP command is returned to prevent dangerous motion.
"""
from __future__ import annotations

import logging
import time
from typing import Optional

from minelab.app.core.commands import ControlCommand
from minelab.interfaces.data_models import DepthFeatures, MarkerInfo

logger = logging.getLogger(__name__)


class SafetyGuard:
    """Timeout-based safety guard for sensor data streams.

    Args:
        depth_timeout_s:   Maximum age of ``DepthFeatures`` before STOP [s].
                           Use ``None`` to disable.
        marker_timeout_s:  Maximum age of ``MarkerInfo`` before STOP [s].
                           Use ``None`` to disable.
    """

    def __init__(
        self,
        depth_timeout_s: Optional[float] = 1.0,
        marker_timeout_s: Optional[float] = None,
    ) -> None:
        self._depth_timeout = depth_timeout_s
        self._marker_timeout = marker_timeout_s

    def check(
        self,
        depth_features: Optional[DepthFeatures],
        marker_info: Optional[MarkerInfo],
        now: Optional[float] = None,
    ) -> Optional[ControlCommand]:
        """Return ``ControlCommand.stop()`` if any required stream has timed out.

        Args:
            depth_features: Latest ``DepthFeatures`` (``None`` = never received).
            marker_info:    Latest ``MarkerInfo`` (``None`` = never received).
            now:            Current time in seconds.  Defaults to
                            ``time.monotonic()`` if not supplied.

        Returns:
            ``ControlCommand.stop()`` on timeout, ``None`` when all streams
            are fresh.
        """
        t = now if now is not None else time.monotonic()

        if self._depth_timeout is not None:
            if depth_features is None:
                logger.warning("[SafetyGuard] DepthFeatures never received → STOP")
                return ControlCommand.stop()
            age = t - depth_features.timestamp
            if age > self._depth_timeout:
                logger.warning(
                    "[SafetyGuard] DepthFeatures stale (age=%.2fs > %.2fs) → STOP",
                    age,
                    self._depth_timeout,
                )
                return ControlCommand.stop()

        if self._marker_timeout is not None:
            if marker_info is None:
                logger.warning("[SafetyGuard] MarkerInfo never received → STOP")
                return ControlCommand.stop()
            age = t - marker_info.timestamp
            if age > self._marker_timeout:
                logger.warning(
                    "[SafetyGuard] MarkerInfo stale (age=%.2fs > %.2fs) → STOP",
                    age,
                    self._marker_timeout,
                )
                return ControlCommand.stop()

        return None
