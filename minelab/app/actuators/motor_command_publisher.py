"""Motor command publisher stub (ROS-free base).

The ROS publisher implementation lives in the node layer.  This class
defines the interface so the app layer can reference it without importing
``rospy``.
"""
from __future__ import annotations

import logging
from abc import ABC, abstractmethod

from minelab.app.core.commands import ControlCommand

logger = logging.getLogger(__name__)


class BaseMotorCommandPublisher(ABC):
    """Abstract publisher interface for motor commands."""

    @abstractmethod
    def publish(self, cmd: ControlCommand) -> None:
        """Publish *cmd* to the appropriate downstream topic or system."""
