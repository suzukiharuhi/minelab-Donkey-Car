"""Command mapper – converts ControlCommand to low-level string tokens (ROS-free).

Provides a thin translation layer between the app-level ``ControlCommand``
objects and the string-based protocol expected by ``motor_output.py``
(``"move_forward"``, ``"mid"``, etc.).
"""
from __future__ import annotations

from enum import Enum, auto
from typing import Tuple

from minelab.app.core.commands import ControlCommand


class CommandMapper:
    """Converts ``ControlCommand`` instances to ``(command_str, speed_str)`` tuples."""

    @staticmethod
    def to_strings(cmd: ControlCommand) -> Tuple[str, str]:
        """Return ``(command_str, speed_str)`` ready for ``MotorOutputController``.

        Examples::

            mapper = CommandMapper()
            cmd_str, spd_str = mapper.to_strings(ControlCommand.forward_left())
            # → ("forward_left", "mid")
        """
        return cmd.command.name.lower(), cmd.speed.name.lower()
