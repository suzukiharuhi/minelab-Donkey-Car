"""State-machine implementation (ROS-free).

Provides a minimal generic state machine used by the pilot node.  State
objects are created in the app layer; the pilot node wires them together
and calls ``process()`` once per control cycle.
"""
from __future__ import annotations

import logging
import traceback
from typing import Any, Dict, Optional

from minelab.app.states.base_state import BaseState
from minelab.app.states.state_type import StateType
from minelab.app.core.commands import ControlCommand

logger = logging.getLogger(__name__)


class StateMachine:
    """Simple state machine with enter/execute/check_transition/exit protocol.

    Args:
        state_map:           Mapping of ``StateType`` → ``BaseState`` instance.
        initial_state_type:  The state to activate on construction.
    """

    def __init__(
        self,
        state_map: Dict[StateType, BaseState],
        initial_state_type: StateType,
    ) -> None:
        if initial_state_type not in state_map:
            raise ValueError(
                f"Initial state {initial_state_type} not in state_map"
            )
        self._state_map: Dict[StateType, BaseState] = dict(state_map)
        self._current_type: StateType = initial_state_type
        self._current_state: BaseState = self._state_map[initial_state_type]

        try:
            self._current_state.enter()
        except Exception:
            logger.exception(
                "[StateMachine] enter() raised on init (state=%s)",
                initial_state_type,
            )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def current_type(self) -> StateType:
        return self._current_type

    @property
    def current_state(self) -> BaseState:
        return self._current_state

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def register_state(self, state_type: StateType, state: BaseState) -> None:
        """Dynamically register or replace a state at runtime."""
        self._state_map[state_type] = state
        logger.debug("[StateMachine] Registered state=%s", state_type)

    def transition_to(self, next_type: StateType, reason: str = "") -> None:
        """Perform a guarded state transition (exit → update → enter)."""
        if next_type not in self._state_map:
            logger.warning(
                "[StateMachine] Unknown next state %s – transition ignored", next_type
            )
            return
        if next_type == self._current_type:
            return

        old_state = self._current_state
        new_state = self._state_map[next_type]

        logger.info(
            "[StateMachine] Transition %s → %s  reason=%s",
            self._current_type.name,
            next_type.name,
            reason,
        )

        try:
            old_state.exit()
        except Exception:
            logger.exception(
                "[StateMachine] exit() raised (state=%s)", self._current_type
            )

        self._current_type = next_type
        self._current_state = new_state

        try:
            new_state.enter()
        except Exception:
            logger.exception(
                "[StateMachine] enter() raised (state=%s)", next_type
            )

    def process(
        self,
        imu_data: Any,
        depth_features: Any,
        marker_info: Any,
    ) -> Optional[ControlCommand]:
        """Run one control cycle.

        1. ``execute()`` → generate ``ControlCommand``
        2. ``check_transition()`` → optionally change state
        3. Return the command.

        Args:
            imu_data:       Latest ``SensorSnapshot`` or ``IMUData`` object.
            depth_features: Latest ``DepthFeatures`` object (may be ``None``).
            marker_info:    Latest ``MarkerInfo`` object (may be ``None``).

        Returns:
            ``ControlCommand`` or ``None`` on error.
        """
        command: Optional[ControlCommand] = None

        try:
            command = self._current_state.execute(imu_data, depth_features, marker_info)
        except Exception:
            logger.exception(
                "[StateMachine] execute() raised (state=%s)", self._current_type
            )
            return None

        next_type: Optional[StateType] = None
        try:
            next_type = self._current_state.check_transition(
                imu_data, depth_features, marker_info
            )
        except Exception:
            logger.exception(
                "[StateMachine] check_transition() raised (state=%s)", self._current_type
            )

        if next_type is not None:
            self.transition_to(next_type)

        return command
