from __future__ import annotations

import logging
from typing import Dict, Optional, Any

# 既存の状態クラス・列挙に合わせてインポート
# README は state_types.py ですが、実体は state_type.py のためこちらに合わせます
from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType  # e.g. IDLE, APPROACH_MARKER, TURNING, EMERGENCY_STOP

logging.basicConfig(level=logging.DEBUG)  # ここで表示レベルを DEBUG に設定
logger = logging.getLogger(__name__)


class StateMachine:
    """
    シンプルなステートマシン:
      - 現在の状態 current_state を保持
      - execute() で ControlCommand を生成
      - check_transition() で遷移先を判断し、必要に応じて遷移

    ROS 非依存にしてテスト・再利用しやすくするため logging を使用
    """

    def __init__(self, state_map: Dict[StateType, BaseState], initial_state_type: StateType) -> None:
        if initial_state_type not in state_map:
            raise ValueError(f"Initial state {initial_state_type} not in state_map")
        self._state_map: Dict[StateType, BaseState] = dict(state_map)
        self._current_type: StateType = initial_state_type
        self._current_state: BaseState = self._state_map[self._current_type]
        logger.debug("StateMachine initialized with state=%s", self._current_type)

        # 初期状態の入場フック（enter/on_enter）
        try:
            if hasattr(self._current_state, "enter"):
                self._current_state.enter()
        except Exception as e:
            logger.exception("enter failed: %s", e)

    @property
    def current_type(self) -> StateType:
        return self._current_type

    @property
    def current_state(self) -> BaseState:
        return self._current_state

    def register_state(self, state_type: StateType, state: BaseState) -> None:
        self._state_map[state_type] = state
        logger.debug("Registered state=%s", state_type)

    def transition_to(self, next_type: StateType) -> None:
        """
        状態遷移処理
        - 現状態の exit() 呼び出し
        - 新状態の enter() 呼び出し
        - current_state 更新
        """
        if next_type not in self._state_map:
            logger.warning("Unknown next state: %s (ignored)", next_type)
            return
        if next_type == self._current_type:
            return

        old_state = self._current_state
        new_state = self._state_map[next_type]

        logger.info("Transition: %s -> %s", self._current_type, next_type)

        # 現状態の終了処理(exitがあれば)
        if hasattr(old_state, "exit"):
            try:
                old_state.exit()
            except Exception as e:
                logger.exception("exit failed: %s", e)

        # 状態更新
        self._current_type = next_type
        self._current_state = new_state

        # 新状態の開始処理(enterがあれば)
        if hasattr(new_state, "enter"):
            try:
                new_state.enter()
            except Exception as e:
                logger.exception("enter failed: %s", e)

    def process(self, imu_data: Any, camera_data: Any, markers: Any) -> Any:
        """
        1) 現状態の execute で ControlCommand を生成
        2) 現状態の check_transition で遷移判定
        3) 遷移があれば transition_to
        """
        try:
            command = self._current_state.execute(imu_data, camera_data, markers)
        except Exception as e:
            """ 
            現状態の execute で例外発生 
            → ログ出力後、EMERGENCY_STOP 状態があれば遷移を試みる
            """
            logger.exception("execute failed in state=%s: %s", self._current_type, e)
            # 失敗時は緊急停止状態があれば遷移を試みる
            if StateType.EMERGENCY_STOP in self._state_map:
                self.transition_to(StateType.EMERGENCY_STOP)
                try:
                    return self._current_state.execute(imu_data, camera_data, markers)
                except Exception:
                    logger.exception("execute failed in EMERGENCY_STOP")
            return None

        try:
            next_type: Optional[StateType] = self._current_state.check_transition(
                imu_data, camera_data, markers
            )
        except Exception as e:
            logger.exception("check_transition failed in state=%s: %s", self._current_type, e)
            next_type = None

        if next_type is not None:
            self.transition_to(next_type)

        return command