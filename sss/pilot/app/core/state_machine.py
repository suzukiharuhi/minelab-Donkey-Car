from __future__ import annotations

# import logging
import rospy
import traceback
from typing import Dict, Optional, Any

# 既存の状態クラス・列挙に合わせてインポート
# README は state_types.py ですが、実体は state_type.py のためこちらに合わせます
from pilot.app.states.base_state import BaseState
from pilot.app.states.state_type import StateType  # e.g. IDLE, APPROACH_MARKER, TURNING, EMERGENCY_STOP
# from pilot.app.core.step_logger import log_common_step
from pilot.app.logging import SessionLogger

# logging.basicConfig(level=logging.DEBUG)  # ここで表示レベルを DEBUG に設定
# logger = logging.getLogger(__name__)


class StateMachine:
    """
    シンプルなステートマシン:
      - 現在の状態 current_state を保持
      - execute() で ControlCommand を生成
      - check_transition() で遷移先を判断し、必要に応じて遷移
    """

    def __init__(self, state_map: Dict[StateType, BaseState], initial_state_type: StateType) -> None:
        if initial_state_type not in state_map:
            raise ValueError(f"Initial state {initial_state_type} not in state_map")
        self._state_map: Dict[StateType, BaseState] = dict(state_map)
        self._current_type: StateType = initial_state_type
        self._current_state: BaseState = self._state_map[self._current_type]
        self._logger = SessionLogger.get_instance()

        # 初期状態の入場フック（enter/on_enter）
        try:
            if hasattr(self._current_state, "enter"):
                self._current_state.enter()
        except Exception as e:
            rospy.logerr("[StateMachine] enter() failed on init:\n%s", traceback.format_exc())
            # logger.exception("enter failed: %s", e)

    @property
    def current_type(self) -> StateType:
        return self._current_type

    @property
    def current_state(self) -> BaseState:
        return self._current_state

    def register_state(self, state_type: StateType, state: BaseState) -> None:
        self._state_map[state_type] = state
        rospy.logdebug("[StateMachine] Registered: state=%s", state_type)

    def transition_to(self, next_type: StateType, reason: str = "") -> None:
        """
        状態遷移処理
        - 現状態の exit() 呼び出し
        - 新状態の enter() 呼び出し
        - current_state 更新
        """
        if next_type not in self._state_map:
            rospy.logwarn("[StateMachine] Unknown next state: %s (ignored)", next_type)
            return
        if next_type == self._current_type:
            return

        old_state = self._current_state
        new_state = self._state_map[next_type]

        # 状態遷移ロギング
        self._logger.log_state_transition(from_state=self._current_type.name, to_state=next_type.name)

        rospy.loginfo("[StateMachine] Transition: %s -> %s", self._current_type, next_type)

        # 現状態の終了処理(exitがあれば)
        if hasattr(old_state, "exit"):
            try:
                old_state.exit()
            except Exception as e:
                rospy.logerr("[StateMachine] exit failed: %s", e)

        # 状態更新
        self._current_type = next_type
        self._current_state = new_state

        # 新状態の開始処理(enterがあれば)
        if hasattr(new_state, "enter"):
            try:
                new_state.enter()
            except Exception as e:
                rospy.logerr("[StateMachine] enter failed: %s", e)

    def process(self, imu_data: Any, camera_data: Any, markers: Any) -> Any:
        """
        1) 現状態の execute で ControlCommand を生成
        2) 現状態の check_transition で遷移判定
        3) 遷移があれば transition_to
        """
        try:
            command = self._current_state.execute(imu_data, camera_data, markers)
            print("command(statemachi_process):", command)
        except Exception as e:
            """ 
            現状態の execute で例外発生 
            """
            rospy.logerr(
                "[StateMachine] execute raised in state=%s\n%s",
                self._current_type,
                traceback.format_exc(),
            )
            return None

        next_type: Optional[StateType] = None
        try:
            next_type = self._current_state.check_transition(imu_data, camera_data, markers)
        except Exception as e:
            rospy.logerr("[StateMachine] check_transition failed in state=%s: %s", self._current_type, e)
            next_type = None

        if next_type is not None:
            self.transition_to(next_type)
        
        return command