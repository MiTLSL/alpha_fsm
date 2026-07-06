from __future__ import annotations

import time
from collections.abc import Callable

from .base_state import BaseState
from .broadcaster import Broadcaster, NullBroadcaster
from .state_logger import NullStateLogger, StateLogger
from .transition import StateResult, TransitionType


class FSMEngine:
    """单线程 tick 引擎。一个 FSM 一个实例。"""

    def __init__(
        self,
        name: str,
        states: dict[str, BaseState],
        transitions: dict[tuple[str, TransitionType], str],
        initial_state: str,
        ctx,
        logger: StateLogger | None = None,
        broadcaster: Broadcaster | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        if initial_state not in states:
            raise ValueError(f"initial_state {initial_state!r} not in states")
        self.name = name
        self.states = states
        self.transitions = transitions
        self.ctx = ctx
        self.logger = logger or NullStateLogger()
        self.broadcaster = broadcaster or NullBroadcaster()
        self._clock = clock or time.monotonic
        self._initial_state = initial_state
        self._current: str | None = initial_state
        self._enter_time = 0.0
        self._entered = False
        self._abort_flag = False
        self._abort_reason = ""
        self._last_result: StateResult | None = None

    @property
    def current_state(self) -> str | None:
        return self._current

    def reset(self) -> None:
        self._current = self._initial_state
        self._enter_time = 0.0
        self._entered = False
        self._abort_flag = False
        self._abort_reason = ""
        self._last_result = None

    def last_result(self) -> StateResult | None:
        return self._last_result

    def request_abort(self, reason: str) -> None:
        self._abort_flag = True
        self._abort_reason = reason

    def tick(self) -> bool:
        if self._current is None:
            return False

        if self._abort_flag:
            self._handle_abort()
            return False

        state = self.states[self._current]
        self._ensure_entered(state)

        timeout_sec = state.timeout_sec
        if timeout_sec is not None and self._clock() - self._enter_time > timeout_sec:
            result = state.on_timeout(self.ctx)
            self._do_transition(state, result)
            return self._current is not None

        result = state.execute(self.ctx)
        if result.transition == TransitionType.STAY:
            return True

        self._do_transition(state, result)
        return self._current is not None

    def _ensure_entered(self, state: BaseState) -> None:
        if self._entered:
            return
        self._enter_time = self._clock()
        if hasattr(self.ctx, "current_fsm"):
            self.ctx.current_fsm = self.name
        if hasattr(self.ctx, "current_state"):
            self.ctx.current_state = state.name
        state.on_enter(self.ctx)
        self.logger.on_enter(self.name, state.name, self.ctx)
        self.broadcaster.publish(self.name, state.name, self.ctx)
        self._entered = True

    def _do_transition(self, state: BaseState, result: StateResult) -> None:
        duration_ms = (self._clock() - self._enter_time) * 1000.0
        state.on_exit(self.ctx, result)
        self.logger.on_exit(self.name, state.name, self.ctx, result, duration_ms)
        if result.error_code is not None:
            self.logger.on_error(self.name, state.name, int(result.error_code), self.ctx)

        next_state = self._resolve_next_state(state, result)
        self.logger.on_transition(self.name, state.name, next_state, result, self.ctx, duration_ms)
        self._last_result = result
        self._current = next_state
        self._entered = False

        if self._current is not None and self._current not in self.states:
            raise ValueError(f"transition target {self._current!r} not in states")

    def _resolve_next_state(self, state: BaseState, result: StateResult) -> str | None:
        if result.transition in (TransitionType.ABORT, TransitionType.TERMINATE):
            return None
        if result.next_state:
            return result.next_state
        return self.transitions.get((state.name, result.transition))

    def _handle_abort(self) -> None:
        if self._current is None:
            return
        state = self.states[self._current]
        if not self._entered:
            self._ensure_entered(state)
        state.on_abort(self.ctx, self._abort_reason)
        result = StateResult(TransitionType.ABORT, error_code=None, payload={"reason": self._abort_reason})
        self._do_transition(state, result)
