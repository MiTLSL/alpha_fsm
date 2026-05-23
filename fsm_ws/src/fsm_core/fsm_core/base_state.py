from __future__ import annotations

from abc import ABC, abstractmethod
from typing import ClassVar

from .error_code import ErrorCode
from .transition import StateResult, TransitionType


class BaseState(ABC):
    """所有状态的抽象基类。

    具体状态的 execute 必须快速返回；长耗时动作通过 Action future 在多次 tick 中推进。
    """

    name: ClassVar[str] = ""
    fsm_layer: ClassVar[str] = ""
    timeout_sec: ClassVar[float | None] = 10.0
    max_retry: ClassVar[int] = 0
    custom_recovery: ClassVar[dict[int, object]] = {}

    def __init__(self) -> None:
        if not self.name:
            self.name = self.__class__.__name__.upper()

    def on_enter(self, ctx) -> None:
        pass

    @abstractmethod
    def execute(self, ctx) -> StateResult:
        ...

    def on_exit(self, ctx, result: StateResult) -> None:
        pass

    def on_timeout(self, ctx) -> StateResult:
        return StateResult(
            transition=TransitionType.TIMEOUT,
            error_code=int(ErrorCode.E_STATE_TIMEOUT_GENERIC),
        )

    def on_abort(self, ctx, reason: str) -> None:
        pass
