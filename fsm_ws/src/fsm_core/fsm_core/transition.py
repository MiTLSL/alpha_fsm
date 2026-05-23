from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class TransitionType(Enum):
    STAY = "stay"
    SUCCESS = "success"
    FAILURE = "failure"
    TIMEOUT = "timeout"
    ABORT = "abort"
    TERMINATE = "terminate"


@dataclass(frozen=True)
class StateResult:
    transition: TransitionType
    next_state: str | None = None
    error_code: int | None = None
    payload: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def stay(cls) -> "StateResult":
        return cls(TransitionType.STAY)

    @classmethod
    def success(cls, next_state: str | None = None, **payload: Any) -> "StateResult":
        return cls(TransitionType.SUCCESS, next_state=next_state, payload=payload)

    @classmethod
    def failure(cls, error_code: int | None = None, next_state: str | None = None, **payload: Any) -> "StateResult":
        return cls(TransitionType.FAILURE, next_state=next_state, error_code=error_code, payload=payload)
