from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any

from .transition import StateResult


class StateLogger:
    """FSM 事件日志器。

    默认只把事件保存在内存中，便于 L0 单测；配置 file_path 后同时写 JSON Lines。
    """

    def __init__(self, node_name: str = "", file_path: str | os.PathLike[str] | None = None) -> None:
        self.node_name = node_name
        self.events: list[dict[str, Any]] = []
        self._file = None
        if file_path:
            path = Path(file_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._file = path.open("a", encoding="utf-8")

    def on_enter(self, fsm: str, state: str, ctx) -> None:
        self.emit("state_enter", fsm=fsm, state=state, ctx=ctx)

    def on_exit(self, fsm: str, state: str, ctx, result: StateResult, duration_ms: float) -> None:
        self.emit(
            "state_exit",
            fsm=fsm,
            state=state,
            ctx=ctx,
            transition=result.transition.value,
            duration_ms=duration_ms,
            error_code=result.error_code,
        )

    def on_transition(
        self,
        fsm: str,
        from_state: str,
        to_state: str | None,
        result: StateResult,
        ctx,
        duration_ms: float,
    ) -> None:
        self.emit(
            "transition",
            fsm=fsm,
            state=from_state,
            from_state=from_state,
            to_state=to_state,
            transition=result.transition.value,
            duration_ms=duration_ms,
            error_code=result.error_code,
            ctx=ctx,
        )

    def on_error(self, fsm: str, state: str, error_code: int, ctx) -> None:
        self.emit("error", fsm=fsm, state=state, error_code=error_code, ctx=ctx)

    def metric(self, fsm: str, state: str, name: str, value: Any, ctx=None) -> None:
        self.emit("metric", fsm=fsm, state=state, extra={name: value}, ctx=ctx)

    def emit(self, event: str, **kwargs: Any) -> dict[str, Any]:
        ctx = kwargs.pop("ctx", None)
        record = {
            "ts": time.time(),
            "node": self.node_name or getattr(ctx, "node_name", ""),
            "fsm": kwargs.pop("fsm", ""),
            "task_id": getattr(ctx, "task_id", ""),
            "event": event,
            "state": kwargs.pop("state", ""),
            "from_state": kwargs.pop("from_state", ""),
            "to_state": kwargs.pop("to_state", None),
            "transition": kwargs.pop("transition", ""),
            "duration_ms": kwargs.pop("duration_ms", None),
            "error_code": kwargs.pop("error_code", None),
            "retry_count": getattr(ctx, "retry_count", 0),
            "ctx_snapshot": self._ctx_snapshot(ctx),
            "extra": kwargs.pop("extra", {}),
        }
        record.update(kwargs)
        self.events.append(record)
        if self._file:
            self._file.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
            self._file.flush()
        return record

    def close(self) -> None:
        if self._file:
            self._file.close()
            self._file = None

    def _ctx_snapshot(self, ctx) -> dict[str, Any]:
        if ctx is None:
            return {}
        keys = ("task_id", "wall_index", "phase", "current_state", "retry_count")
        snapshot = {key: getattr(ctx, key) for key in keys if hasattr(ctx, key)}
        last_error = getattr(ctx, "last_error", None)
        if last_error is not None:
            snapshot["last_error"] = asdict(last_error) if is_dataclass(last_error) else str(last_error)
        return snapshot


class NullStateLogger(StateLogger):
    def __init__(self) -> None:
        self.node_name = ""
        self.events = []
        self._file = None

    def emit(self, event: str, **kwargs: Any) -> dict[str, Any]:
        record = {"event": event, **kwargs}
        self.events.append(record)
        return record
