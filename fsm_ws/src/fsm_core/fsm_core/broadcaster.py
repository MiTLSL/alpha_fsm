from __future__ import annotations

import json
import time
from dataclasses import asdict, is_dataclass
from typing import Any


class Broadcaster:
    """状态广播器。

    M0 阶段允许没有 ROS publisher；此时只记录最后一帧和历史，便于单测。
    接 ROS2 节点时可通过 register_publisher 注入实际 publisher 与消息构造器。
    """

    def __init__(self, node_name: str = "") -> None:
        self.node_name = node_name
        self.snapshots: list[dict[str, Any]] = []
        self.last_snapshot: dict[str, Any] | None = None
        self._publishers: dict[str, tuple[Any, Any]] = {}

    def register_publisher(self, fsm_name: str, publisher: Any, msg_factory: Any) -> None:
        self._publishers[fsm_name] = (publisher, msg_factory)

    def publish(self, fsm_name: str, current_state: str, ctx, *, parent_fsm: str = "", parent_state: str = "") -> None:
        snapshot = {
            "ts": time.time(),
            "node_name": self.node_name or getattr(ctx, "node_name", ""),
            "fsm_name": fsm_name,
            "current_state": current_state,
            "parent_fsm": parent_fsm,
            "parent_state": parent_state,
            "task_id": getattr(ctx, "task_id", ""),
            "wall_index": getattr(ctx, "wall_index", 0),
            "phase": getattr(ctx, "phase", 0),
            "retry_count": getattr(ctx, "retry_count", 0),
            "last_error_code": self._last_error_code(ctx),
            "extra_json": json.dumps(self._extra(ctx), ensure_ascii=False, default=str),
        }
        self.last_snapshot = snapshot
        self.snapshots.append(snapshot)

        publisher_pair = self._publishers.get(fsm_name)
        if publisher_pair is None:
            return
        publisher, msg_factory = publisher_pair
        publisher.publish(msg_factory(snapshot))

    def _last_error_code(self, ctx) -> int:
        last_error = getattr(ctx, "last_error", None)
        return int(getattr(last_error, "error_code", 0) or 0)

    def _extra(self, ctx) -> dict[str, Any]:
        if ctx is None:
            return {}
        if is_dataclass(ctx):
            data = asdict(ctx)
            for key in ("config", "error_history"):
                data.pop(key, None)
            return data
        return {}


class NullBroadcaster(Broadcaster):
    pass
