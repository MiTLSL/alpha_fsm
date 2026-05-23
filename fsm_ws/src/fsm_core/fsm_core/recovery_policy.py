from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .error_code import ERROR_TABLE, ErrorLevel, RecoveryAction, get_error_meta


DEFAULT_MAX_ATTEMPTS: dict[RecoveryAction, int] = {
    RecoveryAction.RETRY_CURRENT_STATE: 3,
    RecoveryAction.REPLAN: 2,
    RecoveryAction.REPERCEPTION: 3,
    RecoveryAction.SWITCH_TARGET: 5,
    RecoveryAction.SWITCH_PHASE: 1,
    RecoveryAction.MOVE_BASE: 2,
    RecoveryAction.RETREAT_SAFE: 1,
    RecoveryAction.RELOCALIZE: 2,
    RecoveryAction.REBUILD_GRID: 2,
    RecoveryAction.WAIT_MANUAL_RECOVERY: 1,
    RecoveryAction.ABORT_TASK: 1,
    RecoveryAction.E_STOP: 1,
    RecoveryAction.NONE: 1,
}


@dataclass(frozen=True)
class RecoveryDecision:
    error_code: int
    level: ErrorLevel
    action: RecoveryAction
    max_attempts: int
    silenced: bool = False


class RecoveryPolicy:
    def __init__(
        self,
        overrides: dict[int, dict[str, Any]] | None = None,
        silenced: set[int] | None = None,
        max_attempts: dict[RecoveryAction, int] | None = None,
    ) -> None:
        self.overrides = overrides or {}
        self.silenced = silenced or set()
        self.max_attempts = {**DEFAULT_MAX_ATTEMPTS, **(max_attempts or {})}

    @classmethod
    def from_yaml(cls, path: str | Path) -> "RecoveryPolicy":
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "RecoveryPolicy":
        root = data.get("error_codes", data)
        overrides_raw = root.get("overrides", {}) or {}
        overrides = {int(code): value for code, value in overrides_raw.items()}
        silenced = {int(code) for code in (root.get("silenced", []) or [])}

        max_attempts_raw = root.get("recovery_max_attempts", {}) or root.get("max_attempts", {}) or {}
        max_attempts: dict[RecoveryAction, int] = {}
        for name, attempts in max_attempts_raw.items():
            max_attempts[RecoveryAction[name]] = int(attempts)
        return cls(overrides=overrides, silenced=silenced, max_attempts=max_attempts)

    def decide(self, error_code: int, state_custom_recovery: dict[int, RecoveryAction] | None = None) -> RecoveryDecision:
        meta = get_error_meta(error_code)
        override = self.overrides.get(int(error_code), {})

        level = ErrorLevel[override["level"]] if "level" in override else meta.level
        if int(error_code) in self.silenced:
            level = ErrorLevel.INFO

        if state_custom_recovery and int(error_code) in state_custom_recovery:
            action = state_custom_recovery[int(error_code)]
        elif "recovery" in override:
            action = RecoveryAction[override["recovery"]]
        else:
            action = meta.default_recovery

        attempts = int(override.get("max_attempts", self.max_attempts.get(action, 1)))
        return RecoveryDecision(
            error_code=int(error_code),
            level=level,
            action=action,
            max_attempts=attempts,
            silenced=int(error_code) in self.silenced,
        )

    def validate_known_codes(self) -> None:
        unknown = [code for code in self.overrides if code not in ERROR_TABLE]
        if unknown:
            raise ValueError(f"unknown error code in overrides: {unknown}")
