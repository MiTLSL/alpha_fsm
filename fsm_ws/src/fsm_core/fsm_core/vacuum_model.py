from __future__ import annotations


class VacuumPressureModel:
    """一阶真空压力模型，用于 mock 与 mock-first 生产骨架。"""

    def __init__(self, clock, buildup_time_ms: float = 150.0, release_time_ms: float = 100.0) -> None:
        self._clock = clock
        self._enabled = False
        self._pressure = 0.0
        self._last_time_sec = self._now_sec()
        self.buildup_time_ms = float(buildup_time_ms)
        self.release_time_ms = float(release_time_ms)

    def set_enabled(self, enabled: bool) -> None:
        self._update_pressure()
        self._enabled = bool(enabled)

    def sample(self, failure: str = "NONE") -> float:
        self._update_pressure()
        if failure == "NEVER_BUILDUP" and self._enabled:
            return -10.0
        if failure == "LEAK_AFTER_ATTACH" and self._enabled and self._pressure < -50.0:
            return -20.0
        return self._pressure

    def _update_pressure(self) -> None:
        import math

        now = self._now_sec()
        dt = max(now - self._last_time_sec, 0.0)
        self._last_time_sec = now

        target = -60.0 if self._enabled else 0.0
        tau_ms = self.buildup_time_ms if self._enabled else self.release_time_ms
        tau = max(float(tau_ms) / 3000.0, 0.001)
        alpha = 1.0 - math.exp(-dt / tau)
        self._pressure = self._pressure + (target - self._pressure) * alpha

    def _now_sec(self) -> float:
        return self._clock.now().nanoseconds / 1_000_000_000.0
