#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "fsm_ws" / "src"
for package in ("fsm_core", "wall_destacking_strategy"):
    sys.path.insert(0, str(SRC / package))

from fsm_core.base_state import BaseState
from fsm_core.error_code import ERROR_TABLE, ErrorCode
from fsm_core.fsm_engine import FSMEngine
from fsm_core.recovery_policy import RecoveryAction, RecoveryPolicy
from fsm_core.state_context import StateContext
from fsm_core.transition import StateResult, TransitionType
from wall_destacking_strategy.data import GraspPair, PoseData, Vector3Data, WallGrid


class _Start(BaseState):
    name = "START"

    def execute(self, ctx):
        return StateResult(TransitionType.SUCCESS)


class _End(BaseState):
    name = "END"

    def execute(self, ctx):
        return StateResult(TransitionType.TERMINATE)


def check_fsm_engine() -> None:
    engine = FSMEngine(
        "SelfCheckFSM",
        {"START": _Start(), "END": _End()},
        {("START", TransitionType.SUCCESS): "END"},
        "START",
        StateContext(),
    )
    assert engine.tick() is True
    assert engine.current_state == "END"


def check_errors() -> None:
    missing = [code for code in ErrorCode if int(code) not in ERROR_TABLE]
    assert not missing, f"missing error meta: {missing}"
    for key, meta in ERROR_TABLE.items():
        assert key == meta.code

    policy = RecoveryPolicy.from_dict({"error_codes": {"overrides": {5102: {"recovery": "SWITCH_TARGET", "max_attempts": 1}}}})
    decision = policy.decide(5102)
    assert decision.action == RecoveryAction.SWITCH_TARGET
    assert decision.max_attempts == 1


def check_data() -> None:
    grid = WallGrid.empty("task001", 0)
    assert len(grid.slots) == 25
    assert grid.slot_at(4, 4).row_major_index() == 24
    pair = GraspPair(
        pair_id="task001_w0_lp_p0001",
        task_id="task001",
        wall_index=0,
        phase=0,
        left_slot_id="wall_0_row_0_col_0",
        left_box_pose_robot=PoseData.zero(),
        left_box_size=Vector3Data(0.4, 0.4, 0.4),
        grasp_mode=1,
    )
    assert pair.right_slot_id == ""


def main() -> int:
    check_fsm_engine()
    check_errors()
    check_data()
    print("M0 self-check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
