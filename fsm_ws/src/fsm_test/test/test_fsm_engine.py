import unittest

from fsm_core.base_state import BaseState
from fsm_core.fsm_engine import FSMEngine
from fsm_core.state_context import StateContext
from fsm_core.transition import StateResult, TransitionType


class StartState(BaseState):
    name = "START"
    timeout_sec = 10.0

    def __init__(self):
        super().__init__()
        self.entered = False
        self.exited = False

    def on_enter(self, ctx):
        self.entered = True

    def execute(self, ctx):
        return StateResult(TransitionType.SUCCESS)

    def on_exit(self, ctx, result):
        self.exited = True


class EndState(BaseState):
    name = "END"
    timeout_sec = None

    def execute(self, ctx):
        return StateResult(TransitionType.TERMINATE)


class TimeoutState(BaseState):
    name = "WAIT"
    timeout_sec = 1.0

    def execute(self, ctx):
        return StateResult.stay()


class AbortState(BaseState):
    name = "RUNNING"

    def __init__(self):
        super().__init__()
        self.abort_reason = ""

    def execute(self, ctx):
        return StateResult.stay()

    def on_abort(self, ctx, reason):
        self.abort_reason = reason


class TestFsmEngine(unittest.TestCase):
    def test_tick_success_transitions_and_hooks(self):
        start = StartState()
        end = EndState()
        ctx = StateContext()
        engine = FSMEngine(
            "DemoFSM",
            {"START": start, "END": end},
            {("START", TransitionType.SUCCESS): "END"},
            "START",
            ctx,
        )

        self.assertTrue(engine.tick())
        self.assertTrue(start.entered)
        self.assertTrue(start.exited)
        self.assertEqual(engine.current_state, "END")

    def test_timeout_transitions_to_failure_branch(self):
        now = [0.0]
        engine = FSMEngine(
            "TimeoutFSM",
            {"WAIT": TimeoutState(), "END": EndState()},
            {("WAIT", TransitionType.TIMEOUT): "END"},
            "WAIT",
            StateContext(),
            clock=lambda: now[0],
        )
        self.assertTrue(engine.tick())
        now[0] = 1.1
        self.assertTrue(engine.tick())
        self.assertEqual(engine.current_state, "END")

    def test_abort_calls_on_abort_and_stops(self):
        state = AbortState()
        engine = FSMEngine("AbortFSM", {"RUNNING": state}, {}, "RUNNING", StateContext())
        self.assertTrue(engine.tick())
        engine.request_abort("estop")
        self.assertFalse(engine.tick())
        self.assertEqual(state.abort_reason, "estop")
        self.assertIsNone(engine.current_state)
