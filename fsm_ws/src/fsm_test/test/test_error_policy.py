from pathlib import Path
import unittest

from fsm_core.error_code import ERROR_TABLE, ErrorCode, ErrorLevel
from fsm_core.recovery_policy import RecoveryAction, RecoveryPolicy


class TestErrorPolicy(unittest.TestCase):
    def test_every_error_code_has_meta(self):
        missing = [code for code in ErrorCode if int(code) not in ERROR_TABLE]
        self.assertFalse(missing)
        for key, meta in ERROR_TABLE.items():
            self.assertEqual(meta.code, key)

    def test_recovery_policy_yaml_override_wins(self):
        tmp_path = Path(self._get_tmp_dir())
        config = tmp_path / "error_codes.yaml"
        config.write_text(
            """
error_codes:
  overrides:
    5102:
      recovery: "SWITCH_TARGET"
      max_attempts: 1
      level: "INFO"
""",
            encoding="utf-8",
        )
        policy = RecoveryPolicy.from_yaml(config)
        decision = policy.decide(5102)
        self.assertEqual(decision.action, RecoveryAction.SWITCH_TARGET)
        self.assertEqual(decision.max_attempts, 1)
        self.assertEqual(decision.level, ErrorLevel.INFO)

    def _get_tmp_dir(self) -> str:
        import tempfile

        return tempfile.mkdtemp(prefix="fsm_test_")
