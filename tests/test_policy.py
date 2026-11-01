import importlib.util
import json
from pathlib import Path
import unittest

from rb_runtime.policy import Policy

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(
    importlib.util.find_spec("onnxruntime") and importlib.util.find_spec("numpy"),
    "install the cpu extra to check the actual ONNX policy",
)
class OnnxReferenceTests(unittest.TestCase):
    def test_recurrent_actions_and_hidden_match_the_supplied_reference(self):
        rows = json.loads((ROOT / "tests/data/policy_reference.json").read_text())
        policy = Policy(ROOT, "onnx")
        try:
            for index, row in enumerate(rows):
                action = policy.step(row["obs"])
                error = max(abs(x - y) for x, y in zip(action, row["action"]))
                hidden_error = max(
                    abs(x - y) for x, y in zip(policy.hidden, row["hidden"])
                )
                self.assertLess(error, 1e-4, f"action at step {index}")
                self.assertLess(hidden_error, 1e-4, f"hidden at step {index}")
        finally:
            policy.close()
