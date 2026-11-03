from pathlib import Path
from types import SimpleNamespace
import unittest

from rb_runtime.recovery import StandupProcess, fault_message

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "build/host/bin/standup_helper"


@unittest.skipUnless(
    HELPER.is_file(), "build the host helper with scripts/build.sh host"
)
class RecoveryProcessTests(unittest.TestCase):
    def test_reset_preserves_prepared_gains_and_restarts_the_same_sequence(self):
        sample = SimpleNamespace(
            gravity=[0.0, 0.0, -1.0],
            gyro=[0.0] * 3,
            accel=[0.0, 0.0, 9.81],
            q=[2.35706715, 2.35706715, 0.0, 0.0, 0.0, 0.0],
            qd=[0.0] * 6,
            hip_torque=[0.0, 0.0],
            flags=7,
        )
        process = StandupProcess(HELPER)
        try:
            reference = None
            for _ in range(3):
                process.reset()
                replies = [
                    process.step(3.0 + tick * 0.005, sample) for tick in range(25)
                ]
                if reference is None:
                    reference = replies
                self.assertEqual(replies, reference)
                self.assertTrue(all(len(r[3]) == 6 and len(r[4]) == 4 for r in replies))
                self.assertEqual(replies[0][1], 1)  # Folding
                self.assertAlmostEqual(replies[0][3][2], replies[0][3][0])
        finally:
            process.close()


class FaultMessageTests(unittest.TestCase):
    def test_failure_code_is_named_and_inputs_are_kept(self):
        values = [12.5, 0.1, 0.0, 0.0, 0.0, 9.8, 2.1, 2.2, 0.3, 0.4, 0, 0, -1.5, 1.5, 0, 0, 0, 0]
        message = fault_message(5.0, values)
        self.assertIn("FloorConstraint (code 5)", message)
        self.assertIn("t=12.5 pitch=0.1", message)
        self.assertIn("hip=[2.1, 2.2]", message)
        self.assertIn("hip_rate=[-1.5, 1.5]", message)

    def test_unknown_or_nonfinite_code_does_not_raise(self):
        for code in (99.0, -1.0, float("nan"), 2.5):
            self.assertIn("stand-up fault unknown", fault_message(code, [0.0] * 18))


class TraceTests(unittest.TestCase):
    @unittest.skipUnless(HELPER.is_file(), "build the host helper with scripts/build.sh host")
    def test_trace_keeps_samples_before_a_fault(self):
        import json, os, tempfile
        path = os.path.join(tempfile.mkdtemp(), "trace.json")
        process = StandupProcess(HELPER, trace_file=path)
        try:
            process.reset()
            good = [0.0, 0.0, 0.0, 0.0, 0.0, 9.81, 2.357, 2.357, 0, 0, 0, 0] + [0.0] * 6 + [0.0] * 5 + [1.0, 1.0, 0.0, 0.0]
            good = good[:24]
            process.exchange(good)
            bad = list(good)
            bad[0] = float("nan")  # non-finite input makes the helper fault
            with self.assertRaises(RuntimeError):
                process.exchange(bad[:23] + [0.0])
        finally:
            process.close()
        with open(path) as stream:
            data = json.load(stream)
        self.assertGreaterEqual(len(data["samples"]), 1)
        self.assertIn("wheelL_cmd", data["samples"][0]["out"])
        self.assertIn("hipL", data["samples"][0]["in"])
