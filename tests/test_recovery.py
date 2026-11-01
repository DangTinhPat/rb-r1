from pathlib import Path
from types import SimpleNamespace
import unittest

from rb_runtime.recovery import StandupProcess

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
