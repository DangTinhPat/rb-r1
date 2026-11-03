"""Fold-down planner, checked against tests/data/fold_reference.json (the ramp as the removed
stand-up Folding phase produced it)."""

import json
from pathlib import Path
import shutil
import subprocess
import unittest

from rb_runtime import protocol as p
from rb_runtime.fold import FOLDED, FOLDING, FoldLogic
from rb_runtime.paths import native_directory

DATA = json.loads((Path(__file__).parent / "data/fold_reference.json").read_text())
RATE_RAD_S, TRAVEL_RAD, FLAGS = 2.4, 3.05, 3


def telemetry(hip_l, hip_r, gravity=(0.0, 0.0, -1.0)):
    return p.Telemetry(1, 7, 0, [0.0] * 3, list(gravity), [0.0, 0.0, 9.81],
                       [hip_l, hip_r, 0.0, 0.0, 0.0, 0.0], [0.0] * 6, [0.0, 0.0], [0.0] * 6)


class FoldReferenceTests(unittest.TestCase):
    def test_ramp_matches_the_removed_stand_up_folding_phase(self):
        columns = {name: i for i, name in enumerate(DATA["columns"])}
        for variant, (hip_l, hip_r) in DATA["home_hips"].items():
            logic = FoldLogic()
            for row in (r for r in DATA["rows"] if r[0] == variant):
                tick = row[columns["tick"]]
                folded, phase, flags, target, ff = logic.step(3.0 + 0.005 * tick, telemetry(hip_l, hip_r))
                where = f"{variant} tick {tick}"
                for key, got in (("hipL", target[0]), ("hipR", target[1]), ("crankL", target[2]),
                                 ("crankR", target[3]), ("wheelL", target[4]), ("wheelR", target[5]),
                                 ("ffHipL", ff[0]), ("ffHipR", ff[1]), ("ffKneeL", ff[2]), ("ffKneeR", ff[3])):
                    self.assertAlmostEqual(got, row[columns[key]], delta=2e-4, msg=f"{where} {key}")
                self.assertEqual(phase, FOLDED if tick >= DATA["fold_complete_tick"] else FOLDING, where)
                self.assertEqual(folded, tick >= DATA["fold_complete_tick"], where)
                # The reference has enables=1 at tick 0 (a one-tick wheel off/on glitch). The
                # planner keeps legs+wheels enabled from the first tick, as in HOME.
                self.assertEqual(flags, FLAGS)
                self.assertEqual(row[columns["enables"]], 1 if tick == 0 else 3)

    def test_both_hips_follow_the_mean_of_the_measured_hips(self):
        folded, _, _, target, _ = FoldLogic().step(3.0, telemetry(2.30, 2.40))
        self.assertFalse(folded)
        self.assertAlmostEqual(target[0], 2.35)
        self.assertEqual(target[0], target[1])


class FoldBehaviourTests(unittest.TestCase):
    def test_hold_after_the_fold_and_no_lag_after_lost_frames(self):
        logic = FoldLogic()
        logic.step(3.0, telemetry(2.357, 2.357))
        # Three frames never arrived: the reference is where the clock says, not one tick behind.
        _, _, _, target, _ = logic.step(3.020, telemetry(2.357, 2.357))
        self.assertAlmostEqual(target[0], 2.357 - RATE_RAD_S * 0.020)
        done = 3.0 + TRAVEL_RAD / RATE_RAD_S + 0.001
        folded, phase, _, held, _ = logic.step(done, telemetry(2.357, 2.357))
        self.assertTrue(folded)
        self.assertEqual(phase, FOLDED)
        self.assertAlmostEqual(held[0], 2.357 - TRAVEL_RAD)
        for later in (done + 1.0, done + 8.0):
            folded, _, _, again, ff = logic.step(later, telemetry(2.357, 2.357))
            self.assertTrue(folded)
            self.assertEqual(again[0], held[0])
        self.assertLess(abs(ff[0]), 1e-6)  # feed-forward has decayed once the hips hold

    def test_rate_feed_forward_is_filtered_and_bounded(self):
        logic = FoldLogic()
        logic.step(3.0, telemetry(2.357, 2.357))
        _, _, _, _, ff = logic.step(3.005, telemetry(2.357, 2.357))
        self.assertAlmostEqual(ff[0], -0.2 * RATE_RAD_S)  # dt / tau = 0.2 of the 2.4 rad/s step
        for tick in range(2, 60):
            _, _, _, _, ff = logic.step(3.0 + 0.005 * tick, telemetry(2.357, 2.357))
        self.assertAlmostEqual(ff[0], -RATE_RAD_S, delta=0.01)
        self.assertEqual(ff[2], 0.0)

    def test_reset_restarts_from_the_present_hip(self):
        logic = FoldLogic()
        logic.step(3.0, telemetry(2.357, 2.357))
        logic.step(4.0, telemetry(2.357, 2.357))
        logic.reset()
        _, phase, _, target, ff = logic.step(9.0, telemetry(1.0, 1.0))
        self.assertEqual(phase, FOLDING)
        self.assertEqual(target[0], 1.0)
        self.assertEqual(ff[0], 0.0)

    def test_two_planners_do_not_share_state(self):
        a, b = FoldLogic(), FoldLogic()
        a.step(3.0, telemetry(2.0, 2.0))
        a.step(3.1, telemetry(2.0, 2.0))
        _, _, _, target, _ = b.step(50.0, telemetry(1.5, 1.5))
        self.assertEqual(target[0], 1.5)

    def test_pitch_limit_is_the_atan2_threshold(self):
        # |pitch| = atan2(g0, hypot(g1, g2)); the native check avoids libm by comparing squares.
        import math
        for pitch in (0.79, 0.7999, 0.8001, 0.81, -0.7999, -0.8001):
            g = (math.sin(pitch), 0.0, -math.cos(pitch))
            logic = FoldLogic()
            if abs(pitch) > 0.8:
                with self.assertRaisesRegex(RuntimeError, "PitchLimit"):
                    logic.step(3.0, telemetry(2.357, 2.357, gravity=g))
            else:
                logic.step(3.0, telemetry(2.357, 2.357, gravity=g))

    def test_faults_do_not_produce_a_target(self):
        for bad, text in ((telemetry(float("nan"), 2.357), "InvalidInput"),
                          (telemetry(2.357, 2.357, gravity=(0.9, 0.0, -0.4)), "PitchLimit")):
            with self.assertRaisesRegex(RuntimeError, text):
                FoldLogic().step(3.0, bad)
        with self.assertRaisesRegex(RuntimeError, "InvalidInput"):
            FoldLogic().step(float("inf"), telemetry(2.357, 2.357))


class LibraryHygiene(unittest.TestCase):
    @unittest.skipUnless(shutil.which("nm") and shutil.which("readelf"), "binutils required")
    def test_native_libraries_need_nothing_from_the_target_libc(self):
        for name in ("libfold_planner.so", "libtensor_math.so"):
            path = str(native_directory() / "lib" / name)
            undefined = subprocess.check_output(["nm", "-D", "--undefined-only", path], text=True).split()
            self.assertEqual(undefined, [], f"{name} imports {undefined}")
            self.assertNotIn("(NEEDED)", subprocess.check_output(["readelf", "-d", path], text=True), name)


if __name__ == "__main__":
    unittest.main()
