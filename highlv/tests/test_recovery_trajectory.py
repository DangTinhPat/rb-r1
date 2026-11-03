"""Compare native stand-up outputs with frozen Lumi-Quantum sensor traces."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("c++"), "host C++ compiler required")
class RecoveryTrajectoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        binary = str(Path(cls.directory.name) / "trajectory")
        subprocess.run([
            "c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(ROOT / "native/include"),
            str(ROOT / "tests/fixtures/standup_trajectory_probe.cpp"), "-o", binary,
        ], check=True)
        cls.actual = json.loads(subprocess.check_output([binary]))
        cls.reference = json.loads((ROOT / "tests/data/standup_lumi_reference.json").read_text())

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_rb_tuning_matches_lumi_quantum(self):
        self.assertEqual(self.actual["tuning"], self.reference["outputs"]["tuning"])

    def test_native_trajectory_matches_lumi_quantum(self):
        for scenario in ("rb_sequence", "rise_brake_measured", "stalled_knee"):
            with self.subTest(scenario=scenario):
                wanted = self.reference["outputs"][scenario]
                actual = self.actual[scenario]
                self.assertEqual(len(actual), len(wanted))
                for row, expected in zip(actual, wanted):
                    self.assertEqual(row[:3], expected[:3])  # tick, phase, failure
                    for value, target in zip(row[3:], expected[3:]):
                        self.assertAlmostEqual(value, target, delta=2e-6)
                self.assertTrue(all(row[2] == 0 for row in actual))
        # The source trajectory really traversed all three recovery phases.
        self.assertTrue({1, 2, 3}.issubset({r[1] for r in self.actual["rb_sequence"]}))
        self.assertLessEqual(max(r[4] for r in self.actual["stalled_knee"]), .15001)

    def test_adapter_paces_slower_knee_and_does_not_delay_wheels(self):
        library = ROOT / "build/host/librb_recovery.a"
        if not library.is_file():
            self.skipTest("build the host recovery library with scripts/build.sh host")
        binary = str(Path(self.directory.name) / "adapter")
        subprocess.run([
            "c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
            "-DRB_RECOVERY_SPLIT_SUPPORT=1", "-I", str(ROOT / "native/include"),
            str(ROOT / "tests/fixtures/recovery_adapter_probe.cpp"), str(library),
            "-o", binary,
        ], check=True)
        result = subprocess.check_output([binary], text=True)
        self.assertIn("PASS adapter", result)
