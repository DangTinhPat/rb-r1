"""Model artifacts and the policy loader, as far as a host without the NPU can check them.

The RKNN model itself only runs on the board; there is no ONNX source to compare against."""

import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from rb_runtime.policy import Policy

ROOT = Path(__file__).resolve().parents[1]


class ArtifactTests(unittest.TestCase):
    def test_manifest_lists_exactly_the_shipped_artifacts_with_matching_hashes(self):
        manifest = json.loads((ROOT / "models/manifest.json").read_text())
        self.assertEqual(
            sorted(manifest["sha256"]),
            ["models/fitted22500_dfp16_jumprec.rknn", "models/fitted22500_normalizer.json"],
        )
        for name, expected in manifest["sha256"].items():
            self.assertEqual(hashlib.sha256((ROOT / name).read_bytes()).hexdigest(), expected, name)
        self.assertEqual(sorted(p.name for p in (ROOT / "models").iterdir()),
                         ["fitted22500_dfp16_jumprec.rknn", "fitted22500_normalizer.json", "manifest.json"])

    def test_normalizer_covers_the_32_observation_channels(self):
        normalizer = json.loads((ROOT / "models/fitted22500_normalizer.json").read_text())
        for key in ("mean", "divisor"):
            self.assertEqual(len(normalizer[key]), 32, key)
            self.assertTrue(all(isinstance(v, float) for v in normalizer[key]), key)
        self.assertTrue(all(v != 0 for v in normalizer["divisor"]))

    def test_recorded_observations_are_32_finite_values(self):
        rows = json.loads((ROOT / "tests/data/policy_observations.json").read_text())["obs"]
        self.assertEqual(len(rows), 100)
        self.assertTrue(all(len(r) == 32 and all(abs(v) < 1e6 for v in r) for r in rows))


class LoaderTests(unittest.TestCase):
    def test_a_tampered_artifact_is_refused_before_anything_is_loaded(self):
        with tempfile.TemporaryDirectory() as directory:
            shutil.copytree(ROOT / "models", Path(directory) / "models")
            (Path(directory) / "models/fitted22500_normalizer.json").write_text("{}")
            with self.assertRaisesRegex(ValueError, "artifact hash mismatch"):
                Policy(directory)

    def test_only_the_rknn_backend_exists(self):
        with self.assertRaisesRegex(ValueError, "unsupported backend"):
            Policy(ROOT, "onnx")


if __name__ == "__main__":
    unittest.main()
