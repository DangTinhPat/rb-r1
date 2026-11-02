from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from rb_runtime import protocol as p

ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(shutil.which("cc"), "host C compiler required")
class CodecInterop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.binary = str(Path(cls.directory.name) / "codec")
        subprocess.run(
            [
                "cc",
                "-std=c99",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "protocol/include"),
                str(ROOT / "tests/fixtures/codec_probe.c"),
                "-o",
                cls.binary,
            ],
            check=True,
        )

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_c_encode_matches_python(self):
        actual = subprocess.check_output([self.binary, "encode"])
        expected = p.Frame(
            p.HELLO_ACK, 42, 123456, p.HELLO_DATA.pack(0x10203040, 200, 100, p.SCHEMA)
        ).encode()
        self.assertEqual(actual, expected)

    def test_python_encode_c_decode_and_latched_watchdog(self):
        packet = p.Frame(
            p.TARGET,
            123,
            999,
            p.TARGET_DATA.pack(
                7, 44, 3, 3, -0.7, -0.6, 0.2, 0.3, 1.0, 2.0, 0.0, 0.0, 0.0, 0.125
            ),
        ).encode()
        damaged = bytearray(packet)
        damaged[-1] ^= 1
        reply = (
            subprocess.check_output([self.binary], input=b"noise" + damaged + packet)
            .decode()
            .split()
        )
        self.assertEqual(reply[:3], ["123", "7", "3"])
        self.assertAlmostEqual(float(reply[3]), -0.7, places=6)
        self.assertEqual(float(reply[4]), 0.125)
