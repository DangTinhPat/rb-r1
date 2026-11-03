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

    def test_encoder_home_contract_vectors(self):
        # Same independent wire vectors are checked by the MCU core tests.
        expected = bytes.fromhex(
            'a55a01830800000003000000030000000000000007000000010000006539d111'
            'a55a01050c00000002000000a186010000000000070000000300000001000000eb8da1a6')
        request = p.Frame(p.ENCODER_HOME, 3, 3, p.ENCODER_HOME_DATA.pack(7, 1))
        ack = p.Frame(p.ENCODER_HOME_ACK, 2, 100001, p.ENCODER_HOME_ACK_DATA.pack(7, 3, p.HOME_APPLIED))
        self.assertEqual(request.encode() + ack.encode(), expected)
        self.assertEqual(subprocess.check_output([self.binary, 'home']), expected)

    def test_execution_diagnostics_c_python_contract(self):
        values = [7, 1] + list(range(2, 24))
        raw = p.Frame(p.EXECUTION_DIAG, 9, 123456, p.EXECUTION_DIAG_DATA.pack(*values)).encode()
        self.assertEqual(raw.hex(), 'a55a0106600000000900000040e2010000000000070000000100000002000000030000000400000005000000060000000700000008000000090000000a0000000b0000000c0000000d0000000e0000000f00000010000000110000001200000013000000140000001500000016000000170000007d72bbac')
        self.assertEqual(subprocess.check_output([self.binary, 'diagnostics']), raw)
        self.assertEqual(p.decode_execution_diagnostics(raw[20:-4])['applied_latency_max_us'], 23)
