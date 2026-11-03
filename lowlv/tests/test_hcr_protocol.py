"""MCU tool codec matches the HCR1 disarmed HOME contract (no hardware)."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import hcr_protocol as p


class HomeCodecTests(unittest.TestCase):
    def test_home_wire_vectors(self):
        # Duplicated contract vectors, with no source/build dependency on LF.
        vectors = [
            (p.Frame(p.ENCODER_HOME, 3, 3, p.ENCODER_HOME_DATA.pack(7, 1)),
             'a55a01830800000003000000030000000000000007000000010000006539d111'),
            (p.Frame(p.ENCODER_HOME_ACK, 2, 100001, p.ENCODER_HOME_ACK_DATA.pack(7, 3, p.HOME_APPLIED)),
             'a55a01050c00000002000000a186010000000000070000000300000001000000eb8da1a6'),
        ]
        for frame, raw in vectors:
            self.assertEqual(frame.encode().hex(), raw)
            self.assertEqual(p.Parser().feed(bytes.fromhex(raw)), [frame])

    def test_execution_diagnostics_wire_vector(self):
        raw = p.Frame(p.EXECUTION_DIAG, 9, 123456,
            p.EXECUTION_DIAG_DATA.pack(7, 1, *range(2, 24))).encode()
        self.assertEqual(raw.hex(), 'a55a0106600000000900000040e2010000000000070000000100000002000000030000000400000005000000060000000700000008000000090000000a0000000b0000000c0000000d0000000e0000000f00000010000000110000001200000013000000140000001500000016000000170000007d72bbac')
        self.assertEqual(p.decode_execution_diagnostics(p.Parser().feed(raw)[0].payload)['io_l_us'], 11)
        with self.assertRaises(ValueError):
            p.decode_execution_diagnostics(p.EXECUTION_DIAG_DATA.pack(7, 2, *range(2, 24)))
