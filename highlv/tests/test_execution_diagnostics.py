"""Optional measurements cannot become control evidence or renew watchdogs."""
import unittest
from test_hybrid_core import Rig
from rb_runtime import protocol as p


class ExecutionDiagnosticsTests(unittest.TestCase):
    def payload(self, r, **overrides):
        d = dict.fromkeys(p.EXECUTION_DIAG_FIELDS, 0)
        d.update(epoch=r.core.epoch, version=1, flags=0x80000004)
        d.update(overrides)
        return p.EXECUTION_DIAG_DATA.pack(*(d[k] for k in p.EXECUTION_DIAG_FIELDS))

    def test_diagnostics_are_not_telemetry_source_ack_or_output(self):
        r = Rig(); r.control('ARM'); r.telemetry()
        before = (r.core.last_rx, r.core.last_stamp, r.core.last_t,
                  r.core.last_ack, r.core.last_ack_rx, r.core.encoder_source,
                  r.core.counters['telemetry_received'], r.core.tx_seq)
        for _ in range(7):
            r.stamp += 5000
            self.assertEqual(r.send(p.EXECUTION_DIAG, self.payload(r, motor_error=5)), [])
        after = (r.core.last_rx, r.core.last_stamp, r.core.last_t,
                 r.core.last_ack, r.core.last_ack_rx, r.core.encoder_source,
                 r.core.counters['telemetry_received'], r.core.tx_seq)
        self.assertEqual(before, after)
        self.assertEqual(r.core.mode, 'HOME')  # measurements cannot latch/control
        snapshot = r.core.snapshot(r.stamp / 1e6 + .2)
        self.assertAlmostEqual(snapshot['mcu_diagnostics']['age_s'], .2)
        self.assertEqual(snapshot['mcu_diagnostics']['motor_error'], 5)
        self.assertTrue(r.core.watchdog(r.stamp / 1e6))
        self.assertEqual(r.core.fault, 1)  # live diagnostics did not renew telemetry
        r.core.new_epoch()
        self.assertIsNone(r.core.snapshot()['mcu_diagnostics'])

    def test_bad_version_length_epoch_and_replay_do_not_fault_or_refresh(self):
        r = Rig()
        good = self.payload(r)
        for payload in (good[:-1], self.payload(r, version=2), self.payload(r, epoch=0)):
            self.assertEqual(r.send(p.EXECUTION_DIAG, payload), [])
            self.assertEqual(r.core.mode, 'DISARMED')
            self.assertIsNone(r.core.mcu_diagnostics)
        r.send(p.EXECUTION_DIAG, good)
        rx = r.core.mcu_diagnostics_rx
        self.assertEqual(r.core.receive(p.Frame(p.EXECUTION_DIAG, r.seq, r.stamp, good), rx+1), [])
        self.assertEqual(r.core.mcu_diagnostics_rx, rx)
        # A late diagnostic carries its transport age; it cannot look fresh.
        r.stamp += 500000
        r.seq += 1
        r.core.receive(p.Frame(p.EXECUTION_DIAG, r.seq, 100000, good), r.stamp/1e6)
        self.assertGreater(r.core.mcu_diagnostics['transport_age_s'], .4)

    def test_measurements_available_before_imu_calibration_without_control_clock(self):
        r = Rig(); r.core.new_epoch()
        r.send(p.HELLO_ACK, p.HELLO_DATA.pack(r.core.epoch, 200, 100, p.SCHEMA))
        self.assertIsNone(r.core.offset_us)
        self.assertIsNone(r.core.last_rx)
        r.send(p.EXECUTION_DIAG, self.payload(r, local_fault=2))
        self.assertEqual(r.core.mcu_diagnostics['transport_age_s'], 0)
        self.assertIsNone(r.core.offset_us)
        self.assertIsNone(r.core.last_rx)
        self.assertEqual(r.core.mode, 'DISARMED')
