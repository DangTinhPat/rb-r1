"""Operator encoder-zero flow over HCR1, with no physical endpoints."""
import unittest
from test_hybrid_core import Rig
from rb_runtime import protocol as p
from rb_runtime.commands import parse_command


class EncoderResetTests(unittest.TestCase):
    def test_capture_ack_resets_session_stays_disabled_and_never_replays(self):
        r = Rig()
        self.assertEqual(parse_command(b'RESET_ENCODERS\n'), ('RESET_ENCODERS',0.,0.))
        frames = r.control('RESET_ENCODERS')
        self.assertEqual(len(frames),1)
        request = frames[0]
        self.assertEqual(request.kind,p.ENCODER_HOME)
        self.assertEqual(p.ENCODER_HOME_DATA.unpack(request.payload), (r.core.epoch,r.seq))
        r.control('RESET_ENCODERS')
        self.assertIsNotNone(r.core.last_command_error)
        r.control('ARM')
        self.assertEqual(r.core.mode,'DISARMED')
        self.assertIn('Reset encoder',r.core.last_command_error)
        old_epoch = r.core.epoch
        frames = r.send(p.ENCODER_HOME_ACK,p.ENCODER_HOME_ACK_DATA.pack(old_epoch,request.seq,p.HOME_APPLIED))
        self.assertNotEqual(r.core.epoch,old_epoch)
        self.assertEqual([f.kind for f in frames],[p.TARGET,p.HELLO,p.STATUS])
        self.assertEqual(p.TARGET_DATA.unpack(frames[0].payload)[2:4],(p.DISABLED,0))
        self.assertEqual(r.core.encoder_reset['state'],'done')
        self.assertEqual(r.core.watchdog(r.stamp/1e6),[])
        r.send(p.HELLO_ACK,p.HELLO_DATA.pack(r.core.epoch,200,100,p.SCHEMA))
        self.assertEqual(r.core.mode,'DISARMED')

    def test_home_is_available_before_imu_home_validity_but_not_driver_fault(self):
        r = Rig(); r.t.flags = p.ENCODER_OK; r.telemetry()
        self.assertEqual(r.core.mode,'FAULT')
        self.assertEqual(r.core.fault,2)
        frames = r.control('RESET_ENCODERS')
        self.assertEqual(frames[0].kind,p.ENCODER_HOME)
        r.core.estop()
        r.send(p.ENCODER_HOME_ACK,p.ENCODER_HOME_ACK_DATA.pack(r.core.epoch,frames[0].seq,p.HOME_APPLIED))
        self.assertEqual(r.core.mode,'FAULT')
        self.assertEqual(r.core.fault,3)
        self.assertNotEqual(r.core.encoder_reset['state'],'done')
        self.assertFalse(any(f.kind==p.ENCODER_HOME for f in r.control('RESET_ENCODERS')))

    def test_timeout_refusal_mismatched_reply_and_active_mode(self):
        for result in (p.HOME_NOT_OFF,p.HOME_FEEDBACK,p.HOME_REFUSED):
            r = Rig(); request = r.control('RESET_ENCODERS')[0]
            r.send(p.ENCODER_HOME_ACK,p.ENCODER_HOME_ACK_DATA.pack(r.core.epoch,request.seq+1,p.HOME_APPLIED))
            self.assertEqual(r.core.encoder_reset['state'],'pending')
            r.send(p.ENCODER_HOME_ACK,p.ENCODER_HOME_ACK_DATA.pack(r.core.epoch,request.seq,result))
            self.assertEqual(r.core.encoder_reset['state'],'error')
            self.assertEqual(r.core.mode,'DISARMED')
        r = Rig();r.control('RESET_ENCODERS')
        self.assertEqual(r.core.watchdog(r.stamp/1e6+1.1),[])
        self.assertEqual(r.core.encoder_reset['state'],'error')
        r = Rig();r.control('ARM')
        self.assertFalse(any(f.kind==p.ENCODER_HOME for f in r.control('RESET_ENCODERS')))
        r = Rig();r.stamp+=31000
        self.assertFalse(any(f.kind==p.ENCODER_HOME for f in r.control('RESET_ENCODERS')))
