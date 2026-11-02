"""Check channel order against explicit HCR1/model values, not just roundtrips."""

import unittest

from rb_runtime import protocol as p
from rb_runtime.policy import decode_actions, observation


class FlowContractTests(unittest.TestCase):
    def test_observation_channels_and_measured_knee_coordinates(self):
        t = p.Telemetry(
            11,
            7,
            9,
            [1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0],
            [101.0, 102.0, 103.0],
            [-0.6, -0.5, 1.2, 1.3, 200.0, 300.0],
            [7.0, 8.0, 9.0, 10.0, 11.0, 12.0],
            [104.0, 105.0],
            [0.0] * 6,
        )
        obs = observation(t, [13.0, 14.0, 15.0, 16.0, 17.0, 18.0], [0.4, -0.8])
        expected = [
            1.0,
            2.0,
            3.0,
            4.0,
            5.0,
            6.0,
            0.4,
            0.0,
            -0.8,
            0.1,
            0.2,
            0.3,
            0.4,
            7.0,
            8.0,
            9.0,
            10.0,
            11.0,
            12.0,
            13.0,
            14.0,
            15.0,
            16.0,
            17.0,
            18.0,
            0.0,
            0.0,
            0.16,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.assertEqual(len(obs), 32)
        for actual, wanted in zip(obs, expected):
            self.assertAlmostEqual(actual, wanted)

    def test_actions_produce_absolute_crank_angles_and_wheel_rates(self):
        result = decode_actions([2.0, -2.0, 4.0, -4.0, 0.35, -0.7])
        expected = [-0.2, -1.2, 1.2, -0.8, 8.0, -16.0]
        for actual, wanted in zip(result, expected):
            self.assertAlmostEqual(actual, wanted)
        # Values beyond [-1,1] are intentional actor outputs, not clipped commands.
        self.assertAlmostEqual(result[2] - result[0], 1.4)

    def test_target_wire_order_and_feedforward_are_independent(self):
        raw = p.TARGET_DATA.pack(
            17, 23, p.RECOVERY, 3, -0.2, -0.3, 0.4, 0.5, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0
        )
        self.assertEqual(len(raw), 52)
        fields = p.TARGET_DATA.unpack(raw)
        self.assertEqual(fields[:4], (17, 23, p.RECOVERY, 3))
        for actual, wanted in zip(
            fields[4:], [-0.2, -0.3, 0.4, 0.5, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0]
        ):
            self.assertAlmostEqual(actual, wanted, places=6)
