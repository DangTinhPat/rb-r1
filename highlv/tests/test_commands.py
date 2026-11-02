import unittest

from rb_runtime.commands import parse_command


class CommandTests(unittest.TestCase):
    def test_operator_commands_are_separate_from_uart_frames(self):
        self.assertEqual(parse_command(b"arm\n"), ("ARM", 0.0, 0.0))
        self.assertEqual(parse_command(b"VELOCITY .5 -.3\n"), ("VELOCITY", 0.5, -0.3))

    def test_malformed_commands_do_not_arm_or_set_velocity(self):
        for data in [
            b"",
            b"ARM extra",
            b"RESET extra",
            b"VELOCITY 1",
            b"VELOCITY nan 0",
            b"VELOCITY 0 inf",
            b"UNKNOWN",
        ]:
            with self.subTest(data=data), self.assertRaises(ValueError):
                parse_command(data)
