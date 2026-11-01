import os
import pty
import select
import unittest
from unittest.mock import patch
from rb_runtime.protocol import Frame, Parser
from rb_runtime.transport import open_serial, Writer


class TransportTests(unittest.TestCase):
    def test_uart_full_duplex_on_pty(self):
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        os.close(slave)
        fd = open_serial(path, 921600)
        try:
            packet = Frame(4, 9, 123, b"command")
            writer = Writer(fd)
            writer.add([packet])
            writer.flush()
            self.assertTrue(select.select([master], [], [], 1)[0])
            self.assertEqual(Parser().feed(os.read(master, 1024)), [packet])
            os.write(master, packet.encode())
            self.assertTrue(select.select([fd], [], [], 1)[0])
            self.assertEqual(Parser().feed(os.read(fd, 1024)), [packet])
        finally:
            os.close(fd)
            os.close(master)

    def test_writer_preserves_partial_frame(self):
        writer = Writer(123)
        packet = Frame(4, 1, 22, b"abcd")
        raw = packet.encode()
        writer.add([packet])
        with patch("rb_runtime.transport.os.write", return_value=7):
            writer.flush()
        self.assertEqual(writer.pending, raw[7:])
        with patch("rb_runtime.transport.os.write", side_effect=BlockingIOError):
            writer.flush()
        self.assertEqual(writer.pending, raw[7:])
        with patch("rb_runtime.transport.os.write", return_value=len(raw) - 7):
            writer.flush()
        self.assertFalse(writer.pending)
