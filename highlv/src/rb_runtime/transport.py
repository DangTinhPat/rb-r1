import fcntl
import glob
import os
import sys
import termios


def open_serial(path, baud):
    real = os.path.realpath(path)
    owners = []
    for name in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            if os.path.realpath(name) == real:
                owners.append(name.split("/")[2])
        except OSError:
            pass
    if owners:
        raise RuntimeError(
            "UART is already open by PID(s): " + ",".join(sorted(set(owners)))
        )
    speed = getattr(termios, "B" + str(baud), None)
    if speed is None:
        raise ValueError("unsupported baud")
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        _low_latency(fd)
        return fd
    except Exception:
        os.close(fd)
        raise


TIOCGSERIAL, TIOCSSERIAL, ASYNC_LOW_LATENCY = 0x541E, 0x541F, 0x2000


def _low_latency(fd):
    """Deliver received bytes from the interrupt path instead of a deferred workqueue.

    Best effort: not every driver implements it (a pty does not)."""
    try:
        info = bytearray(fcntl.ioctl(fd, TIOCGSERIAL, bytes(128)))
        flags = int.from_bytes(info[16:20], sys.byteorder) | ASYNC_LOW_LATENCY  # serial_struct.flags
        info[16:20] = flags.to_bytes(4, sys.byteorder)
        fcntl.ioctl(fd, TIOCSSERIAL, bytes(info))
    except (OSError, ValueError):
        pass


class Writer:
    def __init__(self, fd):
        self.fd = fd
        self.pending = bytearray()

    def add(self, frames):
        for frame in frames:
            self.pending.extend(frame.encode())
        if len(self.pending) > 4096:
            raise RuntimeError("output transport backpressure")

    def flush(self):
        if not self.pending:
            return
        try:
            count = os.write(self.fd, self.pending)
            del self.pending[:count]
        except BlockingIOError:
            pass
