"""Temporary PTY/Unix RPC; no hardware. UART must survive a web-process GIL stall."""
import os
from pathlib import Path
import pty
import select
import sys
import time
import tty
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from server import BridgeProcess
TEL = '#T 1000 1000 0 0' + ' 1,0,0,00,10,0,10,0'*6

class ProcessIsolation(unittest.TestCase):
    def test_uart_heartbeat_survives_web_gil_stall(self):
        master,slave=pty.openpty();tty.setraw(slave);os.set_blocking(slave,False)
        read_result,write_result=os.pipe()
        emulator=os.fork()
        if emulator==0:
            os.close(slave);os.close(read_result)
            buf=b'';beats=[];now=time.monotonic();end=now+3;next_tel=now
            while time.monotonic()<end:
                if select.select([master],[],[],.002)[0]:
                    try:data=os.read(master,4096)
                    except OSError:break
                    if not data:break
                    buf+=data
                    while b'\n' in buf:
                        line,buf=buf.split(b'\n',1)
                        if line==b'HELLO':os.write(master,b'#HELLO MOTOR_BENCH_1\n')
                        if line.startswith(b'P '):beats.append(time.monotonic())
                if time.monotonic()>=next_tel:
                    try:os.write(master,('#S 1 0 63\n'+TEL+'\n').encode())
                    except OSError:break
                    next_tel=time.monotonic()+.02
            os.write(write_result,str(len(beats)).encode());os._exit(0)
        os.close(master);os.close(write_result)
        proxy=BridgeProcess(fd=slave)
        try:
            end=time.monotonic()+1
            while not proxy.snapshot('a')['connected'] and time.monotonic()<end:time.sleep(.01)
            proxy.connect('a')
            # Simulate web serialization/UI workload holding the parent's GIL.
            before=time.monotonic();switch=sys.getswitchinterval();sys.setswitchinterval(.5)
            try:
                while time.monotonic()-before<.2:pass
            finally:sys.setswitchinterval(switch)
            state=proxy.snapshot('a')
            self.assertTrue(state['controlling'])
            self.assertEqual(state['uart_pid'],proxy.pid)
            self.assertNotEqual(state['uart_pid'],os.getpid())
            proxy.command('a','STOP','test complete')
        finally:
            proxy.close();os.waitpid(emulator,0)
        count=int(os.read(read_result,100));os.close(read_result)
        self.assertGreater(count,15,'UART paused with the web interpreter GIL')

    def test_parent_ipc_eof_sends_stop_and_exits(self):
        master,slave=pty.openpty();tty.setraw(slave);os.set_blocking(slave,False)
        proxy=BridgeProcess(fd=slave)
        # Simulate web process losing its only socket: no shutdown RPC is sent.
        proxy.peer.close();proxy.running=False
        proxy.close()
        buf=b''
        while select.select([master],[],[],.02)[0]:
            try:data=os.read(master,4096)
            except OSError:break
            if not data:break
            buf+=data
        os.close(master)
        self.assertIn(b'HELLO\n',buf)
        self.assertTrue(buf.endswith(b'STOP\n'),buf)
        with self.assertRaises(ChildProcessError):
            os.waitpid(proxy.pid,os.WNOHANG)

if __name__=='__main__':unittest.main()
