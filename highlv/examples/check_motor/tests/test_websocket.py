import base64
import json
import os
from pathlib import Path
import socket
import struct
import sys
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from server import Bridge,make_handler

class WebSocketProtocol(unittest.TestCase):
    def test_authenticated_stream_renews_and_closes_only_its_owner(self):
        b=Bridge(-1);b.hello=True;b.telemetry_at=time.monotonic();b.connect('a')
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(b,'test-token'))
        thread=threading.Thread(target=server.serve_forever,kwargs={'poll_interval':.01});thread.start()
        s=socket.create_connection(server.server_address);s.settimeout(1)
        f=s.makefile('rb')
        def send(message):
            data=json.dumps(message).encode();mask=b'abcd';n=len(data)
            header=bytes([0x81,0x80|n]) if n<126 else bytes([0x81,0xfe])+struct.pack('!H',n)
            s.sendall(header+mask+bytes(v^mask[i%4] for i,v in enumerate(data)))
        def receive():
            h=f.read(2);self.assertEqual(h[0],0x81);n=h[1]&127
            if n==126:n=struct.unpack('!H',f.read(2))[0]
            return json.loads(f.read(n))
        try:
            key=base64.b64encode(os.urandom(16)).decode()
            s.sendall((f'GET /api/stream HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: {key}\r\n\r\n').encode())
            self.assertIn(b'101',f.readline())
            while f.readline()!=b'\r\n':pass
            send({'type':'auth','token':'test-token','client':'a'})
            self.assertEqual(receive()['type'],'ready')
            before=b.pulse_at;send({'type':'heartbeat','seq':1})
            self.assertEqual(receive(),{'type':'pong','seq':1});self.assertGreaterEqual(b.pulse_at,before)
            # A previous stream ending must not revoke a newly connected client.
            b.command('a','STOP');b.connect('b')
            s.shutdown(socket.SHUT_RDWR);f.close();s.close()
            time.sleep(.02);self.assertEqual(b.owner,'b')
            b.disconnect('b','test close');self.assertFalse(b.owner)
        finally:
            f.close();s.close();server.shutdown();thread.join(1);server.server_close()

if __name__=='__main__':unittest.main()
