"""Local dashboard; HTTP threads queue commands, ROS owns all robot state."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread

class Dashboard:
    def __init__(self, commands, port=8765):
        self.snapshot='{}'
        page_path=Path(__file__).with_name('dashboard.html')
        outer=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args): pass
            def respond(self,status,data,kind='application/json'):
                self.send_response(status)
                self.send_header('Content-Type',kind)
                self.send_header('Cache-Control','no-store')
                self.send_header('X-Content-Type-Options','nosniff')
                self.end_headers(); self.wfile.write(data)
            def do_GET(self):
                if self.path=='/': self.respond(200,page_path.read_bytes(),'text/html; charset=utf-8')
                elif self.path=='/api/state': self.respond(200,outer.snapshot.encode())
                else: self.respond(404,b'{}')
            def do_POST(self):
                origin=self.headers.get('Origin')
                if origin and origin not in ('http://localhost:8765','http://127.0.0.1:8765'):
                    self.respond(403,b'{"error":"Local origin required"}'); return
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=2048: raise ValueError()
                    body=json.loads(self.rfile.read(length))
                    if self.path!='/api/command' or body.get('command') not in ('start','stop','inject','ack'):
                        raise ValueError()
                    commands.put_nowait(body)
                    self.respond(202,b'{"queued":true}')
                except Exception: self.respond(400,b'{"error":"Invalid command or busy queue"}')
        self.server=ThreadingHTTPServer(('0.0.0.0',port),Handler)
        Thread(target=self.server.serve_forever,daemon=True).start()

    def close(self):
        self.server.shutdown(); self.server.server_close()
