"""Optional loopback-only latest-frame sensor monitor; no ROS or dependencies."""
import json
import struct
import threading
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE=b'''<!doctype html><meta charset="utf-8"><title>kart_sim D455</title>
<style>body{background:#171a20;color:#eee;font:16px sans-serif;margin:24px}section{display:flex;gap:14px;flex-wrap:wrap}img{width:424px;height:240px;background:#000}pre{white-space:pre-wrap}</style>
<h1>kart_sim / D455 nominal</h1><section><div>Left mono8<br><img id="infra1"></div><div>Right mono8<br><img id="infra2"></div><div>RGB<br><img id="color"></div></section><pre id="state"></pre>
<script>setInterval(async()=>{for(const name of ['infra1','infra2','color'])document.getElementById(name).src='/'+name+'.png?t='+Date.now();document.getElementById('state').textContent=JSON.stringify(await(await fetch('/state')).json(),null,2)},200)</script>'''

def png(pixels):
    height,width=pixels.shape[:2]
    def chunk(tag,data):
        return struct.pack('!I',len(data))+tag+data+struct.pack('!I',zlib.crc32(tag+data)&0xffffffff)
    payload=b''.join(b'\x00'+row.tobytes() for row in pixels)
    color=0 if pixels.ndim==2 else 2
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,color,0,0,0))+chunk(b'IDAT',zlib.compress(payload))+chunk(b'IEND',b'')

class SensorMonitor:
    def __init__(self):
        self.images={}; self.metadata={}; self.lock=threading.Lock()
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path=self.path.split('?')[0]
                with owner.lock:
                    if path=='/': data,kind=PAGE,'text/html; charset=utf-8'
                    elif path=='/state': data,kind=json.dumps(owner.metadata).encode(),'application/json'
                    elif path[1:-4] in owner.images and path.endswith('.png'): data,kind=owner.images[path[1:-4]],'image/png'
                    else: self.send_error(404); return
                self.send_response(200); self.send_header('Content-Type',kind); self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(data)
            def log_message(self,*args): pass
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True); self.thread.start()
        print(f'Sensor monitor: http://127.0.0.1:{self.server.server_port}/',flush=True)

    def update(self,sim):
        images={name:png(frame.values['image']) for name,frame in sim.sensors.latest.items()}
        pos,quat,linear,angular=sim.state()
        metadata={'sim_time':float(sim.data.time),'position_m':pos.tolist(),'orientation_wxyz':quat.tolist(),
            'body_velocity_mps':linear.tolist(),'body_angular_velocity_rad_s':angular.tolist(),
            'image_stamps':{name:frame.time for name,frame in sim.sensors.latest.items()}}
        if sim.imu_enabled:
            imu=sim.imu_state()
            metadata['imu']={key:value.tolist() if hasattr(value,'tolist') else value for key,value in imu.items()}
        with self.lock: self.images=images; self.metadata=metadata

    def close(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(timeout=1)
