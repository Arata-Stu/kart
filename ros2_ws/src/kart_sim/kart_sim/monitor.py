"""Optional loopback-only latest-frame sensor monitor; no ROS or dependencies."""
import json
import struct
import threading
import zlib
from queue import Full
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE=r'''<!doctype html><meta charset="utf-8"><title>kart_sim monitor</title>
<style>body{background:#171a20;color:#eee;font:15px sans-serif;margin:20px}section{display:flex;gap:12px;flex-wrap:wrap;margin:12px 0}img{width:424px;height:240px;background:#000}pre{white-space:pre-wrap}button,select{padding:8px;margin:3px;max-width:650px}canvas{background:#222936;width:650px;max-width:100%}fieldset{border:1px solid #596474}#notice{color:#ffcc66}.status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px;margin:16px 0}.status-card{padding:16px;border:2px solid #596474;border-radius:10px;background:#242a34}.status-card strong{display:block;font-size:25px;margin:8px 0}.status-card small{display:block;color:#c7cfdb;overflow-wrap:anywhere}.status-card[data-tone="active"]{border-color:#55db88}.status-card[data-tone="recording"]{border-color:#ff626b;background:#38252d}.status-card[data-tone="warning"]{border-color:#ffcc66}.status-card[data-tone="error"]{border-color:#ff626b}#statusError{padding:12px;border:2px solid #ff626b;background:#38252d;white-space:pre-wrap}summary{cursor:pointer;padding:12px}</style>
<h1>kart_sim monitor</h1>
<div class="status-grid" aria-live="polite">
<div class="status-card" id="recordStatus"><span>ROSBAG</span><strong>接続待ち</strong><small>記録状態を取得しています</small></div>
<div class="status-card" id="driveStatus"><span>走行</span><strong>接続待ち</strong><small>走行状態を取得しています</small></div>
<div class="status-card" id="bringupStatus"><span>KART BRINGUP</span><strong>接続待ち</strong><small>準備状態を取得しています</small></div>
<div class="status-card" id="simStatus"><span>SIM</span><strong>接続待ち</strong><small>sim時刻 / 実速度</small></div>
</div><p id="statusError" role="alert" hidden></p><section><div>Left mono8<br><img id="infra1"></div><div>Right mono8<br><img id="infra2"></div><div>RGB<br><img id="color"></div></section>
<fieldset><legend>BEV / コース全景</legend><label><input id="showBev" type="checkbox" onchange="sendSimCommand('bev',this.checked)"> 真上視点を表示（描画負荷が増えます）</label><p id="contactStatus">接触状態を取得しています</p><img id="bev" alt="MuJoCoコース全景" hidden style="width:800px;height:auto;max-width:100%"></fieldset>
<fieldset><legend>Drive / recording</legend><section>
<button onclick="sendSimCommand('stop')">STOP (all drive)</button>
<button onclick="sendSimCommand('auto_start')">Data collection lap (truth)</button>
<button id="kartAuto" onclick="sendSimCommand('kart_auto')" disabled>Kart AUTO (estimated pose)</button>
<button id="recordStart" onclick="sendSimCommand('record_start')">Start rosbag</button><button id="recordStop" onclick="sendSimCommand('record_stop')">Stop rosbag</button><button onclick="sendSimCommand('reset')">Reset sim</button></section>
<p>Steering <input id="steering" type="range" min="-1" max="1" step=".05" value="0"> Throttle <input id="throttle" type="range" min="-.5" max=".5" step=".02" value="0"><button onclick="sendSimCommand('manual',[+document.getElementById('steering').value,+document.getElementById('throttle').value])">Apply manual (sim)</button></p></fieldset>
<fieldset><legend>Terminal 2: kart saved-map bringup</legend><p id="operator">Waiting for sim_bringup</p>
<p>HDMap <select id="hdmap" onchange="fillLines()"></select></p>
<p><input id="vgl" type="checkbox" onchange="fillMaps()"> Use VGL <select id="localmap"></select></p>
<p>VGL model <select id="models"></select></p><p>Lane / line <select id="line"></select></p>
<button onclick="configure()">Load maps / restart bringup (STOP)</button>
<button onclick="selectLine()">Apply lane / line (STOP)</button>
<button onclick="sendSimCommand('kart_unload')">Unload bringup (STOP)</button>
<button onclick="sendSimCommand('kart_refresh')">Refresh assets</button>
<p>HDMap and localization map must share coordinates. Loading does not start AUTO.</p></fieldset>
<fieldset><legend>VSLAM / VGL / HDMap creation</legend><button onclick="sendSimCommand('stop');document.getElementById('studio').src='/studio/';document.getElementById('studio').hidden=false">Open Map Studio (STOP)</button><button onclick="document.getElementById('studio').hidden=true">Hide Studio</button><iframe id="studio" title="Kart Map Studio" hidden style="width:100%;height:800px;border:0"></iframe></fieldset>
<p id="notice"></p><canvas id="trajectory" width="650" height="400"></canvas>
<p>Green: truth / Orange: estimate. Initial alignment for display only; asynchronous, not an error measurement.</p><details><summary>詳細状態（JSON）</summary><pre id="state"></pre></details>
<p>Recording and driving are independent. Stop rosbag and unload bringup before Reset. Map creation remains in Map Studio.</p>
<script>
let assets={},assetsKey='';
async function sendSimCommand(action,value){try{const r=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,value})});const data=await r.json();document.getElementById('notice').textContent=data.message}catch(e){document.getElementById('notice').textContent=String(e)}}
function statusCard(id,title,detail,tone='idle'){const el=document.getElementById(id);el.querySelector('strong').textContent=title;el.querySelector('small').textContent=detail;el.dataset.tone=tone}
function renderStatus(s){
 const bag=s.bag||{},op=s.operator||{},auto=s.autodrive||{},owner=s.control_owner,command=s.manual_command||[];
 const bev=document.getElementById('bev');bev.hidden=!s.bev_enabled;document.getElementById('showBev').checked=!!s.bev_enabled;if(s.bev_enabled&&!s.bev_error)bev.src='/bev.png?t='+Date.now();
 const hit=s.wall_contact;document.getElementById('contactStatus').textContent=s.bev_error||`壁接触: ${hit?'最後 '+Number(hit.sim_time).toFixed(2)+' s / '+hit.geoms.join(', '):'未検出'} / 制御: ${op.tracking||'未接続'}${op.last_tracking_issue?' / 最後の追従異常: '+op.last_tracking_issue.status+' ('+Number(op.last_tracking_issue.sim_time).toFixed(2)+' s)':''}`;
 statusCard('recordStatus',bag.error?'記録エラー':bag.recording?'● 記録中':bag.finalized?'■ 記録停止・保存済み':'■ 記録停止',`${Number(bag.messages||0).toLocaleString()} messages${bag.path?' / '+bag.path:''}`,bag.error?'error':bag.recording?'recording':'idle');
 let drive='制御入力待ち',tone='idle',detail='外部制御の入力を待っています';
 if(owner==='kart'){drive='Kart AUTO';tone='active';detail='推定姿勢による追従';}
 else if(auto.enabled){drive='データ収集周回';tone='active';detail=`真値制御 / ${auto.laps||0} 周完了`;}
 else if(owner==='monitor'){const moving=command[1]>0||command[3]>0;drive=moving?'手動操作中':'■ 走行指令停止';tone=moving?'active':'idle';detail=moving?'monitorから手動操作':'停止指令 / 残速度はSIM欄を確認';}
 if(s.paused){drive='一時停止';tone='warning';detail='simの物理更新を停止しています';}
 statusCard('driveStatus',drive,detail,tone);
 statusCard('bringupStatus',!op.connected?'未接続':op.stopping?'終了処理中':op.error?'エラー':op.ready?'AUTO準備完了':op.running?'準備中':'待機中',!op.connected?'terminal 2未起動 / 接続切れ':op.error||op.tracking||'保存地図を選んで読み込んでください',op.error?'error':op.ready?'active':op.running?'warning':'idle');
 const v=s.body_velocity_mps||[],speed=Math.hypot(...v);
 statusCard('simStatus',s.paused?'一時停止':'稼働中',`sim ${Number(s.sim_time||0).toFixed(2)} s / 実速度 ${speed.toFixed(2)} m/s`,s.paused?'warning':'active');
 const errors=[s.error,bag.error,auto.error,op.error,op.studio_error].filter(Boolean),error=document.getElementById('statusError');error.hidden=!errors.length;error.textContent=[...new Set(errors)].join('\n');
 document.getElementById('recordStart').disabled=!!bag.recording;
 document.getElementById('recordStop').disabled=!bag.recording;
}
function disconnectedStatus(){for(const id of ['recordStatus','driveStatus','bringupStatus','simStatus'])statusCard(id,'状態取得不可','monitorとの接続を確認してください。最後の状態は未確認です。','warning');document.getElementById('kartAuto').disabled=true;}
function options(id,values){const el=document.getElementById(id),old=el.value;el.replaceChildren();for(const value of values){const o=document.createElement('option');o.value=typeof value==='string'?value:JSON.stringify(value);o.textContent=typeof value==='string'?value:value.join(' / ');el.append(o)}if([...el.options].some(o=>o.value===old))el.value=old}
function fillLines(){options('line',(assets.lanes||{})[document.getElementById('hdmap').value]||[])}
function fillMaps(){const vgl=document.getElementById('vgl').checked;options('localmap',assets[vgl?'vgl_bundles':'vslam_maps']||[]);document.getElementById('models').disabled=!vgl}
function configure(){const pair=document.getElementById('line').value;if(!pair){document.getElementById('notice').textContent='Select prepared HDMap and lane first';return}const [lane_id,line_type]=JSON.parse(pair),vgl=document.getElementById('vgl').checked;sendSimCommand('kart_configure',{map_file:document.getElementById('hdmap').value,map_dir:document.getElementById('localmap').value,model_dir:vgl?document.getElementById('models').value:'',enable_vgl:String(vgl),lane_id,line_type})}
function selectLine(){const line=document.getElementById('line').value;if(line)sendSimCommand('kart_line',JSON.parse(line))}
function trace(data){const c=document.getElementById('trajectory'),ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);const paths=[data.truth||[],data.estimated||[]],points=paths.flat();if(!points.length)return;const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]),x0=Math.min(...xs),y0=Math.min(...ys),scale=Math.min((c.width-40)/Math.max(.5,Math.max(...xs)-x0),(c.height-40)/Math.max(.5,Math.max(...ys)-y0));paths.forEach((p,i)=>{ctx.strokeStyle=i?'#ffad42':'#55db88';ctx.lineWidth=2;ctx.beginPath();p.forEach(([x,y],j)=>{const a=20+(x-x0)*scale,b=c.height-20-(y-y0)*scale;j?ctx.lineTo(a,b):ctx.moveTo(a,b)});ctx.stroke()})}
let busy=false;setInterval(async()=>{if(busy)return;busy=true;try{for(const name of ['infra1','infra2','color'])document.getElementById(name).src='/'+name+'.png?t='+Date.now();const response=await fetch('/state',{signal:AbortSignal.timeout(3000)});if(!response.ok)throw Error('Status HTTP '+response.status);const s=await response.json(),op=s.operator||{};renderStatus(s);document.getElementById('state').textContent=JSON.stringify(s,null,2);document.getElementById('operator').textContent=op.connected?`地図: ${(op.selection||{}).map_file||'未選択'} / lane: ${(op.selection||{}).lane_id||'—'} / line: ${(op.selection||{}).line_type||'—'}`:'terminal 2（sim_bringup）未接続';document.getElementById('kartAuto').disabled=!op.ready;if(op.assets&&JSON.stringify(op.assets)!==assetsKey){assets=op.assets;assetsKey=JSON.stringify(assets);options('hdmap',assets.hdmaps||[]);options('models',assets.models||[]);fillMaps();fillLines()}trace(s.trajectory||{})}catch(e){disconnectedStatus();document.getElementById('notice').textContent=String(e)}finally{busy=false}},200)
</script>'''.encode('utf-8')

def png(pixels):
    height,width=pixels.shape[:2]
    def chunk(tag,data):
        return struct.pack('!I',len(data))+tag+data+struct.pack('!I',zlib.crc32(tag+data)&0xffffffff)
    payload=b''.join(b'\x00'+row.tobytes() for row in pixels)
    color=0 if pixels.ndim==2 else 2
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,color,0,0,0))+chunk(b'IDAT',zlib.compress(payload))+chunk(b'IEND',b'')

class SensorMonitor:
    def __init__(self,session=None):
        self.session=session
        self.images={}; self.metadata={}; self.lock=threading.Lock()
        self.bev_renderer=None;self.bev_error=''
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def proxy_studio(self):
                # Fixed loopback service; no user supplied URL or filesystem path.
                port=owner.session.operator.state.get('studio_port') if owner.session else None
                if not isinstance(port,int) or not 1<=port<=65535:
                    self.send_error(503,'Start terminal 2 sim_bringup with studio_enabled');return
                path=self.path[len('/studio'):] if self.path.startswith('/studio/') else self.path
                data=None
                if self.command=='POST':
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=4*1024*1024:self.send_error(400);return
                    data=self.rfile.read(length)
                req=Request(f'http://127.0.0.1:{port}'+path,data=data,method=self.command,
                            headers={'Content-Type':'application/json','Origin':f'http://127.0.0.1:{port}'})
                try:
                    try:response=urlopen(req,timeout=15)
                    except HTTPError as exc:response=exc
                    with response:
                        body=response.read();kind=response.headers.get('Content-Type','application/json')
                        if kind.startswith('text/html'):body=body.replace(b'href="/"',b'href="/studio/"')
                        self.send_response(response.code);self.send_header('Content-Type',kind)
                        # Served through this origin: allow only this origin's parent monitor.
                        csp=response.headers.get('Content-Security-Policy')
                        if csp:self.send_header('Content-Security-Policy',csp.replace("frame-ancestors 'none'","frame-ancestors 'self'"))
                        self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
                except (URLError,OSError):self.send_error(503,'Map Studio unavailable; see terminal 2')

            def do_POST(self):
                # Browser requests must come from this exact loopback origin.
                origin=self.headers.get('Origin')
                if origin and origin != 'http://'+self.headers.get('Host',''):
                    self.send_error(403); return
                if self.path.startswith('/api/'):
                    self.proxy_studio();return
                try:
                    if self.path != '/command' or owner.session is None: raise ValueError('Controls unavailable')
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0 < length <= 8192: raise ValueError('Invalid request size')
                    request=json.loads(self.rfile.read(length))
                    owner.session.request(request['action'],request.get('value'))
                    code,message=202,'Request queued; check state for result'
                except (ValueError,KeyError,TypeError,Full) as error:
                    code,message=400,str(error)
                data=json.dumps({'message':message}).encode()
                self.send_response(code); self.send_header('Content-Type','application/json'); self.end_headers(); self.wfile.write(data)
            def do_GET(self):
                path=self.path.split('?')[0]
                if path.startswith(('/studio/','/api/')):
                    self.proxy_studio();return
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

    def update(self,sim,extra=None):
        images={name:png(frame.values['image']) for name,frame in sim.sensors.latest.items()}
        if self.session and self.session.bev_enabled:
            try:
                import mujoco
                import math
                if self.bev_renderer is None:self.bev_renderer=mujoco.Renderer(sim.model,height=480,width=640)
                camera=mujoco.MjvCamera();mujoco.mjv_defaultCamera(camera)
                a,b,c,d=sim.map['room']['bounds']
                camera.lookat[:]=[(a+c)/2,(b+d)/2,0]
                camera.distance=max((c-a)*.75,d-b)/(2*math.tan(math.radians(float(sim.model.vis.global_.fovy))/2))*1.12
                camera.elevation=-90;camera.azimuth=90
                option=mujoco.MjvOption();option.geomgroup[4]=0;option.geomgroup[5]=1
                self.bev_renderer.update_scene(sim.data,camera=camera,scene_option=option)
                # Visual marker only: no model geometry or sensor/physics changes.
                import numpy as np
                position,quat,_,_=sim.state();w,x,y,z=quat
                yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
                start=np.array([position[0],position[1],.24])
                end=start+np.array([.45*math.cos(yaw),.45*math.sin(yaw),0])
                scene=self.bev_renderer.scene
                if scene.ngeom<scene.maxgeom:
                    geom=scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_ARROW,np.zeros(3),start,np.eye(3).flatten(),np.array([1,.85,0,1]))
                    mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_ARROW,.04,start,end)
                    scene.ngeom+=1
                images['bev']=png(self.bev_renderer.render().copy());self.bev_error=''
            except Exception as exc:self.bev_error=str(exc)
        pos,quat,linear,angular=sim.state()
        metadata={'sim_time':float(sim.data.time),'position_m':pos.tolist(),'orientation_wxyz':quat.tolist(),
            'body_velocity_mps':linear.tolist(),'body_angular_velocity_rad_s':angular.tolist(),
            'bev_error':self.bev_error,'wall_contact':getattr(sim,'last_wall_contact',None),
            'image_stamps':{name:frame.time for name,frame in sim.sensors.latest.items()}}
        if sim.imu_enabled:
            imu=sim.imu_state()
            metadata['imu']={key:value.tolist() if hasattr(value,'tolist') else value for key,value in imu.items()}
        if self.session: metadata.update(self.session.status())
        if extra:metadata.update(extra)
        with self.lock: self.images=images; self.metadata=metadata

    def close(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(timeout=1)
        if self.bev_renderer is not None:self.bev_renderer.close();self.bev_renderer=None
