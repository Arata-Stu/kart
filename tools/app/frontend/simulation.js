import { $, state, action, on, remember, emit, toast, api } from "./api.js";
import { reverseLane } from "./direction.js";
import { replaySimulation } from "./simulation_replay.js";
let requestSerial=0, parkingRoute=null, documentRevision=null, parkingMessage="";
let sim=null, snapshot=null, trail=[], running=false, last=0, accumulator=0, raf=0;
const dialog=$("simulation-dialog"), canvas=$("simulation-canvas"), ctx=canvas.getContext("2d");
function pause() {running=false; cancelAnimationFrame(raf); raf=0; $("simulation-play").textContent="再生";}
async function reset() {
  pause();parkingRoute=null;parkingMessage="";
  const serial=++requestSerial;sim=null;
  const mpc=$("simulation-controller").value==="mpc";
  $("simulation-lookahead").hidden=mpc;
  for(const input of dialog.querySelectorAll("[data-mpc]")) input.parentElement.hidden=!mpc;
  for (const input of dialog.querySelectorAll("input")) if (!input.checkValidity()) throw Error("試走設定の範囲を確認してください");
  const kind=$("simulation-line").value;
  const cfg={
    controller:$("simulation-controller").value,
    horizon:Number($("simulation-horizon").value),tracking:Number($("simulation-tracking").value),
    smoothness:Number($("simulation-smoothness").value),steeringRate:Number($("simulation-rate").value),
    speed:Number($("simulation-speed").value),
    wheelbase:Number($("simulation-wheelbase").value), maxSteer:Number($("simulation-steer").value)*Math.PI/180,
  };
  $("simulation-metrics").textContent="実車と共通の制御計算で試走を計算中…（初回はコンパイル）";
  $("simulation-play").disabled=true;
  try {
    const result=await api("tracking-preview",{line:snapshot.lines[kind],closed:snapshot.closed,options:cfg});
    if(serial!==requestSerial || !dialog.open)return;
    sim=replaySimulation(result,cfg);trail=[[sim.car.x,sim.car.y]];draw();
  } catch(e) {if(serial===requestSerial)$("simulation-metrics").textContent=e.message;throw e;} finally {if(serial===requestSerial)$("simulation-play").disabled=!sim;}

}
function draw() {
  if (!sim) return;
  const w=canvas.clientWidth, h=canvas.clientHeight, dpr=devicePixelRatio||1;
  canvas.width=w*dpr; canvas.height=h*dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.fillStyle="#f4f6f8"; ctx.fillRect(0,0,w,h);
  const line=parkingRoute || snapshot.lines[$("simulation-line").value];
  const diagnostic=sim.car.done?sim.diagnostic:null;
  const pts=[...snapshot.left,...snapshot.right,...line.points,...(diagnostic?.body||[])];
  let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
  for(const [x,y] of pts){x0=Math.min(x0,x);x1=Math.max(x1,x);y0=Math.min(y0,y);y1=Math.max(y1,y);}
  const scale=Math.min((w-60)/Math.max(1,x1-x0),(h-60)/Math.max(1,y1-y0));
  const project=([x,y])=>[w/2+(x-(x0+x1)/2)*scale,h/2-(y-(y0+y1)/2)*scale];
  function path(points,color,closed=false,fill=false){
    if(!points.length)return; ctx.beginPath(); points.forEach((p,i)=>{const [x,y]=project(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});
    if(closed)ctx.closePath();ctx.strokeStyle=color;ctx.lineWidth=1.8;ctx.stroke();if(fill){ctx.fillStyle=color+"33";ctx.fill();}
  }
  path(snapshot.left,"#328ccb",snapshot.closed);path(snapshot.right,"#e78f64",snapshot.closed);
  for(const o of snapshot.obstacles)path(o.polygon,"#ce4141",true,true);
  for(const a of snapshot.parking_areas||[]){path(a.polygon,'#238457',true,true);path(a.entrance,'#00bddd');}
  path(line.points,"#339675",parkingRoute?false:snapshot.closed);path(trail,"#9763bb");
  const c=sim.car, p=line.settings||{}, length=p.vehicle_length||.35,width=p.vehicle_width||.19,rear=p.rear_axle_to_rear??.1065;
  const [x,y]=project([c.x,c.y]);ctx.save();ctx.translate(x,y);ctx.rotate(-c.yaw);ctx.fillStyle="#263e57";
  ctx.fillRect(-rear*scale,-width/2*scale,length*scale,width*scale);
  ctx.strokeStyle="#f09e38";ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(Math.max(14,(length-rear)*scale),0);ctx.stroke();ctx.restore();
  const [tx,ty]=project([c.target.x,c.target.y]);
  ctx.save();ctx.strokeStyle="#dd3877";ctx.lineWidth=1.5;ctx.setLineDash([5,4]);
  ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(tx,ty);ctx.stroke();ctx.setLineDash([]);
  ctx.beginPath();ctx.arc(tx,ty,6,0,Math.PI*2);ctx.fillStyle="#fff";ctx.fill();ctx.lineWidth=2;ctx.stroke();
  ctx.fillStyle="#b51f5a";ctx.font="12px sans-serif";ctx.fillText("先読み点",tx+10,ty-10);ctx.restore();
  if(diagnostic){
    function geometry(g,color,fill=false){
      if(!g)return;
      if(g.type==='GeometryCollection'){for(const child of g.geometries)geometry(child,color,fill);return;}
      if(g.type==='Polygon'){
        ctx.beginPath();for(const ring of g.coordinates){ring.forEach((v,i)=>{const p=project(v);i?ctx.lineTo(...p):ctx.moveTo(...p);});ctx.closePath();}
        ctx.strokeStyle=color;ctx.lineWidth=2;ctx.stroke();if(fill){ctx.fillStyle=color+'44';ctx.fill('evenodd');}
      } else if(g.type==='MultiPolygon')for(const coordinates of g.coordinates)geometry({type:'Polygon',coordinates},color,fill);
      else if(g.type==='LineString')path(g.coordinates,color);
      else if(g.type==='MultiLineString')for(const coordinates of g.coordinates)path(coordinates,color);
    }
    geometry(diagnostic.surface,'#278ca0',true);
    geometry(diagnostic.clearance,'#ed9b15');
    path(diagnostic.body,'#d52943',true);
    for(const hit of diagnostic.hits){
      geometry(hit.geometry,'#d52943');geometry(hit.outside,'#d52943',true);path(hit.closest,'#d52943');
      const [hx,hy]=project(hit.marker);ctx.beginPath();ctx.arc(hx,hy,8,0,Math.PI*2);ctx.strokeStyle='#d52943';ctx.lineWidth=3;ctx.stroke();
      ctx.font='12px sans-serif';ctx.fillStyle='#a51c30';ctx.fillText(`${hit.kind}（距離 ${hit.distance_m.toFixed(3)} m）`,Math.min(w-230,Math.max(8,hx+12)),Math.max(20,hy-12));
    }
  }
  $("simulation-metrics").textContent=`${parkingRoute?"駐車試走（理想車両モデル） · ":""}${diagnostic?`水色:走行可能領域 / 赤:車体・抵触箇所 / 橙:余裕 ${diagnostic.margin_m.toFixed(3)} m${diagnostic.hits.length?"":" / 壁・領域の抵触なし"} · `:""}${parkingMessage || c.reason || (running?"試走中":"停止中")} · ${c.time.toFixed(1)} s · ${c.speed.toFixed(2)} m/s${sim.cfg.controller==="pure_pursuit"?` · 先読み ${c.lookahead.toFixed(2)} m`:""} · 目標 ${c.targetSpeed.toFixed(2)} m/s · 計算 ${c.solverMs.toFixed(2)} ms · 舵角 ${(c.steering*180/Math.PI).toFixed(1)}° · 誤差 ${c.error.toFixed(2)} m / 最大 ${c.maxError.toFixed(2)} m`;
}
function play(){
  if(!sim || sim.car.done)return;
  running=true;last=performance.now();accumulator=0;
  $("simulation-play").textContent="一時停止";raf=requestAnimationFrame(tick);
}
function tick(now){
  if(!running)return;
  accumulator+=Math.min((now-last)/1000,.1);last=now;
  while(accumulator>=.02&&!sim.car.done){sim.step(.02);accumulator-=.02;trail.push([sim.car.x,sim.car.y]);}
  if(sim.car.done)pause();draw();if(running)raf=requestAnimationFrame(tick);
}
action("simulation-open",async()=>{
  if(!state.lane || state.doc.snapshot_status==="pending")throw Error("編集可能な地図を選んでください");
  const kinds=["centerline","raceline","customline"].filter(k=>state.lane.lines[k]?.points.length);
  if(!kinds.length)throw Error("先にラインを生成してください");
  documentRevision=state.doc.revision;
  snapshot=structuredClone({...state.lane,obstacles:state.doc.obstacles||[],parking_areas:state.doc.parking_areas||[]});
  $("simulation-speed").value = state.lane.centerline_speed_mps ?? 1.5;
  $("simulation-title").textContent=`試走 · ${state.laneId}`;
  $("simulation-line").replaceChildren(...kinds.map(k=>new Option(k,k)));
  dialog.showModal();await reset();
});
action("simulation-play",async()=>{if(!sim)await reset();if(running){pause();draw();return;}if(!sim)return;if(sim.car.done){if(parkingRoute){toast(sim.car.reason+"。通常試走へ戻るにはリセットしてください。");return;}await reset();}parkingMessage="";play();draw();});
action("simulation-reset",reset);
for(const id of ["simulation-controller","simulation-horizon","simulation-tracking","simulation-smoothness","simulation-rate","simulation-line","simulation-speed","simulation-wheelbase","simulation-steer"]){
  $(id).addEventListener("change",async()=>{try{await reset();}catch(e){pause();sim=null;$("simulation-metrics").textContent=e.message;toast(e.message,true);}});
}
dialog.addEventListener("close",()=>{requestSerial++;pause();});
new ResizeObserver(()=>{if(dialog.open)draw();}).observe(canvas);
action("lane-reverse",()=>{
  if(!state.lane || state.doc.snapshot_status==="pending")throw Error("編集可能な地図を選んでください");
  remember();reverseLane(state.lane);state.selected=-1;state.dirty=true;emit("document");
  toast("進行方向を反転しました。各ラインを再生成してください（元に戻す操作も可能）");
});
on("document",()=>{if(dialog.open)dialog.close();});

action('simulation-parking',async()=>{
  if(!sim || state.dirty || state.doc.revision!==documentRevision)throw Error('地図を保存し、試走を開き直してください');
  pause();parkingMessage="駐車経路と追従を計算中…";draw();const serial=++requestSerial;
  $('simulation-parking').disabled=true;$('simulation-play').disabled=true;
  try {
    const result=await api('parking-preview',{id:state.id,revision:documentRevision,lane_id:state.laneId,line_type:$('simulation-line').value,slot:$('simulation-parking-slot').value,pose:[sim.car.x,sim.car.y,sim.car.yaw],options:sim.cfg,initial_speed:sim.car.speed});
    if(serial!==requestSerial || !dialog.open)return;
    parkingRoute={points:result.route.map(p=>p.slice(0,2))};
    sim=replaySimulation(result,sim.cfg);trail=[[sim.car.x,sim.car.y]];parkingMessage="";
    play();draw();
    toast(sim.car.done?sim.car.reason:'駐車試走を開始しました。',sim.car.done && result.reason!=='simulated_parked');
  } catch(e) {
    if(serial===requestSerial && dialog.open){parkingMessage='駐車試走を開始できません: '+e.message;draw();}
    throw e;
  } finally {if(serial===requestSerial)$('simulation-play').disabled=!sim;$('simulation-parking').disabled=false;}
});
