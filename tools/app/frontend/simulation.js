import { $, state, action, on, remember, emit, toast } from "./api.js";
import { reverseLane } from "./direction.js";
import { createSimulation } from "./simulation_model.js";
let sim=null, snapshot=null, trail=[], running=false, last=0, accumulator=0, raf=0;
const dialog=$("simulation-dialog"), canvas=$("simulation-canvas"), ctx=canvas.getContext("2d");
function pause() {running=false; cancelAnimationFrame(raf); raf=0; $("simulation-play").textContent="再生";}
function reset() {
  pause();
  for (const input of dialog.querySelectorAll("input")) if (!input.checkValidity()) throw Error("試走設定の範囲を確認してください");
  const kind=$("simulation-line").value;
  sim=createSimulation(snapshot.lines[kind], snapshot.closed, {
    speed:Number($("simulation-speed").value), lookahead:Number($("simulation-lookahead").value),
    wheelbase:Number($("simulation-wheelbase").value), maxSteer:Number($("simulation-steer").value)*Math.PI/180,
  });
  trail=[[sim.car.x,sim.car.y]]; draw();
}
function draw() {
  if (!sim) return;
  const w=canvas.clientWidth, h=canvas.clientHeight, dpr=devicePixelRatio||1;
  canvas.width=w*dpr; canvas.height=h*dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.fillStyle="#f4f6f8"; ctx.fillRect(0,0,w,h);
  const line=snapshot.lines[$("simulation-line").value];
  const pts=[...snapshot.left,...snapshot.right,...line.points];
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
  path(line.points,"#339675",snapshot.closed);path(trail,"#9763bb");
  const c=sim.car, p=line.settings||{}, length=p.vehicle_length||.47,width=p.vehicle_width||.19,rear=p.rear_axle_to_rear??.1065;
  const [x,y]=project([c.x,c.y]);ctx.save();ctx.translate(x,y);ctx.rotate(-c.yaw);ctx.fillStyle="#263e57";
  ctx.fillRect(-rear*scale,-width/2*scale,length*scale,width*scale);
  ctx.strokeStyle="#f09e38";ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(Math.max(14,(length-rear)*scale),0);ctx.stroke();ctx.restore();
  $("simulation-metrics").textContent=`${c.reason || (running?"試走中":"停止中")} · ${c.time.toFixed(1)} s · ${c.speed.toFixed(2)} m/s · 舵角 ${(c.steering*180/Math.PI).toFixed(1)}° · 誤差 ${c.error.toFixed(2)} m / 最大 ${c.maxError.toFixed(2)} m`;
}
function tick(now){
  if(!running)return;
  accumulator+=Math.min((now-last)/1000,.1);last=now;
  while(accumulator>=.02&&!sim.car.done){sim.step(.02);accumulator-=.02;trail.push([sim.car.x,sim.car.y]);}
  if(sim.car.done)pause();draw();if(running)raf=requestAnimationFrame(tick);
}
action("simulation-open",()=>{
  if(!state.lane || state.doc.snapshot_status==="pending")throw Error("編集可能な地図を選んでください");
  const kinds=["centerline","raceline","customline"].filter(k=>state.lane.lines[k]?.points.length);
  if(!kinds.length)throw Error("先にラインを生成してください");
  snapshot=structuredClone({...state.lane,obstacles:state.doc.obstacles||[]});
  $("simulation-title").textContent=`試走 · ${state.laneId}`;
  $("simulation-line").replaceChildren(...kinds.map(k=>new Option(k,k)));
  dialog.showModal();reset();
});
action("simulation-play",()=>{if(!sim)reset();if(running){pause();draw();return;}if(sim.car.done)reset();running=true;last=performance.now();accumulator=0;$("simulation-play").textContent="一時停止";raf=requestAnimationFrame(tick);});
action("simulation-reset",reset);
for(const id of ["simulation-line","simulation-speed","simulation-lookahead","simulation-wheelbase","simulation-steer"]){
  $(id).addEventListener("change",()=>{try{reset();}catch(e){pause();sim=null;$("simulation-metrics").textContent=e.message;toast(e.message,true);}});
}
dialog.addEventListener("close",pause);
new ResizeObserver(()=>{if(dialog.open)draw();}).observe(canvas);
action("lane-reverse",()=>{
  if(!state.lane || state.doc.snapshot_status==="pending")throw Error("編集可能な地図を選んでください");
  remember();reverseLane(state.lane);state.selected=-1;state.dirty=true;emit("document");
  toast("進行方向を反転しました。各ラインを再生成してください（元に戻す操作も可能）");
});
on("document",()=>{if(dialog.open)dialog.close();});
