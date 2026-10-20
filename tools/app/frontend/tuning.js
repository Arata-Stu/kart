import { renderVehicle } from './vehicle_status.js';
import { $, state, api, action, on, emit, toast } from './api.js';

export const tuning = {connected:false, busy:false, epoch:0, received:0, status:null,
  active:null, preview:null, trail:[], timer:null, id:null, error:''};
const fresh = () => tuning.connected && Date.now() - tuning.received < 3000;
function invalidatePreview() { tuning.preview=null; render(); }
function stop() {
  tuning.epoch++;
  tuning.connected=false; tuning.received=0;
  tuning.status=null; tuning.active=null; tuning.preview=null; tuning.trail=[];
  clearTimeout(tuning.timer); tuning.timer=null;
  render(); emit('tuning');
}
export function parameterConnectionReady() { return fresh() && tuning.id===state.id; }
export function parameterRequest(action, extra={}) { return request(action,extra); }
function request(action, extra={}) {
  return api('tuning/'+action, {id:tuning.id, connection:state.config.connection,
    known_revision:tuning.status?.revision, ...extra});
}
function update(result) {
  if(result.revision !== tuning.status?.revision) tuning.active=result.active || null;
  tuning.status=result; tuning.received=Date.now(); tuning.error='';
  if(result.pose && result.localized) {
    tuning.trail.push([result.pose.x,result.pose.y]);
    if(tuning.trail.length>1200) tuning.trail.shift();
  }
  render(); emit('tuning');
}
async function poll(epoch) {
  if(epoch!==tuning.epoch || !tuning.connected) return;
  // Leaving the map page disconnects display only; driving remains independent.
  if($('map-page').hidden || state.id!==tuning.id) { stop(); return; }
  try {
    const result=await request('status');
    if(epoch!==tuning.epoch)return;
    update(result);
  } catch(error) {
    if(epoch!==tuning.epoch)return;
    tuning.error=error.message; stop(); return;
  }
  if(epoch===tuning.epoch)tuning.timer=setTimeout(()=>poll(epoch),1000);
}
function applyIssue() {
  if(!fresh())return '自己位置接続を確認してください';
  return tuning.status?.apply_issue || '';
}
function previewValid() {
  const p=tuning.preview;
  return p && !state.dirty && p.document.revision===state.doc?.revision &&
    p.lane_id===state.laneId && p.line_type===$('tuning-line').value &&
    p.centerline_speed_mps===Number($('tuning-speed').value);
}
function render() {
  const s=tuning.status, target=state.config?.connection;
  $('tuning-target').textContent=target ? `${target.user}@${target.host}:${target.port} · 接続設定は「転送」と共通` : '';
  $('tuning-connect').textContent=tuning.connected?'切断':'自己位置を接続';
  $('tuning-connect').disabled=tuning.busy || !state.doc;
  renderVehicle(fresh() ? s : null);
  $('tuning-offset-open').disabled=tuning.busy || !parameterConnectionReady();
  $('tuning-parameters-open').disabled=tuning.busy || !parameterConnectionReady();
  $('tuning-line-switch').disabled=tuning.busy || !!applyIssue();
  $('tuning-preview').disabled=tuning.busy || !state.lane || state.dirty;
  $('tuning-apply').disabled=tuning.busy || !!applyIssue() || !previewValid();
  $('tuning-rollback').disabled=tuning.busy || !!applyIssue() || !s?.can_rollback;
  const mode={1:'AUTO',2:'MANUAL',3:'STOP',4:'PROPO'}[s?.mode] || '不明';
  $('tuning-status').textContent=tuning.error || (fresh()
    ? `${mode} · ${s.localized?'自己位置確定':'自己位置未確定'} · ${s.speed_mps==null?'速度不明':s.speed_mps.toFixed(2)+' m/s'} · ${s.apply_issue || '適用可能'}`
    : '未接続・受信停止');
  $('tuning-revision').textContent=`実車 ${s?.revision?.slice(0,10) || '—'} / 候補 ${previewValid()?tuning.preview.revision.slice(0,10):'—'}`;
  const ref = fresh() ? s?.tracking_reference : null;
  $('tuning-lookahead').textContent=ref
    ? `Pure Pursuit: lookahead ${ref.lookahead_m.toFixed(2)} m（経路長） / 参照点まで ${ref.target_distance_m.toFixed(2)} m（直線）`
    : 'Pure Pursuit参照点: 未受信・無効・受信停止';

}
action('tuning-connect', async()=> {
  if(tuning.connected){stop();return;}
  tuning.id=state.id; tuning.busy=true; tuning.error='';
  const epoch=++tuning.epoch;
  render();
  try {
    const result=await request('status');
    if(epoch!==tuning.epoch)return;
    tuning.connected=true; update(result);
    tuning.timer=setTimeout(()=>poll(epoch),1000);
  } catch(error) { if(epoch===tuning.epoch)tuning.error=error.message; throw error; }
  finally {tuning.busy=false;render();}
});
action('tuning-preview', async()=> {
  if(state.dirty)throw new Error('編集を保存し、必要なラインを生成してください');
  const epoch=tuning.epoch, id=state.id, revision=state.doc.revision, lane=state.laneId;
  const kind=$('tuning-line').value, speed=Number($('tuning-speed').value);
  tuning.id=id; tuning.busy=true; tuning.preview=null; render();
  try {
    const result=await api('tuning/prepare',{id,document_revision:revision,lane_id:lane,
      line_type:kind,centerline_speed_mps:speed});
    if(epoch!==tuning.epoch || state.id!==id || state.doc.revision!==revision || state.dirty || state.laneId!==lane || $('tuning-line').value!==kind || Number($('tuning-speed').value)!==speed)
      throw new Error('編集内容が変わりました。プレビューを作り直してください');
    tuning.preview=result; emit('tuning');
  } finally {tuning.busy=false;render();}
});
for(const [id, actionName] of [['tuning-apply','apply'],['tuning-rollback','rollback']]) {
  action(id, async()=> {
    if(applyIssue())throw new Error(applyIssue());
    if(actionName==='apply' && !previewValid())throw new Error('候補を作り直してください');
    const epoch=tuning.epoch;
    tuning.busy=true; render();
    try {
      const result=await request(actionName,{snapshot:tuning.preview,expected_revision:tuning.status.revision});
      if(epoch!==tuning.epoch)return;
      update(result); toast('Jetsonの適用版を確認しました。再開は既存の操作系でAUTOへ切り替えてください');
    } catch(error) {
      if(epoch===tuning.epoch) {tuning.error=`適用結果を確認してください: ${error.message}`; tuning.received=0; emit('tuning');}
      throw error;
    } finally {tuning.busy=false;render();}
  });
}
$('tuning-line').addEventListener('change',invalidatePreview);
$('tuning-speed').addEventListener('input',invalidatePreview);
on('loaded',stop);
on('connection',stop);
on('document',()=> { if(tuning.preview && !previewValid())tuning.preview=null; render(); });
on('ready',render);
on('status',render);
setInterval(()=> { if(tuning.connected && !fresh()){render();emit('tuning');} },500);

export function paintTuning(ctx, project, line) {
  if(tuning.id!==state.id)return;
  const active=tuning.active;
  if(active) {
    const lane=active.document.lanes.find(l=>l.id===active.lane_id);
    if(lane?.lines[active.line_type])line(lane.lines[active.line_type].points,'#ffffff',lane.closed,false,4);
  }
  if(previewValid()) {
    const p=tuning.preview, lane=p.document.lanes.find(l=>l.id===p.lane_id);
    line(lane.lines[p.line_type].points,'#1599df',lane.closed,false,3);
  }
  const s=tuning.status;
  if(!fresh())return;
  const reference=s?.tracking_reference;
  if(reference) {
    line([reference.origin,reference.target],'#efb62d',false,false,2);
    const target=project(reference.target);
    ctx.save();
    ctx.beginPath();ctx.arc(target[0],target[1],7,0,Math.PI*2);
    ctx.fillStyle='#efb62d';ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();
    ctx.fillStyle='#745000';ctx.font='12px sans-serif';
    ctx.fillText(`参照点 · L=${reference.lookahead_m.toFixed(2)} m`,target[0]+12,target[1]-10);
    ctx.restore();
  }
  if(!s?.pose || !s.localized)return;
  line(tuning.trail,'#e78228',false,false,2);
  const p=project([s.pose.x,s.pose.y]), end=project([s.pose.x+Math.cos(s.pose.yaw),s.pose.y+Math.sin(s.pose.yaw)]);
  const yaw=Math.atan2(end[1]-p[1],end[0]-p[0]);
  ctx.save();ctx.translate(...p);ctx.rotate(yaw);
  ctx.beginPath();ctx.moveTo(17,0);ctx.lineTo(-9,-8);ctx.lineTo(-5,0);ctx.lineTo(-9,8);ctx.closePath();
  ctx.fillStyle='#e78228';ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();ctx.restore();
}
