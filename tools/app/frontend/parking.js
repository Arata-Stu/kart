import { $, state, action, on, emit, api, remember, toast } from './api.js';
import { parsePoints, pointText, parkingWarnings } from './parking_model.js';
import { nearestEdge, moveVertex } from './parking_visual.js';
const fields = ['polygon','entrance','maneuver','drivable'];
let mapId = null, picking = null;
function row() { return state.doc?.parking_areas?.find(a => a.id === $('parking-id').value); }
function load() {
  picking = state.parkingPick = null;
  const a = row();
  for (const f of fields) $('parking-'+f).value = pointText(f === 'drivable' ? state.doc?.parking_drivable : a?.[f]);
  for (const [i,f] of ['x','y','yaw'].entries()) $('parking-goal-'+f).value = a?.goal_pose?.[i] ?? '';
  $('parking-pick-note').textContent = '1から順に地図で指定します。丸い点はドラッグで修正。右ドラッグで移動、ホイールで拡大できます。';
  state.parkingDraft = null;
  if($('parking-editor').open) preview();
  emit('view');
}
function status() {
  const disabled = !state.doc || state.doc.snapshot_status === 'pending';
  for (const e of $('parking-editor').querySelectorAll('button,input,textarea,select')) e.disabled = disabled;
  $('parking-warning').textContent = parkingWarnings(state.doc);
  $('parking-defined').textContent = (state.doc?.parking_areas || []).map(a => a.id).join('、') || '未定義';
  if (state.id !== mapId) { mapId = state.id; load(); }
}
function preview() {
  try {
    state.parkingDraft = { id: $('parking-id').value,
      polygon:parsePoints($('parking-polygon').value), entrance:parsePoints($('parking-entrance').value),
      maneuver:parsePoints($('parking-maneuver').value) };
    const goal=['x','y','yaw'].map(f => $('parking-goal-'+f).value);
    if (goal.every(v => v.trim() !== '' && Number.isFinite(Number(v)))) state.parkingDraft.goal_pose=goal.map(Number);
    const a=state.parkingDraft;
    $('parking-progress').textContent=[['枠',a.polygon.length>=3],['入口',a.entrance.length===2],['停車姿勢',!!a.goal_pose]].map(([name,ok])=>`${ok?'✓':'未指定'} ${name}`).join(' · ')+'（形状の検証は区画を適用時）';
    emit('view');
  } catch { /* Incomplete text is not committed to the map. */ }
}
$('parking-id').addEventListener('change',load);
$('parking-editor').addEventListener('toggle',()=>{
  if($('parking-editor').open) preview();
  else {picking=state.parkingPick=null;state.parkingDraft=null;emit('view');}
});
for(const f of ['polygon','maneuver']) action('parking-clear-'+f,()=>{
  $('parking-'+f).value='';if(f==='polygon')$('parking-entrance').value='';
  picking=state.parkingPick=f==='maneuver'?null:f;preview();
  $('parking-pick-note').textContent=f==='maneuver'?'経路制限を解除しました。通常コース＋駐車枠で計画します。区画を適用して保存してください。':'角を順番にクリックしてください。3点以上で範囲になります。';
});
on('parking-drag',e=>{
  const {handle,p}=e.detail;
  if(handle.field==='goal'){$('parking-goal-x').value=p[0];$('parking-goal-y').value=p[1];}
  else if(handle.field==='heading')$('parking-goal-yaw').value=Math.atan2(p[1]-Number($('parking-goal-y').value),p[0]-Number($('parking-goal-x').value));
  else {
    const points=parsePoints($('parking-'+handle.field).value);
    if(handle.field==='polygon') {
      const result=moveVertex(points,parsePoints($('parking-entrance').value),handle.index,p);
      $('parking-polygon').value=pointText(result.points);$('parking-entrance').value=pointText(result.entrance);
    } else {points[handle.index]=p;$('parking-'+handle.field).value=pointText(points);}
  }
  preview();
});
for (const e of $('parking-editor').querySelectorAll('textarea,input')) e.addEventListener('input',preview);
for (const field of [...fields,'goal','heading']) action('parking-pick-'+field, () => {
  picking = state.parkingPick = field;preview();
  $('parking-pick-note').textContent = field === 'heading' ? 'goalから向けたい方向を地図上でクリック。' : field === 'entrance' ? '水色の入口にしたい枠の辺をクリックしてください。' : '地図をクリックして入力。頂点は順番に追加します。「入力終了」で終了。';
});
function finish(){picking=state.parkingPick=null;$('parking-pick-note').textContent='地図入力を終了しました。丸い点はドラッグで修正できます。区画を適用後、地図を保存してください。';emit('view');}
action('parking-pick-end',finish);on('parking-end',finish);
on('parking-click', e => {
  if (!picking || !state.doc) return;
  const p=e.detail.map(v => Number(v.toFixed(4)));
  if(picking==='entrance'){
    const pts=parsePoints($('parking-polygon').value);
    const edge=nearestEdge(pts,p,state.parkingTolerance??.2);
    if(pts.length<3 || edge===null){toast('先に枠を3点以上で描き、その辺の近くをクリックしてください',true);return;}
    $('parking-entrance').value=pointText([pts[edge],pts[(edge+1)%pts.length]]);
    finish();preview();return;
  }
  if (fields.includes(picking)) {
    let rows=parsePoints($('parking-'+picking).value);
    if (picking === 'entrance' && rows.length >= 2) rows=[];
    rows.push(p); $('parking-'+picking).value=pointText(rows);
    if (picking === 'entrance' && rows.length === 2) picking=state.parkingPick=null;
  } else if (picking === 'goal') {
    $('parking-goal-x').value=p[0]; $('parking-goal-y').value=p[1]; if($('parking-goal-yaw').value==='')$('parking-goal-yaw').value=0; picking=state.parkingPick='heading';
  } else if (picking === 'heading') {
    const x=$('parking-goal-x').value,y=$('parking-goal-y').value;
    if (x === '' || y === '') throw Error('先にgoal位置を指定してください');
    if (Math.hypot(p[0]-Number(x),p[1]-Number(y))<1e-6) throw Error('goalと異なる点で向きを指定してください');
    $('parking-goal-yaw').value=Math.atan2(p[1]-Number(y),p[0]-Number(x)).toFixed(6);
    picking=state.parkingPick=null;
  }
  if (!picking) $('parking-pick-note').textContent='指定しました。区画を適用後、地図を保存してください。';
  if(picking==='heading')$('parking-pick-note').textContent='次に車の前方をクリックしてください。';
  preview();
});
action('parking-apply', async () => {
  const id=state.id, revision=state.doc.revision;
  const candidate=structuredClone(state.doc);
  const goal=['x','y','yaw'].map(f => $('parking-goal-'+f).value);
  if (goal.some(v => v.trim()==='')) throw Error('手順3で停車位置と向きを指定してください');
  const area={id:$('parking-id').value,goal_pose:goal.map(Number)};
  for (const f of ['polygon','entrance','maneuver']) area[f]=parsePoints($('parking-'+f).value);
  candidate.parking_areas=[...(candidate.parking_areas || []).filter(a => a.id!==area.id),area];
  candidate.parking_drivable=parsePoints($('parking-drivable').value);
  if(area.polygon.length<3 || (area.maneuver.length>0 && area.maneuver.length<3))throw Error('駐車枠と、指定する場合の経路制限範囲は3点以上で描いてください');
  const fields=await api('parking-validate',{id,document:candidate});
  if (state.id!==id || state.doc.revision!==revision) throw Error('地図が変わりました。再入力してください');
  remember(); Object.assign(state.doc,fields); state.dirty=true;
  finish(); preview(); emit('document'); emit('view');
  toast('駐車区画を適用しました。「保存」で地図へ保存します。');
});
action('parking-reload',load);
action('parking-remove', () => {
  if (!row() || !confirm($('parking-id').value+'の駐車定義を削除しますか？')) return;
  remember(); state.doc.parking_areas=state.doc.parking_areas.filter(a => a.id!==$('parking-id').value);
  state.dirty=true; load(); emit('document');
});
on('document',status); on('status',status);

on("parking-reset-editor", load);
