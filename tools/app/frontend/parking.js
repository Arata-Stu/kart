import { $, state, action, on, emit, api, remember, toast } from './api.js';
import { parsePoints, pointText, parkingWarnings } from './parking_model.js';
const fields = ['polygon','entrance','maneuver','drivable'];
let mapId = null, picking = null;
function row() { return state.doc?.parking_areas?.find(a => a.id === $('parking-id').value); }
function load() {
  picking = state.parkingPick = null; directionStart = null;
  const a = row();
  for (const f of fields) $('parking-'+f).value = pointText(f === 'drivable' ? state.doc?.parking_drivable : a?.[f]);
  for (const [i,f] of ['x','y','yaw'].entries()) $('parking-goal-'+f).value = a?.goal_pose?.[i] ?? '';
  $('parking-forward').value = a?.forward_yaw ?? '';
  $('parking-pick-note').textContent = '座標はmapのm、向きはrad。入口は区画の同じ辺上の2点です。';
  state.parkingDraft = null;
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
    emit('view');
  } catch { /* Incomplete text is not committed to the map. */ }
}
$('parking-id').addEventListener('change',load);
for (const e of $('parking-editor').querySelectorAll('textarea,input')) e.addEventListener('input',preview);
for (const field of [...fields,'goal','heading','forward']) action('parking-pick-'+field, () => {
  picking = state.parkingPick = field; directionStart = null;
  $('parking-pick-note').textContent = field === 'heading' ? 'goalから向けたい方向を地図上でクリック。' : field === 'forward' ? 'コース順方向に2点クリック。' : '地図をクリックして入力。頂点は順番に追加します。「入力終了」で終了。';
});
action('parking-pick-end', () => { picking = state.parkingPick = null; $('parking-pick-note').textContent = '地図入力を終了しました。区画を適用後、地図を保存してください。'; });
let directionStart = null;
on('parking-click', e => {
  if (!picking || !state.doc) return;
  const p=e.detail.map(v => Number(v.toFixed(4)));
  if (fields.includes(picking)) {
    let rows=parsePoints($('parking-'+picking).value);
    if (picking === 'entrance' && rows.length >= 2) rows=[];
    rows.push(p); $('parking-'+picking).value=pointText(rows);
    if (picking === 'entrance' && rows.length === 2) picking=state.parkingPick=null;
  } else if (picking === 'goal') {
    $('parking-goal-x').value=p[0]; $('parking-goal-y').value=p[1]; picking=state.parkingPick=null;
  } else if (picking === 'heading') {
    const x=$('parking-goal-x').value,y=$('parking-goal-y').value;
    if (x === '' || y === '') throw Error('先にgoal位置を指定してください');
    if (Math.hypot(p[0]-Number(x),p[1]-Number(y))<1e-6) throw Error('goalと異なる点で向きを指定してください');
    $('parking-goal-yaw').value=Math.atan2(p[1]-Number(y),p[0]-Number(x)).toFixed(6);
    picking=state.parkingPick=null;
  } else if (picking === 'forward') {
    if (!directionStart) directionStart=p;
    else { if (Math.hypot(p[0]-directionStart[0],p[1]-directionStart[1])<1e-6) throw Error('進行方向は異なる2点で指定してください'); $('parking-forward').value=Math.atan2(p[1]-directionStart[1],p[0]-directionStart[0]).toFixed(6); directionStart=null; picking=state.parkingPick=null; }
  }
  if (!picking) $('parking-pick-note').textContent='指定しました。区画を適用後、地図を保存してください。';
  preview();
});
action('parking-apply', async () => {
  const id=state.id, revision=state.doc.revision;
  const candidate=structuredClone(state.doc);
  const goal=['x','y','yaw'].map(f => $('parking-goal-'+f).value);
  if (goal.some(v => v.trim()==='') || $('parking-forward').value.trim()==='') throw Error('goalのx・y・yawとコース進行方向を入力してください');
  const area={id:$('parking-id').value,goal_pose:goal.map(Number),forward_yaw:Number($('parking-forward').value)};
  for (const f of ['polygon','entrance','maneuver']) area[f]=parsePoints($('parking-'+f).value);
  candidate.parking_areas=[...(candidate.parking_areas || []).filter(a => a.id!==area.id),area];
  candidate.parking_drivable=parsePoints($('parking-drivable').value);
  const fields=await api('parking-validate',{id,document:candidate});
  if (state.id!==id || state.doc.revision!==revision) throw Error('地図が変わりました。再入力してください');
  remember(); Object.assign(state.doc,fields); state.dirty=true;
  state.parkingDraft=null; picking=state.parkingPick=null; emit('document'); emit('view');
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
