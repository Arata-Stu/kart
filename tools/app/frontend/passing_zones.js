import { $,state,on,emit,action,remember,toast } from './api.js';
import { passingColors,passingSides,newZone,zonePath,zoneIndices,nearestWaypoint } from './passing_model.js';
import { lineVisible,showLine } from './line_visibility.js';

let binding='',editing=null;
const kind=()=>$('passing-line').value || 'centerline';
const current=()=>state.lane?.lines[kind()];
function context() {return `${state.id}/${state.laneId}/${kind()}/${current()?.geometry_revision || ''}`;}
function editable() {
  return !!state.lane && state.doc?.snapshot_status!=='pending' && !!current()
    && !state.jobs.some(job=>['queued','running'].includes(job.status) && (job.title.includes(state.id) || job.title.includes('生成')));
}
function index(id) {
  const input=$(id),value=Number(input.value);
  if(!String(input.value).trim() || !Number.isInteger(value))throw Error('WP番号は1からの整数で指定してください');
  return value-1;
}
function draftReset() {
  state.passingPick=null;editing=null;
  $('passing-start').value='1';$('passing-end').value=String(Math.min(2,current()?.points.length || 1));
  $('passing-side').value='both';
}
function changed() {
  state.dirty=true;state.diagnostic=null;emit('document');
}
function render() {
  const line=current(),ready=editable(),next=context();
  if(next!==binding) {binding=next;draftReset();}
  if(state.step!=='lines' || !line || state.target!==null || state.parkingPick)state.passingPick=null;
  for(const value of ['centerline','raceline','customline']) {
    const option=[...$('passing-line').options].find(option=>option.value===value);
    if(option)option.disabled=!state.lane?.lines[value];
  }
  $('passing-line').disabled=!state.lane;
  for(const id of ['passing-start','passing-end','passing-side','passing-pick-start','passing-pick-end','passing-add'])$(id).disabled=!ready;
  $('passing-add').textContent=editing===null ? '区間を追加' : '選択区間を更新';
  $('passing-cancel').disabled=!state.passingPick && editing===null;
  $('passing-start').max=$('passing-end').max=String(line?.points.length || 1);
  const zones=Array.isArray(line?.passing_zones) ? line.passing_zones : [];
  $('passing-clear').disabled=!ready || (!zones.length && !line?.passing_zones_error);
  const invalid=zones.filter(zone=>!zonePath(zone,line,state.lane?.closed)).length;
  const notices=[state.lane?.passing_zones_notice,line?.passing_zones_invalidated,
    line?.passing_zones_error ? '形状と許可が不一致です。失効した区間を削除して設定し直してください。' : '',
    invalid ? `${invalid}区間は形状不一致のため失効（色帯・実行には使いません）。削除して設定し直してください。` : ''].filter(Boolean);
  $('passing-notice').textContent=notices.join('\n');
  $('passing-summary').textContent=line ? `${line.points.length} WP · 有効 ${zones.length-invalid}区間 · ${line.geometry_revision?.slice(0,10) || '形状版未取得'}${state.dirty ? ' · 未保存' : ''}` : '対象ラインを生成してください。未設定は追越し許可なしです。';
  $('passing-pick-status').textContent=state.passingPick ? `地図上の${state.passingPick==='start' ? '開始' : '終了'}WP付近をクリック（Alt＋ドラッグで地図移動、Escで終了）` : '開始・終了WPを番号または地図で選択 → 区間を追加 → 地図を保存';
  const list=$('passing-list');list.replaceChildren();
  if(!zones.length){const empty=document.createElement('p');empty.className='help';empty.textContent='許可区間なし。追越しを開始できる区間は0です。';list.append(empty);}
  zones.forEach((raw,i)=>{
    const zone=raw && typeof raw==='object' ? raw : {};
    const row=document.createElement('div');row.className='passing-zone-row';
    const label=document.createElement('span');label.textContent=`${i+1} · WP ${zone.start_index+1} → ${zone.end_index+1} · ${passingSides[zone.side] || '不正な側'}${zonePath(zone,line,state.lane.closed) ? '' : ' · 失効'}`;
    label.style.borderColor=passingColors[zone.side] || '#83909b';
    const edit=document.createElement('button');edit.type='button';edit.textContent='編集';edit.disabled=!ready || !zonePath(zone,line,state.lane.closed);
    edit.addEventListener('click',()=>{editing=zone.id;state.passingPick=null;$('passing-start').value=String(zone.start_index+1);$('passing-end').value=String(zone.end_index+1);$('passing-side').value=zone.side;render();emit('view');});
    const remove=document.createElement('button');remove.type='button';remove.textContent='削除';remove.disabled=!ready;
    remove.setAttribute('aria-label',`許可区間 ${i+1} を削除`);
    remove.addEventListener('click',()=>{remember();line.passing_zones.splice(i,1);if(editing===zone.id)draftReset();changed();});
    row.append(label,edit,remove);list.append(row);
  });
}
action('passing-add',()=>{
  if(!editable())throw Error('生成済みラインを選択してください');
  const line=current(),start=index('passing-start'),end=index('passing-end');
  let zone;
  if(editing!==null) {
    zoneIndices(start,end,line.points.length,state.lane.closed);
    zone={id:editing,start_index:start,end_index:end,side:$('passing-side').value,geometry_revision:line.geometry_revision};
    if(!Object.hasOwn(passingSides,zone.side))throw Error('許可する側を選択してください');
  } else zone=newZone(line,state.lane.closed,start,end,$('passing-side').value);
  remember();
  line.passing_zones ||= [];
  if(editing!==null) {
    const position=line.passing_zones.findIndex(row=>row.id===editing);
    if(position<0)throw Error('編集対象の区間が変わりました');
    line.passing_zones[position]=zone;
  } else line.passing_zones.push(zone);
  delete line.passing_zones_invalidated;
  editing=null;state.passingPick=null;changed();
});
for(const end of ['start','end'])action(`passing-pick-${end}`,()=>{
  if(!editable())throw Error('生成済みラインを選択してください');
  if($('parking-editor').open)throw Error('駐車の編集を終了してからWPを選択してください');
  state.target=null;state.selected=-1;state.passingPick=end;render();emit('view');
  showLine(kind());
});
action('passing-cancel',()=>{draftReset();render();emit('view');});
action('passing-clear',()=>{
  if(!editable())throw Error('生成済みラインを選択してください');
  remember();current().passing_zones=[];delete current().passing_zones_error;draftReset();changed();
});
$('passing-line').addEventListener('change',()=>{binding='';showLine(kind());render();emit('view');});
$('passing-editor').addEventListener('toggle',()=>{if($('passing-editor').open && current())showLine(kind());emit('view');});
$('passing-show').addEventListener('change',()=>emit('view'));
for(const id of ['passing-start','passing-end','passing-side'])$(id).addEventListener('input',()=>emit('view'));
on('passing-click',event=>{
  if(!state.passingPick || !editable())return;
  const point=nearestWaypoint(current().points,event.detail.point,event.detail.tolerance);
  if(point<0)return toast('対象ラインのWP付近をクリックしてください',true);
  $(`passing-${state.passingPick}`).value=String(point+1);
  state.passingPick=state.passingPick==='start' ? 'end' : null;
  render();emit('view');
});
on('passing-end',()=>{state.passingPick=null;render();});
on('loaded',()=>{binding='';draftReset();render();});
on('document',render);on('status',render);

export function paintPassingZones(ctx,project,line) {
  const data=current();
  if(!$('passing-show').checked || !data || !lineVisible(kind()) || state.step!=='lines')return;
  (Array.isArray(data.passing_zones) ? data.passing_zones : []).forEach((zone,i)=>{
    const points=zonePath(zone,data,state.lane.closed);if(!points)return;
    line(points,passingColors[zone.side]+'55',false,false,14);
    const p=project(points[0]);ctx.save();ctx.fillStyle=passingColors[zone.side];ctx.font='bold 11px sans-serif';
    ctx.fillText(`許可 ${i+1} · ${passingSides[zone.side]}`,p[0]+8,p[1]-12);ctx.restore();
  });
  if(!$('passing-editor').open)return;
  if(state.passingPick) {
    data.points.forEach((point,i)=>{
      const p=project(point);ctx.save();ctx.beginPath();ctx.arc(p[0],p[1],3,0,Math.PI*2);
      ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle='#657d8b';ctx.lineWidth=1;ctx.stroke();
      if(data.points.length<60 || i%Math.ceil(data.points.length/40)===0){ctx.fillStyle='#354959';ctx.font='10px sans-serif';ctx.fillText(String(i+1),p[0]+4,p[1]-4);}ctx.restore();
    });
  }
  try {
    const start=index('passing-start'),end=index('passing-end');
    const points=zoneIndices(start,end,data.points.length,state.lane.closed).map(i=>data.points[i]);
    line(points,'#405c72',false,true,2);
    for(const [key,value] of [['開始',start],['終了',end]]) {
      const p=project(data.points[value]);ctx.save();ctx.beginPath();ctx.arc(p[0],p[1],6,0,Math.PI*2);ctx.fillStyle='#f2d572';ctx.fill();
      ctx.strokeStyle='#405c72';ctx.lineWidth=2;ctx.stroke();ctx.fillStyle='#263a42';ctx.font='bold 11px sans-serif';ctx.fillText(`${key} WP${value+1}`,p[0]+9,p[1]+14);ctx.restore();
    }
  } catch { /* Incomplete numeric drafts never become map permissions. */ }
}
