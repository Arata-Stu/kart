import { $, state, api, on, toast } from './api.js';
import { connectRecording, disconnectRecording } from './recording.js';

const ui = {connection:null, container:null, catalog:null, values:{mode:'esc-trial'},
  revision:null, snapshot:null, timer:null, epoch:0, busy:false, choices:[], assetBase:null};
const labels = {
  device:'車両基板ポート', map_root:'地図の探索先', model_root:'モデルの探索先',
  map_file:'HDMap', map_dir:'VSLAM地図 / VGL bundle', model_dir:'VGL実行モデル',
  lane_id:'走行レーン', line_type:'追従ライン', e2e_model_dir:'E2Eモデル',
  rgb_profile:'RGB 解像度 × FPS', infra_profile:'Infra 解像度 × FPS', imu_profile:'IMU',
  rgb_fps:'RGB FPS（旧指定）', infra_fps:'Infra FPS（旧指定）',
  evs:'EVSを使用', no_evs:'EVSを明示的に無効化', evs_backend:'EVS処理方式',
  evs_aggregation:'EVS集計方式', evs_image:'EVS可視化画像', evs_serial:'EVSシリアル',
  evs_tensor_config:'EVS Tensor設定', evs_bias_file:'EVSバイアス', bias_root:'バイアス探索先',
  record_dir:'記録の保存先', run_name:'記録名', joy_max_throttle:'Joy最大スロットル',
  auto_fixed_throttle:'AUTO固定スロットル（手動ステア）',
  e2e_throttle_mode:'E2Eスロットル方式', e2e_fixed_throttle:'E2E固定スロットル',
  recovery:'自動復帰', tuning:'Studioからの動的調整', foxglove:'Foxglove', no_bridge:'車両基板を起動しない',
  parking_site:'駐車用地図（任意上書き）', parking_slot:'駐車区画',
  eval_localization:'評価方式', bag:'再生bag', rate:'再生速度', tracking_mode:'追跡方式',
  multicam_mode:'VSLAM速度・精度', base_frame:'基準frame', imu_frame:'IMU frame', latency_output:'遅延評価の保存先',
};
const live = ['collect','environment','drive','e2e'];
const modeKeys = {
  device:live, no_bridge:live, record_dir:[...live,'eval','e2e-latency'], run_name:live,
  rgb_fps:live, infra_fps:live, rgb_profile:live, infra_profile:live, imu_profile:live,
  evs:live, no_evs:live, evs_backend:live, evs_aggregation:live, evs_image:live,
  evs_serial:live, evs_tensor_config:live, evs_bias_file:live, bias_root:live,
  joy_max_throttle:['collect','environment'], auto_fixed_throttle:['collect','environment','drive'],
  e2e_model_dir:['e2e','e2e-latency'], e2e_throttle_mode:['e2e'], e2e_fixed_throttle:['e2e'],
  recovery:['drive','e2e'], tuning:['drive'], parking_site:['drive'], parking_slot:['drive'],
  map_file:['drive','eval','e2e'], map_dir:['drive','eval','e2e'], model_dir:['drive','eval'],
  lane_id:['drive','eval'], line_type:['drive','eval'], map_root:['drive','eval','e2e'],
  model_root:['drive','eval','e2e','e2e-latency'], multicam_mode:['drive','eval'],
  eval_localization:['eval'], bag:['eval','e2e-latency'], rate:['eval','e2e-latency'],
  tracking_mode:['eval'], base_frame:['eval'], imu_frame:['eval'], latency_output:['e2e-latency'],
  foxglove:live,
};
const essentials = new Set(['device','rgb_profile','infra_profile','imu_profile','evs','run_name',
  'joy_max_throttle','auto_fixed_throttle','e2e_throttle_mode','e2e_fixed_throttle','recovery','tuning','eval_localization','tracking_mode','rate']);
const assetKeys = new Set(['map_file','map_dir','model_dir','e2e_model_dir','lane_id','line_type','bag']);
const panes = ['basic','assets','details','review','log'];
function showPane(name) {
  for (const pane of panes) {
    $('bringup-pane-'+pane).hidden=pane!==name;
    $('bringup-tab-'+pane).setAttribute('aria-pressed',String(pane===name));
  }
}
for (const pane of panes) $('bringup-tab-'+pane).onclick=()=>showPane(pane);
function request(action, extra={}) {
  if (!ui.connection) throw Error('接続先を確認してください');
  return api('bringup/'+action,{connection:ui.connection,container:ui.container,...extra});
}
function invalidate() {
  ui.revision=null;
  $('bringup-review').textContent='設定を検証すると、採用される構成をここに表示します。';
  renderButtons();
}
function renderButtons() {
  for (const el of $('bringup-workspace').querySelectorAll('input,select')) el.disabled=ui.busy;
  const s=ui.snapshot, occupied=s?.session || s?.ownership?.locked || s?.legacy?.length;
  $('bringup-refresh').disabled=!ui.connection || !ui.container;
  $('bringup-start').disabled=ui.busy || !ui.revision || !s || !!occupied;
  $('bringup-stop').disabled=!s?.session || !s?.run_id || !['running','starting','stopping'].includes(s?.phase);
  for (const id of ['bringup-plan','bringup-assets','bringup-save']) $(id).disabled=ui.busy || !ui.catalog;
}
function renderStatus(s) {
  ui.snapshot=s;
  const names={idle:'未起動',starting:'起動要求済み',running:'プロセス稼働中',stopping:'終了待ち',exited:'終了',failed:'異常終了',lost:'セッション消失・状態要確認'};
  $('bringup-status').textContent=(names[s.phase] || s.phase) +
    (s.exit_code!==undefined?` · 終了コード ${s.exit_code}`:'') +
    (s.ownership?.locked?' · bringupロック保持中':'') +
    (s.stop_requested?' · 終了要求を送信しました。終了確認中':'');
  $('bringup-log').textContent=s.log || 'ログはまだありません。';
  $('bringup-existing').textContent=s.legacy?.length && !s.session
    ? '別の端末・旧TUIの起動を検出しました。既存のtmux/端末から終了後、状態を更新してください。\n'+s.legacy.map(x=>x.command).join('\n')
    : (!s.session && s.ownership?.locked?'別のCLI/TUIが稼働中です。既存端末から終了できます。':'');
  $('bringup-attach').textContent=s.attach || '';
  renderButtons();
}
function renderFields() {
  for (const id of ['bringup-basic','bringup-assets-fields','bringup-details']) $(id).replaceChildren();
  for (const f of ui.catalog?.fields || []) {
    const visible=ui.values.mode==='esc-trial'?f.key==='device':modeKeys[f.key]?.includes(ui.values.mode);
    if (!visible) continue;
    const label=document.createElement('label'), caption=document.createElement('span');
    caption.textContent=labels[f.key] || f.key; label.append(caption);
    let input;
    if (f.kind==='boolean' || f.choices) {
      input=document.createElement('select');
      input.append(new Option('既定を使用', ''));
      if (f.kind==='boolean') {
        input.append(new Option('有効','true'),new Option('無効','false'));
      } else for (const value of f.choices) input.append(new Option(
        (f.key==='rgb_profile' || f.key==='infra_profile') && value==='0'?'なし（OFF）':value,value));
    } else { input=document.createElement('input'); input.type='text'; input.placeholder=f.default==null?'未指定':String(f.default); }
    if (f.key==='rgb_profile' || f.key==='infra_profile') {
      // An inline list never opens a native popup under the releasing pointer.
      input.size=5;
      input.className='bringup-profile-list';
    }
    input.id='bringup-field-'+f.key; input.value=ui.values[f.key] ?? '';
    input.title=f.help || caption.textContent;
    input.onchange=()=>{
      $('bringup-choice').hidden=true;
      if(input.value==='') delete ui.values[f.key];
      else ui.values[f.key]=f.kind==='boolean'?input.value==='true':input.value;
      invalidate();
    };
    label.append(input);
    if(f.help) {const help=document.createElement('small');help.textContent=f.help;label.append(help);}
    $(assetKeys.has(f.key)?'bringup-assets-fields':essentials.has(f.key)?'bringup-basic':'bringup-details').append(label);
  }
  $('bringup-tab-details').hidden=$('bringup-details').children.length===0;
  $('bringup-tab-assets').hidden=$('bringup-assets-fields').children.length===0;
  showPane('basic');
  $('bringup-mode').value=ui.values.mode;
  $('bringup-assets').hidden=['collect','environment','esc-trial'].includes(ui.values.mode);
  $('bringup-detail-section').hidden=$('bringup-details').children.length===0;
}
async function guarded(fn) {
  if(ui.busy) return;
  ui.busy=true;renderButtons();
  const epoch=ui.epoch;
  try {await fn(epoch);} catch(e) {
    if(epoch===ui.epoch) {toast(e.message,true);$('bringup-status').textContent=e.message;}
  } finally {if(epoch===ui.epoch){ui.busy=false;renderButtons();}}
}
function reset() {
  disconnectRecording();
  ui.epoch++;clearTimeout(ui.timer);ui.catalog=null;ui.snapshot=null;ui.revision=null;
  ui.busy=false;ui.connection=null;ui.container=null;
  $('bringup-workspace').hidden=true;$('bringup-container-start').disabled=true;
  $('bringup-status').textContent='未接続。接続先を確認してください。';renderButtons();
}
async function refresh(epoch=ui.epoch) {
  const s=await request('status');if(epoch===ui.epoch)renderStatus(s);
}
function schedule(epoch) {
  clearTimeout(ui.timer);
  ui.timer=setTimeout(async()=>{
    if(epoch!==ui.epoch || !ui.catalog) return;
    try {await refresh(epoch); if(epoch===ui.epoch)schedule(epoch);}
    catch(e) {if(epoch===ui.epoch){ui.snapshot=null;invalidate();$('bringup-status').textContent='接続断・稼働状態は未確認。Jetsonのプロセスは保持されます。「状態を再取得」で復帰してください。';}}
  },2500);
}
async function loadContainer(epoch) {
  ui.container=$('bringup-container').value;ui.catalog=null;ui.snapshot=null;invalidate();
  if(!ui.container) return;
  const data=await request('catalog');if(epoch!==ui.epoch)return;
  ui.catalog=data;$('bringup-workspace').hidden=false;
  connectRecording({connection:ui.connection,container:ui.container});
  $('bringup-mode').replaceChildren(...Object.entries(data.modes).map(([v,l])=>new Option(l,v)));
  renderFields();renderStatus(data.status);schedule(epoch);
}
$('bringup-connect').onclick=()=>{
  disconnectRecording();
  // A fresh epoch binds every response to this explicit connection selection.
  clearTimeout(ui.timer);ui.epoch++;ui.busy=false;
  guarded(async epoch=>{
  ui.connection=structuredClone(state.config.connection);ui.catalog=null;ui.snapshot=null;invalidate();
  $('bringup-target').textContent=`${ui.connection.user}@${ui.connection.host}:${ui.connection.port}`;
  try {
    const [data,saved]=await Promise.all([request('containers'),request('preset_get')]);
    if(epoch!==ui.epoch)return;
    const select=$('bringup-container');select.replaceChildren(new Option('Dockerを選択',''));
    for(const c of data.containers) select.append(new Option(`${c.name} · ${c.running?'稼働中':'停止中'} · ${c.root}`,c.id));
    ui.containers=data.containers;
    if(saved.preset){ui.values=saved.preset.values;select.value=saved.preset.container;}
    if(!select.value && data.containers.length===1)select.value=data.containers[0].id;
    $('bringup-status').textContent=data.containers.length?'Dockerを選択してください。':'kartの既存Dockerが見つかりません。Jetsonで初回環境構築が必要です。';
    await selectContainer(epoch);
  } finally {if(epoch===ui.epoch){ui.busy=false;renderButtons();}}
  });
};
async function selectContainer(epoch) {
  disconnectRecording();
  const selected=ui.containers?.find(c=>c.id===$('bringup-container').value);
  ui.container=selected?.id;ui.catalog=null;ui.snapshot=null;invalidate();
  $('bringup-workspace').hidden=true;
  $('bringup-container-start').disabled=!selected || selected.running;
  if(selected?.running)await loadContainer(epoch);
  else if(selected)$('bringup-status').textContent='Docker停止中。「既存Dockerを起動」で再開できます。';
}
$('bringup-container').onchange=()=>{
  clearTimeout(ui.timer);ui.epoch++;ui.busy=false;
  guarded(epoch=>selectContainer(epoch));
};
$('bringup-container-start').onclick=()=>guarded(async epoch=>{
  const data=await request('container_start');if(epoch!==ui.epoch)return;
  ui.containers=data.containers;await selectContainer(epoch);
});
$('bringup-mode').onchange=()=>{ui.values={mode:$('bringup-mode').value};invalidate();renderFields();$('bringup-choice').hidden=true;};
$('bringup-refresh').onclick=()=>guarded(async epoch=>{await refresh(epoch);if(epoch===ui.epoch)schedule(epoch);});
$('bringup-plan').onclick=()=>guarded(async epoch=>{
  invalidate();const values=structuredClone(ui.values);
  const data=await request('plan',{values});if(epoch!==ui.epoch || JSON.stringify(values)!==JSON.stringify(ui.values))return;
  ui.revision=data.revision;$('bringup-review').textContent=data.review;showPane('review');renderButtons();
});
$('bringup-start').onclick=()=>guarded(async epoch=>{
  const revision=ui.revision;ui.revision=null;renderButtons();
  const data=await request('start',{values:ui.values,revision});if(epoch===ui.epoch){renderStatus(data);schedule(epoch);}
});
// Stop is independent of a pending start/status request. Never queue it behind guarded().
$('bringup-stop').onclick=async()=>{
  const epoch=ui.epoch,run_id=ui.snapshot?.run_id;
  if(!run_id)return;
  ui.revision=null;renderButtons();
  try {const data=await request('stop',{run_id});if(epoch===ui.epoch){renderStatus(data);schedule(epoch);}}
  catch(e){if(epoch===ui.epoch){$('bringup-status').textContent='終了結果は未確認: '+e.message;toast(e.message,true);}}
};
$('bringup-save').onclick=()=>guarded(async()=>{await request('preset_save',{preset:{container:ui.container,values:ui.values}});toast('この接続先の起動設定を保存しました。自動起動はしません。');});
async function assetStep(epoch) {
  const data=await request('assets',{values:ui.assetBase,selections:ui.choices});if(epoch!==ui.epoch)return;
  if(data.prompt){
    ui.prompt=data.prompt;$('bringup-choice-title').textContent=data.prompt.title;
    $('bringup-choice-select').replaceChildren(...data.prompt.labels.map((s,i)=>new Option(s,String(i))));
    $('bringup-choice').hidden=false;
  }else{ui.values=data.values;$('bringup-choice').hidden=true;renderFields();invalidate();}
}
$('bringup-assets').onclick=()=>guarded(async epoch=>{
  ui.assetBase=structuredClone(ui.values);
  for(const key of assetKeys)delete ui.assetBase[key];
  ui.choices=[];invalidate();await assetStep(epoch);
});
$('bringup-choice-next').onclick=()=>guarded(async epoch=>{
  ui.choices.push(ui.prompt.values[Number($('bringup-choice-select').value)]);await assetStep(epoch);
});
$('bringup-choice-cancel').onclick=()=>{$('bringup-choice').hidden=true;ui.choices=[];};
on('connection',reset);
renderButtons();
