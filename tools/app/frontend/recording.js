import { $, api, toast } from './api.js';

const recording = {target:null, epoch:0, sequence:0, snapshot:null, busy:false,
  timer:null, expiry:null, received:0, error:'', polling:false, action:'', feedback:''};
const names={idle:'停止中',starting:'録画開始処理中',recording:'録画中',stopping:'録画停止処理中（保存を完了しています）',error:'録画エラー'};
function render() {
  const s=recording.snapshot;
  const fresh=recording.target && s?.available && Date.now()-recording.received<8000;
  $('recording-start').disabled=recording.busy || !fresh || !['idle','error'].includes(s.phase);
  $('recording-stop').disabled=recording.busy || !fresh || !['starting','recording','stopping'].includes(s.phase);
  $('recording-refresh').disabled=!recording.target || recording.busy || recording.polling;
  $('recording-refresh').textContent=recording.polling?'状態を取得中…':'録画状態を再取得';
  $('recording-start').textContent=recording.action==='start'?'開始要求を送信中…':'● 録画開始（R1と共通）';
  $('recording-stop').textContent=recording.action==='stop'?'停止要求を送信中…':'■ 録画停止（L1と共通）';
  const reason=!recording.target?'JetsonとDockerを選択してください。':
    recording.busy?'録画要求の応答待ちです。':
    !s || !fresh?'録画状態を正常に取得するまで開始・停止できません。':
    !['idle','error'].includes(s.phase)?'既に録画中、または開始・停止処理中です。':'';
  $('recording-start').title=reason;
  $('recording-warning').textContent=reason || '';
  $('recording-state').textContent=recording.busy?(recording.action==='start'?'録画開始要求を送信中…':'録画停止要求を送信中…'):
    recording.polling && !s?'Jetsonへ録画状態を問い合わせ中…':
    recording.error?'録画状態／操作の確認に失敗しました。':
    !recording.target?'JetsonとDockerを選択すると録画状態を取得します。':
    !s?'状態未確認。状態を再取得してください。':
    !s.available?s.message:!fresh?'受信停止・録画状態は未確認です。':names[s.phase] || s.phase;
  $('recording-uri').textContent=s?.current_uri || '—';
  $('recording-event').textContent=[recording.error, recording.feedback, s?.last_event].filter(Boolean).join('\n');
  const progress=s?.progress;
  const elapsed=progress?.elapsed_s;
  const seconds=Number.isFinite(elapsed)?Math.floor(elapsed):null;
  const duration=seconds===null?'—':`${Math.floor(seconds/3600)}:${String(Math.floor(seconds/60)%60).padStart(2,'0')}:${String(seconds%60).padStart(2,'0')}`;
  const bytes=progress?.size_bytes;
  const size=Number.isFinite(bytes)?(bytes>=1024**3?`${(bytes/1024**3).toFixed(2)} GiB`:`${(bytes/1024**2).toFixed(1)} MiB`):'未取得';
  const count=progress?.message_count;
  $('recording-progress').textContent=`経過 ${duration} · 保存サイズ ${size} · メッセージ数 ${Number.isSafeInteger(count)?count.toLocaleString()+'（停止後）':'未取得（停止後に表示）'}`+
    (progress?.started_at?` · 開始 ${progress.started_at}`:'')+
    (s && !fresh?' · 最終取得値':' · 取得時点の値');
  const raw=s?.raw;
  $('recording-raw').textContent=raw ? `EVS RAW: ${raw.state} · ${raw.last_error || raw.last_event || ''}${raw.pending?' · 処理中':''}`:'';
  $('recording-warning').textContent+=s?.writing_unverified?'一時停止／snapshot設定です。録画中の表示だけではデータ書込みを確認できません。':'';
}
function call(action) {
  return api('bringup/recording_'+action,{...recording.target,recording_service:action==='status'?undefined:recording.snapshot?.service});
}
function accept(data) {
  recording.error='';recording.snapshot=data;recording.received=Date.now();render();
  clearTimeout(recording.expiry);
  const epoch=recording.epoch, received=recording.received;
  recording.expiry=setTimeout(()=>{if(epoch===recording.epoch && received===recording.received)render();},8100);
}
function schedule(epoch) {
  clearTimeout(recording.timer);
  if(epoch!==recording.epoch || !recording.target)return;
  recording.timer=setTimeout(()=>poll(epoch),1500);
}
async function poll(epoch=recording.epoch, manual=false) {
  if(!recording.target || recording.busy || recording.polling || epoch!==recording.epoch)return;
  const sequence=++recording.sequence;
  recording.polling=true;
  if(manual)recording.feedback='再取得ボタンを受け付けました。Jetsonからの応答待ちです。';
  render();
  try {
    const data=await call('status');
    if(epoch===recording.epoch && sequence===recording.sequence){
      recording.feedback='状態取得完了 '+new Date().toLocaleTimeString();
      accept(data);
    }
  } catch(e) {
    if(epoch===recording.epoch && sequence===recording.sequence){
      recording.snapshot=null;recording.error=e.message;recording.feedback='状態取得に失敗しました。';render();
    }
  } finally {if(epoch===recording.epoch && sequence===recording.sequence){recording.polling=false;render();schedule(epoch);}}
}
export function disconnectRecording() {
  recording.epoch++;recording.sequence++;clearTimeout(recording.timer);clearTimeout(recording.expiry);
  recording.target=null;recording.error='';recording.snapshot=null;recording.busy=false;recording.polling=false;recording.action='';recording.feedback='';render();
}
export function connectRecording(target) {
  disconnectRecording();recording.target=structuredClone(target);render();poll();
}
async function command(action) {
  render();
  if($('recording-'+action).disabled)return;
  const epoch=recording.epoch;
  ++recording.sequence;clearTimeout(recording.timer);recording.busy=true;recording.polling=false;recording.action=action;recording.feedback='ボタン操作を受け付けました。';render();
  try {
    const data=await call(action);
    if(epoch===recording.epoch){
      recording.feedback=data.accepted?'録画要求を受け付けました。':'録画要求は拒否されました。';
      accept(data);
      if(!data.accepted)toast('録画要求は受け付けられませんでした: '+data.last_event,true);
    }
  } catch(e) {
    if(epoch===recording.epoch){recording.snapshot=null;recording.error=e.message;recording.feedback='要求の結果は未確認です。状態を再取得してください。';toast(e.message,true);}
  } finally {
    if(epoch===recording.epoch){recording.busy=false;recording.action='';render();schedule(epoch);}
  }
}
$('recording-start').onclick=()=>command('start');
$('recording-stop').onclick=()=>command('stop');
$('recording-refresh').onclick=()=>{clearTimeout(recording.timer);return poll(recording.epoch,true);};
render();
