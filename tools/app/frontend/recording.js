import { $, api, toast } from './api.js';

const recording = {target:null, epoch:0, sequence:0, snapshot:null, busy:false,
  timer:null, expiry:null, received:0, error:''};
const names={idle:'停止中',starting:'録画開始処理中',recording:'録画中',stopping:'録画停止処理中（保存を完了しています）',error:'録画エラー'};
function render() {
  const s=recording.snapshot;
  const fresh=recording.target && s?.available && Date.now()-recording.received<8000;
  $('recording-start').disabled=recording.busy || !fresh || !['idle','error'].includes(s.phase);
  $('recording-stop').disabled=recording.busy || !fresh || !['starting','recording','stopping'].includes(s.phase);
  $('recording-refresh').disabled=!recording.target || recording.busy;
  $('recording-state').textContent=recording.busy?'録画操作を送信中…':
    !recording.target?'JetsonとDockerを選択すると録画状態を取得します。':
    !s?'状態未確認。状態を再取得してください。':
    !s.available?s.message:!fresh?'受信停止・録画状態は未確認です。':names[s.phase] || s.phase;
  $('recording-uri').textContent=s?.current_uri || '—';
  $('recording-event').textContent=s?.last_event || recording.error || '';
  const raw=s?.raw;
  $('recording-raw').textContent=raw ? `EVS RAW: ${raw.state} · ${raw.last_error || raw.last_event || ''}${raw.pending?' · 処理中':''}`:'';
  $('recording-warning').textContent=s?.writing_unverified?'一時停止／snapshot設定です。録画中の表示だけではデータ書込みを確認できません。':'';
}
function call(action) {
  return api('bringup/recording_'+action,{...recording.target,recording_service:recording.snapshot?.service});
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
async function poll(epoch=recording.epoch) {
  if(!recording.target || recording.busy || epoch!==recording.epoch)return;
  const sequence=++recording.sequence;
  try {
    const data=await call('status');
    if(epoch===recording.epoch && sequence===recording.sequence)accept(data);
  } catch(e) {
    if(epoch===recording.epoch && sequence===recording.sequence){
      recording.snapshot=null;recording.error=e.message;render();
    }
  } finally {if(epoch===recording.epoch && sequence===recording.sequence)schedule(epoch);}
}
export function disconnectRecording() {
  recording.epoch++;recording.sequence++;clearTimeout(recording.timer);clearTimeout(recording.expiry);
  recording.target=null;recording.error='';recording.snapshot=null;recording.busy=false;render();
}
export function connectRecording(target) {
  disconnectRecording();recording.target=structuredClone(target);render();poll();
}
async function command(action) {
  render();
  if($('recording-'+action).disabled)return;
  const epoch=recording.epoch;
  ++recording.sequence;clearTimeout(recording.timer);recording.busy=true;render();
  try {
    const data=await call(action);
    if(epoch===recording.epoch){
      accept(data);
      if(!data.accepted)toast('録画要求は受け付けられませんでした: '+data.last_event,true);
    }
  } catch(e) {
    if(epoch===recording.epoch){recording.snapshot=null;recording.error=e.message;toast(e.message,true);}
  } finally {
    if(epoch===recording.epoch){recording.busy=false;render();schedule(epoch);}
  }
}
$('recording-start').onclick=()=>command('start');
$('recording-stop').onclick=()=>command('stop');
$('recording-refresh').onclick=()=>{clearTimeout(recording.timer);poll();};
render();
