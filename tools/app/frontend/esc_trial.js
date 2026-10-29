import { $, state, api, on } from './api.js';

const trial = {connection:null, connected:false, timer:null, snapshot:null, events:[], stream:null,
  cursor:0, run:null, busy:false, epoch:0, received:0, stopping:false};
const modeNames={1:'AUTO',2:'MANUAL',3:'STOP',4:'PROPO'};
const pathNames={0:'無効（中立）',1:'プロポ',2:'HOST',3:'フェイルセーフ'};
const escNames={unknown:'不明',neutral_initial:'初期中立',forward:'前進入力',neutral_after_forward:'前進後の中立',brake:'制動入力',neutral_after_brake:'制動後の中立',reverse:'後退入力',neutral_after_reverse:'後退後の中立'};
const active = () => ['arming','running'].includes(trial.snapshot?.phase);
function step(output=0,seconds=0.5) {
  const tr=document.createElement('tr');
  for (const value of ['', output, seconds]) {
    const td=document.createElement('td');
    if (value!=='') {
      const input=document.createElement('input'); input.type='number'; input.step='0.01'; input.value=value;
      input.setAttribute('aria-label',tr.children.length===1?'ESC入力（正規化値）':'保持時間（秒）'); td.append(input);
    }
    tr.append(td);
  }
  tr.children[1].firstChild.min=-1; tr.children[1].firstChild.max=1;
  tr.children[2].firstChild.min=0.02; tr.children[2].firstChild.max=20;
  const td=document.createElement('td'), remove=document.createElement('button');
  remove.textContent='削除'; remove.onclick=()=>{tr.remove();numberRows();};td.append(remove);tr.append(td);
  $('esc-steps').append(tr); numberRows();
}
function numberRows() { [...$('esc-steps').children].forEach((row,i)=>row.firstChild.textContent=i+1); }
[[0.3,1],[-0.3,1],[0,0.5],[-0.3,1]].forEach(([u,t])=>step(u,t));
$('esc-add').onclick=()=>{if($('esc-steps').children.length<32)step();};
function request(action, extra={}) {
  return api('esc/'+action,{connection:trial.connection || state.config?.connection,
    stream:trial.stream,after:trial.cursor,run_id:trial.run,generation:trial.snapshot?.generation,...extra});
}
function update(data) {
  if(data.stream===trial.stream && data.cursor<trial.cursor)return;
  if(data.stream!==trial.stream || data.reset) trial.events=[];
  for(const row of data.events||[]) if(data.stream!==trial.stream || row.seq>trial.cursor || data.reset)trial.events.push(row);
  trial.events=trial.events.slice(-6000);trial.cursor=data.cursor;trial.stream=data.stream;
  trial.snapshot=data;trial.received=Date.now();render();
}
function render() {
  const s=trial.snapshot, fresh=trial.connected && Date.now()-trial.received<2000;
  $('esc-start').disabled=!fresh || trial.busy || trial.stopping || active() || s?.mode!==3 || !s?.healthy;
  $('esc-add').disabled=active()||trial.busy;
  for(const el of $('esc-steps').querySelectorAll('input,button')) el.disabled=active()||trial.busy;
  const bridge=s?.bridge||{};
  $('esc-current').textContent=s ? `${fresh?'接続中':'受信停止／状態未確認'} · ${s.phase} (${s.reason||'—'}) · 段階 ${s.index+1} · 要求 ${s.output} · モード ${modeNames[s.mode]||s.mode} · 基板経路 ${pathNames[s.board_path]||s.board_path} · 基板PWM ${s.pwm?.[1]??'不明'} µs · ESC推定 ${escNames[bridge.esc_command_state]||'不明'} · 次の負入力 ${bridge.esc_negative_action||'不明'} · ブレーキ拒否 ${bridge.esc_brake_rejected||'不明'} · 監視 ${s.healthy?'正常':'未準備／異常'} · 出力上限 ±${s.max_output} · 通信断 ${s.ui_timeout_s}sで中立` : '状態未確認';
  $('esc-events').replaceChildren();
  const start=trial.events.find(e=>e.kind==='start' && e.run_id===s?.run_id)?.mono_s ?? trial.events[0]?.mono_s ?? 0;
  const transitions=[], previous={};
  for(const e of trial.events.filter(e=>e.kind!=='sample')) {
    let description=JSON.stringify(e.data);
    const value=e.kind==='bridge'?escNames[e.data.esc_command_state]:e.kind==='mode'?modeNames[e.data.mode]:e.kind==='board_path'?pathNames[e.data.path]:null;
    if(value){description=`${previous[e.kind]||'未観測'} → ${value}`+(e.kind==='bridge'?`（推定） / 次の負入力 ${e.data.esc_negative_action} / 拒否 ${e.data.esc_brake_rejected}`:'');previous[e.kind]=value;}
    transitions.push({...e,description});
  }
  for(const e of transitions.slice(-150).reverse()) {
    const row=document.createElement('tr');
    for(const text of [(e.mono_s-start).toFixed(3),e.kind,e.description]) {
      const cell=document.createElement('td');cell.textContent=text;row.append(cell);
    }
    $('esc-events').append(row);
  }
  draw();
}
function draw() {
  const canvas=$('esc-chart'),ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height;
  ctx.clearRect(0,0,w,h);ctx.font='13px sans-serif';ctx.fillStyle='#444';
  // Include transitions and PWM receipts so a <50 ms stage is not hidden by the sample rate.
  let held={output:0,pwm:null,healthy:false};const samples=[];
  for(const e of trial.events){
    if(e.kind==='sample')held={...held,...e.data};
    else if(e.kind==='sequence')held={...held,output:e.data.output};
    else if(e.kind==='pwm')held={...held,pwm:[e.data.servo_us,e.data.esc_us]};
    else continue;
    samples.push({mono_s:e.mono_s,data:{...held}});
  }
  if(!samples.length)return;
  const begin=samples[0].mono_s,end=samples.at(-1).mono_s,span=Math.max(1,end-begin);
  const x=t=>55+(t-begin)/span*(w-115),y=u=>25+(1-u)*100;
  ctx.strokeStyle='#ddd';ctx.beginPath();ctx.moveTo(55,y(0));ctx.lineTo(w-60,y(0));ctx.stroke();
  for(const u of [-1,0,1]){ctx.fillText(String(u),8,y(u)+4);ctx.fillText(String(1500+500*u),w-52,y(u)+4);}
  // Staircase, not interpolation: output is held over each sample interval.
  for(const [color,get] of [['#2563eb',e=>e.data.output],['#d97706',e=>e.data.pwm?(e.data.pwm[1]-1500)/500:null]]) {
    ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();let prev=null;
    for(const e of samples){const v=get(e);if(v===null||!e.data.healthy){prev=null;continue;}
      if(prev && e.mono_s-prev.t<0.2){ctx.lineTo(x(e.mono_s),y(prev.v));ctx.lineTo(x(e.mono_s),y(v));}
      else ctx.moveTo(x(e.mono_s),y(v));prev={t:e.mono_s,v};}
    ctx.stroke();
  }
  ctx.fillText(`直近 ${span.toFixed(1)} 秒 / ESC入力`,55,h-12);
}
async function poll(epoch) {
  if(!trial.connected||epoch!==trial.epoch)return;
  try {
    const result=await request('status');
    if(epoch!==trial.epoch)return;
    update(result);
    if(trial.stopping && !active() && result.mode===3){trial.stopping=false;trial.run=null;$('esc-message').textContent='STOP状態を受信しました。中立化は基板PWMでも確認してください。';render();}
  } catch(error) {
    if(epoch!==trial.epoch)return;
    trial.connected=false;trial.run=null;render();
    $('esc-message').textContent='接続失敗。状態未確認・自動再開しません。'+error.message;return;
  }
  trial.timer=setTimeout(()=>poll(epoch),250);
}
$('esc-connect').onclick=()=>{
  if((trial.connected&&active())||trial.busy)return;
  clearTimeout(trial.timer);trial.connection=structuredClone(state.config?.connection);
  trial.connected=true;trial.run=null;trial.stopping=false;poll(++trial.epoch);
};
$('esc-start').onclick=async()=>{
  if(trial.busy||trial.stopping||active())return;
  const steps=[...$('esc-steps').children].map(row=>({output:Number(row.children[1].firstChild.value),seconds:Number(row.children[2].firstChild.value)}));
  trial.busy=true;trial.run=crypto.randomUUID();render();
  try {
    const result=await request('start',{steps});
    if(!trial.stopping){update(result);$('esc-message').textContent='開始受付済み。車体側で再生中。';}
  } catch(error) {
    trial.run=null;$('esc-message').textContent='開始結果未確認／拒否。自動再送しません。'+error.message;
  } finally {trial.busy=false;render();}
};
$('esc-stop').onclick=async()=>{
  // Do not wait for an outstanding start/poll. The server cancels queued starts.
  trial.stopping=true;trial.run=null;render();
  $('esc-message').textContent='STOP送信中…';
  try {await request('stop');$('esc-message').textContent='STOP要求を受付。中立・STOP状態の受信待ち。';}
  catch(error){$('esc-message').textContent='STOP送信未確認。通信監視による中立化と車体を確認してください。'+error.message;}
};
$('esc-export').onclick=()=>{
  const blob=new Blob([JSON.stringify({schema:'kart.esc_trial.v1',snapshot:trial.snapshot,events:trial.events},null,2)],{type:'application/json'});
  const link=document.createElement('a');link.href=URL.createObjectURL(blob);link.download='esc-trial.json';link.click();setTimeout(()=>URL.revokeObjectURL(link.href),1000);
};
on('connection',()=>{
  // End the lease on a target change; retain the old connection for STOP until explicitly reconnecting.
  clearTimeout(trial.timer);trial.epoch++;trial.connected=false;trial.run=null;render();
});
setInterval(render,1000);
