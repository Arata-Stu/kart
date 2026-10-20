import { $, state, action, on, emit, toast } from './api.js';
import { tuning, parameterConnectionReady, parameterRequest } from './tuning.js';

let snapshot=null, busy=false, serial=0, epoch=tuning.epoch;
const inputs=new Map();
function canApply() {
  return parameterConnectionReady() && !tuning.busy && !busy && snapshot?.available && !tuning.status?.apply_issue;
}
function controls() {
  $('tuning-parameters-apply').disabled=!canApply();
  $('tuning-parameters-refresh').disabled=busy || !parameterConnectionReady();
  $('tuning-parameters-target').disabled=busy;
  for(const {input} of inputs.values())input.disabled=busy || !parameterConnectionReady();
}
function reset() {
  serial++; snapshot=null; inputs.clear();
  $('tuning-parameter-rows').replaceChildren();
  $('tuning-parameters-status').textContent='接続または対象が変わりました。現在値を再取得してください。';
  controls();
}
function render(result) {
  snapshot=result; inputs.clear();
  const rows=$('tuning-parameter-rows');rows.replaceChildren();
  for(const field of result.fields || []) {
    const row=document.createElement('tr'), label=document.createElement('td'), current=document.createElement('td'), edit=document.createElement('td');
    label.textContent=field.label;
    const code=document.createElement('code');code.textContent=field.name;label.append(code);
    if(field.description){const help=document.createElement('small');help.textContent=field.description;label.append(help);}
    current.textContent=field.value==null?'未対応':String(field.value);
    if(field.read_only)edit.textContent='起動時固定・変更不可';
    else {
      const input=document.createElement(field.type===4?'select':'input');
      if(field.type===4) {
        for(const [value,text] of [['centerline','Center'],['raceline','Race'],['customline','Custom']]) {
          const option=document.createElement('option');option.value=value;option.textContent=text;
          option.disabled=!!result.line_choices && !result.line_choices.includes(value);
          input.append(option);
        }
      } else {input.type='number';input.step=field.type===2?'1':'any';input.required=true;}
      input.value=String(field.value);input.setAttribute('aria-label',field.name);
      inputs.set(field.name,{input,field});edit.append(input);
    }
    row.append(label,current,edit);rows.append(row);
  }
  $('tuning-parameters-status').textContent=result.available
    ? `${result.node} · 現在値 ${result.revision.slice(0,10)}${result.applied ? (result.readback_matches ? ' · 適用後の値を確認しました' : ' · 値が再変更されています。現在値を確認してください') : ''}`
    : result.reason || '対象ノードが未起動です';
  controls();
}
async function read() {
  if(!parameterConnectionReady())throw new Error('先に自己位置を接続してください');
  const token=++serial, currentEpoch=tuning.epoch, target=$('tuning-parameters-target').value;
  snapshot=null; busy=true; controls();
  $('tuning-parameter-rows').replaceChildren();
  $('tuning-parameters-status').textContent='Jetsonの現在値を取得中…';
  try {
    const result=await parameterRequest('parameters_get',{target});
    if(token===serial && currentEpoch===tuning.epoch)render(result);
  } catch(error) {
    if(token===serial)$('tuning-parameters-status').textContent=error.message;
    throw error;
  } finally {busy=false;controls();}
}
action('tuning-parameters-open', async()=> {
  $('tuning-parameters-dialog').showModal();
  await read();
});
action('tuning-offset-open',async()=> {
  $('tuning-parameters-target').value='vehicle';
  $('tuning-parameters-dialog').showModal();
  await read();
});
action('tuning-parameters-close',()=> $('tuning-parameters-dialog').close());
action('tuning-parameters-refresh',read);
$('tuning-parameters-target').addEventListener('change',()=>read().catch(error=>toast(error.message,true)));
$('tuning-parameters-form').addEventListener('submit',async event=> {
  event.preventDefault();
  if(!canApply())return toast(tuning.status?.apply_issue || '接続と現在値を確認してください',true);
  const changes={};
  for(const [name,{input,field}] of inputs) {
    const value=field.type===4?input.value:Number(input.value);
    if(value!==field.value)changes[name]=value;
  }
  if(!Object.keys(changes).length)return toast('変更した項目がありません');
  const token=++serial, currentEpoch=tuning.epoch;
  const body={target:snapshot.target,expected_revision:snapshot.revision,changes};
  busy=true;controls();
  $('tuning-parameters-status').textContent='適用結果を確認中…';
  try {
    const result=await parameterRequest('parameters_set',body);
    if(token!==serial || currentEpoch!==tuning.epoch)return;
    render(result);emit('tuning');
  } catch(error) {
    if(token===serial) {
      snapshot=null;
      $('tuning-parameters-status').textContent=`${error.message} 現在値を再取得してください。`;
    }
    toast(error.message,true);
  } finally {busy=false;controls();}
});
action('tuning-line-switch',async()=> {
  if(!parameterConnectionReady() || tuning.status?.apply_issue)throw new Error(tuning.status?.apply_issue || '接続を確認してください');
  const currentEpoch=tuning.epoch, line=$('tuning-line').value;
  tuning.busy=true;emit('status');
  try {
    const current=await parameterRequest('parameters_get',{target:'map'});
    if(currentEpoch!==tuning.epoch)return;
    if(!current.available || !current.line_choices?.includes(line))throw new Error('実車の選択laneにこのラインがありません。編集結果をプレビューして適用してください');
    const result=await parameterRequest('parameters_set',{target:'map',expected_revision:current.revision,changes:{line_type:line}});
    if(currentEpoch!==tuning.epoch)return;
    if(!result.readback_matches)throw new Error('適用後の値が変わりました。現在値を確認してください');
    toast('実車の参照ラインを切り替えました。次の状態取得で適用ラインを更新します');
    snapshot=null;serial++;controls();
  } finally {tuning.busy=false;emit('status');}
});
on('tuning',()=> {if(epoch!==tuning.epoch){epoch=tuning.epoch;reset();}controls();});
on('status',controls);
on('loaded',reset);
on('connection',reset);
