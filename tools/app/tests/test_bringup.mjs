import assert from 'node:assert/strict';
class Element {
  constructor(){this.children=[];this.value='';this.disabled=false;this.hidden=false;this.textContent='';}
  set id(value){this._id=value;elements.set(value,this);}
  get id(){return this._id;}
  setAttribute(name,value){this[name]=value;}
  append(...items){this.children.push(...items);}
  replaceChildren(...items){this.children=items;this.value=items[0]?.value || '';}
  querySelectorAll(){return [];}
}
const elements=new Map();
const el=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);};
globalThis.document={getElementById:el,createElement:()=>new Element()};
globalThis.Option=class extends Element {constructor(label,value){super();this.textContent=label;this.value=value;}};
globalThis.CustomEvent=class extends Event {constructor(n,{detail}){super(n);this.detail=detail;}};
const timers=[];globalThis.setTimeout=fn=>{timers.push(fn);return timers.length;};globalThis.clearTimeout=()=>{};
const {state,emit}=await import('../frontend/api.js');
state.config={connection:{user:'kart',host:'jetson',port:22}};
const id='a'.repeat(64), calls=[];
let snapshot={phase:'idle',session:false,ownership:{locked:false},legacy:[]};
let delayedStatus=null,failStatus=false,savedPreset=null,remoteSupportsGate=true;
globalThis.fetch=async(url,options)=>{
  if(url==='/api/bringup/preview')return {ok:true,json:async()=>({modes:{collect:'記録'},fields:[{key:'device',kind:'text'}],status:{phase:'preview',session:false}})};
  const body=JSON.parse(options.body);calls.push({url,body});
  const action=url.split('/').at(-1);
  if(action==='status' && delayedStatus)return new Promise(resolve=>delayedStatus.resolve=resolve);
  if(action==='status' && failStatus)throw Error('offline');
  let result;
  if(action==='containers')result={containers:[{id,name:'kart',running:true,root:'/repo'}]};
  else if(action==='preset_get')result={preset:savedPreset};
  else if(action==='catalog')result={
    modes:{'esc-trial':'ESC試験',collect:'記録',drive:'地図走行',e2e:'E2E'},
    default_labels:{collect:{auto_fixed_throttle:'通常AUTO',auto_fixed_throttle_r2_gate:'無効'}},
    stereo_defaults:{collect:{profile:'640x480x90',infra_profile:'640x480x90',depth_profile:'0'}},
    fields:[{key:'device',kind:'text'},{key:'rgb_profile',choices:['0','640x480x30']},
      {key:'infra_profile',choices:['0','640x480x90']},{key:'depth_profile',choices:['0','640x480x90']},
      {key:'auto_fixed_throttle',kind:'text'},
      {key:'perception',kind:'text'}, {key:'perception_mode',choices:['detect','track']},
      {key:'perception_debug_image',kind:'boolean',default:null},
      {key:'overtake',choices:['off','shadow','active']},{key:'recovery',choices:['off','infra_imu']},
      ...(remoteSupportsGate?[{key:'auto_fixed_throttle_r2_gate',kind:'boolean',default:null}]:[])],
    status:snapshot,
  };
  else if(action==='plan')result={revision:'reviewed',review:'ESC plan'};
  else if(action==='start'){assert.equal(body.revision,'reviewed');snapshot={...snapshot,run_id:'r1',phase:'running',session:true,ownership:{locked:true},managed:true};result=snapshot;}
  else if(action==='stop'){assert.equal(body.run_id,'r1');result={...snapshot,stop_requested:true};}
  else result=snapshot;
  return {ok:true,json:async()=>result};
};
await import('../frontend/bringup.js');
const flush=async()=>{for(let i=0;i<5;i++)await new Promise(resolve=>setImmediate(resolve));};
assert.equal(calls.length,0,'opening page must not SSH');
el('bringup-connect').onclick();await flush();
assert.equal(el('bringup-workspace').hidden,false);
assert.equal(el('bringup-start').disabled,true);
for (const action of ['status', 'auto', 'manual', 'stop']) {
  await el('mode-' + action).onclick();
  assert.equal(calls.at(-1).url, '/api/bringup/mode_' + action);
  assert.equal(calls.at(-1).body.container, id);
}
el('bringup-mode').value='collect';el('bringup-mode').onchange();
for(const key of ['rgb_profile','stereo_profile']) assert.equal(el('bringup-field-'+key).type,'hidden');
el('bringup-field-depth_enabled').value='true';el('bringup-field-depth_enabled').onchange();
const profile=el('bringup-field-rgb_profile');
profile.value='640x480x30';profile.onchange();
const perception=el('bringup-field-perception');
perception.value='/models/yolov8s.plan';perception.onchange();
const perceptionMode=el('bringup-field-perception_mode');
perceptionMode.value='track';perceptionMode.onchange();
const debugImage=el('bringup-field-perception_debug_image');
assert.equal(debugImage.value,'','debug JPEG must stay opt-in');
debugImage.value='true';debugImage.onchange();
el('bringup-tab-log').onclick();
assert.equal(el('bringup-pane-basic').hidden,true);
assert.equal(el('bringup-pane-log').hidden,false);
el('bringup-tab-basic').onclick();
assert.equal(profile.value,'640x480x30','navigation preserves settings');
el('bringup-plan').onclick();await flush();
assert.equal(calls.find(c=>c.url.endsWith('/plan')).body.values.rgb_profile,'640x480x30');
assert.equal(calls.find(c=>c.url.endsWith('/plan')).body.values.infra_profile,'640x480x90');
assert.equal(calls.find(c=>c.url.endsWith('/plan')).body.values.depth_profile,'640x480x90');
assert.equal(calls.find(c=>c.url.endsWith('/plan')).body.values.perception,'/models/yolov8s.plan');
assert.equal(calls.find(c=>c.url.endsWith('/plan')).body.values.perception_mode,'track');
assert.equal(calls.find(c=>c.url.endsWith('/plan')).body.values.perception_debug_image,true);
assert.equal(el('bringup-pane-review').hidden,false);
assert.equal(el('bringup-start').disabled,false);
el('bringup-field-device').value='/dev/new';el('bringup-field-device').onchange();
assert.equal(el('bringup-start').disabled,true,'edits invalidate review');
el('bringup-plan').onclick();await flush();
el('bringup-start').onclick();await flush();
assert.equal(el('bringup-start').disabled,true,'running session cannot start again');
assert.equal(el('bringup-stop').disabled,false);
delayedStatus={};el('bringup-refresh').onclick();await flush();
await el('bringup-stop').onclick();
assert.equal(calls.at(-1).url,'/api/bringup/stop','stop must not await status');
// Changing connection invalidates old in-flight responses and disables commands.
emit('connection');await flush();
delayedStatus.resolve({ok:true,json:async()=>snapshot});delayedStatus=null;await flush();
assert.equal(el('bringup-workspace').hidden,true);
assert.equal(el('bringup-start').disabled,true);
assert.equal(el('bringup-stop').disabled,true);
console.log('Bringup UI: explicit connection, review invalidation, duplicate prevention, independent stop, stale response suppression passed');

const beforePreview=calls.length;
await el('bringup-preview').onclick();
assert.equal(el('bringup-workspace').hidden,false);
assert.equal(el('bringup-connect').disabled,true);
assert.equal(el('bringup-start').disabled,true);
assert.equal(el('bringup-save').disabled,true);
assert.equal(el('recording-start').disabled,true);
el('bringup-plan').onclick();await flush();
assert.match(el('bringup-review').textContent,/プレビュー/);
assert.equal(el('bringup-start').disabled,true);
el('bringup-connect').onclick();await flush();
assert.equal(calls.length,beforePreview,'preview must not contact Jetson or save presets');
el('bringup-preview-exit').onclick();
assert.equal(el('bringup-workspace').hidden,true);
assert.equal(el('bringup-connect').disabled,false);
console.log('Offline preview: local catalog, no remote calls, no start/record/save, explicit exit passed');

const {profilePicker}=await import('../frontend/profile_picker.js');
const picker=profilePicker({id:'default-rgb-test',profiles:['424x240x30','640x480x30'],optional:true,defaultValue:'640x480x30',title:'RGB'});
assert.equal(picker.input.value,'','showing the default must not create an explicit override');
assert.equal(picker.root.children[1].textContent,'640 × 480 / 30 Hz（既定）');
const controls=picker.root.children[2];
assert.equal(controls.children[1].children[1]['aria-pressed'],'true');
assert.equal(controls.children[2].children[0]['aria-pressed'],'true');
controls.children[0].children[1].onclick();
assert.equal(picker.root.children[1].textContent,'OFF');
controls.children[0].children[0].onclick();
assert.equal(picker.root.children[1].textContent,'640 × 480 / 30 Hz（既定）');
console.log('RGB defaults: resolved value, highlighted resolution/rate, OFF and reset passed');

// The identification toggle creates an explicit bool override and cannot leak
// from a recording preset into an unrelated driving mode.
snapshot={phase:'idle',session:false,ownership:{locked:false},legacy:[]};
savedPreset={container:id,values:{mode:'collect'}};
emit('connection');await flush();el('bringup-connect').onclick();await flush();
let fixed=el('bringup-field-auto_fixed_throttle');
let r2Gate=el('bringup-field-auto_fixed_throttle_r2_gate');
assert.equal(r2Gate.disabled,true,'a fixed throttle is required before enabling the gate');
assert.match(r2Gate.title,/□保持/);
assert.match(r2Gate.title,/左右InfraをON/);
assert.match(r2Gate.title,/R1開始／L1停止/);
fixed.value='0.15';fixed.onchange();
assert.equal(r2Gate.disabled,false);
r2Gate.value='true';r2Gate.onchange();
el('bringup-plan').onclick();await flush();
const lastPlan=()=>calls.filter(c=>c.url.endsWith('/plan')).at(-1).body.values;
assert.equal(lastPlan().auto_fixed_throttle_r2_gate,true);
assert.equal(lastPlan().auto_fixed_throttle,'0.15');
fixed.value='-1';fixed.onchange();
assert.equal(r2Gate.disabled,true);
el('bringup-plan').onclick();await flush();
assert.equal(lastPlan().auto_fixed_throttle_r2_gate,false,'normal AUTO explicitly disables a previously selected gate');
fixed.value='0.15';fixed.onchange();r2Gate.value='true';r2Gate.onchange();
el('bringup-mode').value='drive';el('bringup-mode').onchange();
el('bringup-plan').onclick();await flush();
assert.equal('auto_fixed_throttle_r2_gate' in lastPlan(),false,'mode switches must discard the recording option');
savedPreset={container:id,values:{mode:'e2e',auto_fixed_throttle_r2_gate:true}};
emit('connection');await flush();el('bringup-connect').onclick();await flush();
el('bringup-plan').onclick();await flush();
assert.equal(lastPlan().mode,'e2e');
assert.equal('auto_fixed_throttle_r2_gate' in lastPlan(),false,'legacy hidden preset values must not enter an E2E plan');
remoteSupportsGate=false;
savedPreset={container:id,values:{mode:'collect',auto_fixed_throttle:'-1',auto_fixed_throttle_r2_gate:true}};
emit('connection');await flush();el('bringup-connect').onclick();await flush();
el('bringup-plan').onclick();await flush();
assert.equal('auto_fixed_throttle_r2_gate' in lastPlan(),false,'an older remote catalog must not receive a new unsupported option');
console.log('Identification recording: fixed prerequisite, explicit bool override, normal-mode reset, mode/preset isolation passed');

savedPreset={container:id,values:{mode:'drive',perception:'/models/racing.plan',perception_mode:'detect',recovery:'infra_imu'}};
emit('connection');await flush();el('bringup-connect').onclick();await flush();
const overtake=el('bringup-field-overtake');
overtake.value='active';overtake.onchange();
assert.equal(el('bringup-field-perception_mode').value,'track');
assert.equal(el('bringup-field-recovery').value,'off');
el('bringup-plan').onclick();await flush();
assert.equal(lastPlan().overtake,'active');
assert.equal(lastPlan().perception_mode,'track');
assert.equal(lastPlan().recovery,'off');
el('bringup-field-perception_mode').value='detect';el('bringup-field-perception_mode').onchange();
assert.equal(overtake.value,'off');
overtake.value='shadow';overtake.onchange();
el('bringup-field-perception').value='off';el('bringup-field-perception').onchange();
assert.equal(overtake.value,'off');
el('bringup-mode').value='collect';el('bringup-mode').onchange();
el('bringup-plan').onclick();await flush();
assert.equal('overtake' in lastPlan(),false);
console.log('Live behavior startup: tracking/recovery coupling, perception prerequisite resets and drive-only isolation passed');
