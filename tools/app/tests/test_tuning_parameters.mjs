import assert from 'node:assert/strict';
const elements=new Map();
function make(id='') {return {id,value:'',disabled:false,hidden:false,textContent:'',dataset:{},children:[],
  addEventListener(name,fn){this[name]=fn;},setAttribute(name,value){this[name]=value;},
  append(...items){this.children.push(...items);},replaceChildren(...items){this.children=items;},
  showModal(){this.open=true;},close(){this.open=false;}};}
function element(id){if(!elements.has(id))elements.set(id,make(id));return elements.get(id);}
globalThis.document={getElementById:element,createElement:()=>make()};
globalThis.CustomEvent=class extends Event{constructor(name,{detail}){super(name);this.detail=detail;}};
globalThis.setInterval=()=>1;globalThis.setTimeout=()=>1;globalThis.clearTimeout=()=>{};
const {state,emit}=await import('../frontend/api.js');
const {tuning}=await import('../frontend/tuning.js');
await import('../frontend/tuning_parameters.js');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
state.id='course';state.doc={revision:1,lanes:[]};state.config={connection:{user:'kart',host:'example',port:2222}};
tuning.id='course';tuning.connected=true;tuning.received=Date.now();tuning.status={revision:'map1',apply_issue:''};
element('tuning-parameters-target').value='tracking';element('tuning-line').value='customline';
const calls=[];
let available=true, lookahead=.8, offset=0;
const adaptive={lookahead_mode:'fixed',lookahead_low_speed_m:.8,lookahead_high_speed_m:2};
globalThis.fetch=async(url,options)=> {
  const body=JSON.parse(options.body);calls.push({url,body});
  if(url.endsWith('parameters_set') && body.changes.lookahead_m!=null)lookahead=body.changes.lookahead_m;
  if(url.endsWith('parameters_set') && body.changes.steering_offset!=null)offset=body.changes.steering_offset;
  if(url.endsWith('parameters_set'))for(const name of Object.keys(adaptive))if(body.changes[name]!=null)adaptive[name]=body.changes[name];
  const result={map_id:'map-a',target:body.target,node:'pure_pursuit',available,revision:'params1',
    fields:available?[{name:'lookahead_m',label:'lookahead',type:3,value:lookahead,read_only:false},
                      {name:'wheelbase_m',label:'wheelbase',type:3,value:.257,read_only:true},
                      ...Object.entries(adaptive).map(([name,value])=>({name,label:name,type:typeof value==='string'?4:3,value,read_only:false}))]:[],
    line_choices:['centerline','customline'],applied:url.endsWith('parameters_set'),readback_matches:true};
  if(body.target==='vehicle')result.fields=[{name:'steering_offset',label:'offset',type:3,value:offset,read_only:false},{name:'steering_scale',label:'scale',type:3,value:-1,read_only:true}];
  return {ok:true,json:async()=>result};
};
emit('status');await flush();
await element('tuning-parameters-open').click();await flush();
assert.equal(element('tuning-parameters-dialog').open,true);
assert.equal(element('tuning-parameter-rows').children.length,5);
const first=element('tuning-parameter-rows').children[0];
const input=first.children[2].children[0];
assert.equal(input.value,'0.8');
assert.equal(element('tuning-parameter-rows').children[1].children[2].textContent,'起動時固定・変更不可');
input.value='1.1';
await element('tuning-parameters-form').submit({preventDefault(){}});await flush();
assert.equal(calls.at(-1).url,'/api/tuning/parameters_set');
assert.deepEqual(calls.at(-1).body.changes,{lookahead_m:1.1});
assert.equal(calls.at(-1).body.expected_revision,'params1');
assert.match(element('tuning-parameters-status').textContent,/適用後の値/);
tuning.status.mode=1;tuning.status.stop_issue='STOP required';emit('tuning');await flush();
assert.equal(element('tuning-parameters-apply').disabled,false,'tracking changes enabled while AUTO');
const liveInput=element('tuning-parameter-rows').children[0].children[2].children[0];
liveInput.value='1.2';
await element('tuning-parameters-form').submit({preventDefault(){}});await flush();
assert.deepEqual(calls.at(-1).body.changes,{lookahead_m:1.2});
const fieldInput=name=>element('tuning-parameter-rows').children.find(row=>row.children[2].children[0]?.['aria-label']===name).children[2].children[0];
const fieldRow=name=>element('tuning-parameter-rows').children.find(row=>row.children[2].children[0]?.['aria-label']===name);
assert.equal(fieldRow('lookahead_low_speed_m').hidden,true);
fieldInput('lookahead_mode').value='speed_interpolated';fieldInput('lookahead_mode').change();
assert.equal(fieldRow('lookahead_m').hidden,true);
assert.equal(fieldRow('lookahead_low_speed_m').hidden,false);
fieldInput('lookahead_low_speed_m').value='0.6';fieldInput('lookahead_high_speed_m').value='1.8';
await element('tuning-parameters-form').submit({preventDefault(){}});await flush();
assert.deepEqual(calls.at(-1).body.changes,{lookahead_mode:'speed_interpolated',lookahead_low_speed_m:.6,lookahead_high_speed_m:1.8});
fieldInput('lookahead_mode').value='fixed';fieldInput('lookahead_mode').change();
assert.equal(fieldRow('lookahead_m').hidden,false);
assert.equal(fieldRow('lookahead_high_speed_m').hidden,true);
await element('tuning-parameters-form').submit({preventDefault(){}});await flush();
assert.deepEqual(calls.at(-1).body.changes,{lookahead_mode:'fixed'});
tuning.status.apply_issue='STOP required';emit('tuning');await flush();
const count=calls.length;
await element('tuning-parameters-form').submit({preventDefault(){}});
assert.equal(calls.length,count,'stale or rejected state must never send changes');
assert.equal(element('tuning-parameters-apply').disabled,true);
tuning.status.apply_issue='';emit('status');await flush();
available=false;
await element('tuning-parameters-refresh').click();await flush();
assert.equal(element('tuning-parameters-apply').disabled,true);
assert.equal(element('tuning-parameter-rows').children.length,0);
available=true;
await element('tuning-line-switch').click();await flush();
assert.equal(calls.at(-1).body.target,'map');
assert.deepEqual(calls.at(-1).body.changes,{line_type:'customline'});
console.log('Parameter editor reads actual values, locks static fields, allows live tracking, retains stopped calibration and switches active lines');

await element('tuning-offset-open').click();await flush();
assert.equal(calls.at(-1).body.target,'vehicle');
const offsetInput=element('tuning-parameter-rows').children[0].children[2].children[0];
offsetInput.value='0.02';
emit('tuning');await flush();
assert.equal(element('tuning-parameters-apply').disabled,true,'vehicle calibration disabled while running');
tuning.status.mode=3;tuning.status.stop_issue='';emit('tuning');await flush();
await element('tuning-parameters-form').submit({preventDefault(){}});await flush();
assert.deepEqual(calls.at(-1).body.changes,{steering_offset:0.02});
assert.equal(calls.at(-1).body.target,'vehicle');
assert.match(element('tuning-parameters-status').textContent,/適用後の値/);

// MPC controls use the same atomic parameter transport, with method-specific inputs.
element('tuning-parameters-target').value='tracking';
adaptive.lateral_controller='pure_pursuit';adaptive.mpc_horizon_s=1;
adaptive.mpc_tracking_weight=10;adaptive.mpc_smoothness_weight=1;adaptive.mpc_steering_rate_rad_s=1.5;
await element('tuning-parameters-refresh').click();await flush();
const rows=()=>element('tuning-parameter-rows').children;
const byName=name=>rows().find(r=>r.children[0].children[0].textContent===name);
assert.equal(byName('mpc_horizon_s').hidden,true);
const method=byName('lateral_controller').children[2].children[0];
assert.deepEqual(method.children.map(o=>o.value),['pure_pursuit','mpc']);
method.value='mpc';tuning.status.stop_issue='STOP required';method.change();
assert.equal(element('tuning-parameters-apply').disabled,true);
tuning.status.stop_issue='';method.change();
assert.equal(byName('lookahead_m').hidden,true);
assert.equal(byName('mpc_horizon_s').hidden,false);
byName('mpc_horizon_s').children[2].children[0].value='1.2';
await element('tuning-parameters-form').submit({preventDefault(){}});await flush();
assert.deepEqual(calls.at(-1).body.changes,{lateral_controller:'mpc',mpc_horizon_s:1.2});
assert.equal(byName('mpc_horizon_s').children[2].children[0].value,'1.2');

assert.match(element('tuning-parameters-status').textContent,/横制御 · 適用済み: MPC · 設定版/);
assert.equal(element('tuning-parameters-status').title,'ROSノード: pure_pursuit');
const staged=byName('lateral_controller').children[2].children[0];
staged.value='pure_pursuit';staged.change();
assert.match(element('tuning-parameters-status').textContent,/適用済み: MPC/,'editing must not pretend the selection is applied');
