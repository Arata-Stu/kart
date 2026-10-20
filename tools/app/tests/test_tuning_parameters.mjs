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
globalThis.fetch=async(url,options)=> {
  const body=JSON.parse(options.body);calls.push({url,body});
  if(url.endsWith('parameters_set') && body.changes.lookahead_m!=null)lookahead=body.changes.lookahead_m;
  if(url.endsWith('parameters_set') && body.changes.steering_offset!=null)offset=body.changes.steering_offset;
  const result={map_id:'map-a',target:body.target,node:'pure_pursuit',available,revision:'params1',
    fields:available?[{name:'lookahead_m',label:'lookahead',type:3,value:lookahead,read_only:false},
                      {name:'wheelbase_m',label:'wheelbase',type:3,value:.257,read_only:true}]:[],
    line_choices:['centerline','customline'],applied:url.endsWith('parameters_set'),readback_matches:true};
  if(body.target==='vehicle')result.fields=[{name:'steering_offset',label:'offset',type:3,value:offset,read_only:false},{name:'steering_scale',label:'scale',type:3,value:-1,read_only:true}];
  return {ok:true,json:async()=>result};
};
emit('status');await flush();
await element('tuning-parameters-open').click();await flush();
assert.equal(element('tuning-parameters-dialog').open,true);
assert.equal(element('tuning-parameter-rows').children.length,2);
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
tuning.status.apply_issue='STOP required';emit('tuning');await flush();
const count=calls.length;
await element('tuning-parameters-form').submit({preventDefault(){}});
assert.equal(calls.length,count,'running state must never send changes');
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
console.log('Parameter editor reads actual values, locks static fields, gates STOP and switches active lines');

await element('tuning-offset-open').click();await flush();
assert.equal(calls.at(-1).body.target,'vehicle');
const offsetInput=element('tuning-parameter-rows').children[0].children[2].children[0];
offsetInput.value='0.02';
await element('tuning-parameters-form').submit({preventDefault(){}});await flush();
assert.deepEqual(calls.at(-1).body.changes,{steering_offset:0.02});
assert.equal(calls.at(-1).body.target,'vehicle');
assert.match(element('tuning-parameters-status').textContent,/適用後の値/);
