import assert from 'node:assert/strict';

const elements=new Map();
function element(id) {
  if(!elements.has(id))elements.set(id,{id,value:'',hidden:false,open:false,disabled:false,
    textContent:'',innerHTML:'',dataset:{},classList:{toggle(){}},
    addEventListener(name,fn){this[name]=fn;},setAttribute(name,value){this[name]=value;}});
  return elements.get(id);
}
globalThis.document={getElementById:element};
globalThis.CustomEvent=class extends Event {constructor(name,{detail}){super(name);this.detail=detail;}};
const intervals=[];
globalThis.setInterval=fn=>{intervals.push(fn);return intervals.length;};
globalThis.setTimeout=()=>1;
globalThis.clearTimeout=()=>{};
let wall=10000,monotonic=0;
Date.now=()=>wall;
globalThis.performance={now:()=>monotonic};
const {emit}=await import('../frontend/api.js');
const {tuning}=await import('../frontend/tuning.js');
await import('../frontend/tuning_lookahead.js');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const tick=()=>intervals.at(-1)();
const row=(t,values={})=>({t,controller:'pure_pursuit',lookahead_mode:'speed_interpolated',
  lookahead_m:.8,target_distance_m:.75,measured_speed_mps:-.4,target_speed_mps:2,...values});
async function receive(now,samples,stream='first') {
  tuning.received=wall;
  tuning.status={lookahead_history:{stream,now,samples}};
  emit('tuning');await flush();
}
element('plots-page').hidden=false;
element('tuning-parameters-target').value='vehicle';
element('tuning-parameters-dialog').open=false;
tuning.connected=true;
await receive(100,[row(99.9)]);
assert.equal(element('tuning-lookahead-panel').hidden,false,'the plot is independent of the parameter target and closed dialog');
assert.equal(element('tuning-lookahead-value').textContent,'0.80');
assert.equal(element('lookahead-measured-speed').textContent,'-0.40');
assert.match(element('tuning-lookahead-mode').textContent,/Pure Pursuit.*速度補間/);
assert.match(element('tuning-lookahead-note').textContent,/独立した実測値ではありません/);
assert.match(element('tuning-lookahead-chart').innerHTML,/stroke="#2563eb"/);
assert.match(element('tuning-lookahead-speed-chart').innerHTML,/m\/s/);

await receive(100.1,[row(100,{lookahead_mode:'fixed',measured_speed_mps:null})]);
assert.equal(element('lookahead-measured-speed').textContent,'—');
assert.equal(element('tuning-lookahead-value').textContent,'0.80','fixed lookahead survives an absent speed measurement');
assert.equal(element('lookahead-target-speed').textContent,'2.00');
assert.match(element('tuning-lookahead-note').textContent,/固定方式では速度をlookahead計算に使用しない/);
await receive(1,[row(.9,{controller:'mpc',lookahead_mode:null,lookahead_m:1.5,target_distance_m:1.5})],'second');
assert.match(element('tuning-lookahead-mode').textContent,/MPC.*予測終端/);
assert.match(element('tuning-lookahead-note').textContent,/両距離は予測区間の終端/);
assert.equal((element('tuning-lookahead-chart').innerHTML.match(/d="M/g)||[]).length,2,'a restarted stream replaces the preceding history');

monotonic+=2100;wall+=2100;tuning.received=wall;tick();
assert.equal(element('tuning-lookahead-value').textContent,'—','stale diagnostics expire while the connection remains fresh');
assert.equal(element('lookahead-target-speed').textContent,'—');
assert.equal(element('tuning-lookahead-mode').textContent,'—');
assert.match(element('tuning-lookahead-note').textContent,/受信停止/);
tuning.connected=false;emit('status');await flush();
assert.match(element('tuning-lookahead-note').textContent,/未接続/);
emit('connection');await flush();
assert.ok(!element('tuning-lookahead-chart').innerHTML.includes('d="M'),'connection changes clear old plots');

tuning.connected=true;
await receive(2,[row(1.9)]);
tuning.epoch++;tick();
assert.ok(!element('tuning-lookahead-chart').innerHTML.includes('d="M'),'map epoch changes clear plots even before another status event');
assert.equal(element('tuning-lookahead-value').textContent,'—');
await receive(3,[row(2.9)]);
emit('loaded');await flush();
assert.ok(!element('tuning-lookahead-chart').innerHTML.includes('d="M'),'loading a map discards the preceding history');

tuning.connected=true;tuning.received=wall;
tuning.status={};emit('tuning');await flush();
assert.match(element('tuning-lookahead-note').textContent,/未対応・未受信/,'older bridge responses have an explicit unavailable state');
await receive(4,[row(3.9,{lookahead_mode:null})]);
assert.equal(element('tuning-lookahead-value').textContent,'—');
assert.match(element('tuning-lookahead-note').textContent,/未対応・未受信/,'older controller diagnostics are not treated as current');
const invalid={t:4.1,controller:null,lookahead_mode:null,lookahead_m:null,
  target_distance_m:null,measured_speed_mps:null,target_speed_mps:null};
await receive(4.2,[row(4),invalid]);
assert.equal(element('tuning-lookahead-value').textContent,'—');
assert.match(element('tuning-lookahead-note').textContent,/横制御が非active・診断無効・受信停止/);
assert.ok(!element('tuning-lookahead-note').textContent.includes('未対応'),'a STOP or invalid sample after valid tracking is not called unsupported');
await receive(4.2,[invalid]);
assert.match(element('tuning-lookahead-note').textContent,/未対応・未受信.*または横制御が非active/,'an initial invalid sample does not prove an unsupported controller');
await element('plots-filter-speed').click();await flush();
assert.equal(element('tuning-lookahead-panel').hidden,true,'another plot filter hides lookahead');
const hiddenChart=element('tuning-lookahead-chart').innerHTML;
await receive(5,[row(4.9,{lookahead_m:1.7})]);
assert.equal(element('tuning-lookahead-chart').innerHTML,hiddenChart,'hidden cards do not redraw');
await element('plots-filter-lookahead').click();await flush();
assert.equal(element('tuning-lookahead-panel').hidden,false);
assert.equal(element('tuning-lookahead-value').textContent,'1.70','reopening uses the latest received history');
element('plots-page').hidden=true;emit('plots-view');await flush();
assert.equal(element('tuning-lookahead-panel').hidden,true,'leaving the live plots page hides the card');
element('plots-page').hidden=false;emit('plots-view');await flush();
assert.equal(element('tuning-lookahead-panel').hidden,false);
console.log('Live lookahead UI distinguishes controller semantics, stale values, unsupported nodes and connection/map/stream resets');
