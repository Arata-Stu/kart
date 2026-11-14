import assert from 'node:assert/strict';

const elements=new Map();
function make(id='') {
  return {id,value:'',hidden:false,open:false,disabled:false,textContent:'',innerHTML:'',dataset:{},children:[],
    classList:{toggle(){}},addEventListener(name,fn){this[name]=fn;},
    setAttribute(name,value){this[name]=value;},replaceChildren(...items){this.children=items;}};
}
function element(id) {if(!elements.has(id))elements.set(id,make(id));return elements.get(id);}
globalThis.document={getElementById:element,createElement:()=>make()};
globalThis.CustomEvent=class extends Event {constructor(name,{detail}){super(name);this.detail=detail;}};
globalThis.setInterval=()=>1;globalThis.setTimeout=()=>1;globalThis.clearTimeout=()=>{};
let wall=10000,monotonic=0;
Date.now=()=>wall;globalThis.performance={now:()=>monotonic};
const {emit}=await import('../frontend/api.js');
const {tuning}=await import('../frontend/tuning.js');
await import('../frontend/tuning_speed.js');
await import('../frontend/tuning_timing.js');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const panels=['tuning-speed-panel','tuning-tracking-timing-panel','tuning-speed-timing-panel'];
const legend=id=>element(id+'-legend').children.map(span=>span.textContent).join(' ');
async function filter(kind) {await element('plots-filter-'+kind).click();await flush();}
async function render() {emit('plots-view');await flush();}
element('tuning-parameters-target').value='vehicle';element('tuning-parameters-dialog').open=false;
element('plots-page').hidden=false;
tuning.connected=true;tuning.received=wall;
tuning.status={speed_history:{now:100,samples:[{t:99.9,target:2,measured:1.25}]},timing_history:{
  tracking:{now:100,samples:[{t:99.9,processing_ms:2,pose_age_ms:20}]},
  speed:{now:100,samples:[{t:99.9,processing_ms:1,target_age_ms:15,measurement_age_ms:5}]},
}};
emit('tuning');await flush();
for(const id of panels)assert.equal(element(id).hidden,false,'all cards display without parameter target or dialog selection');
assert.equal(element('tuning-speed-target').textContent,'2.00');
assert.equal(element('tuning-speed-measured').textContent,'1.25');
assert.match(legend('tuning-tracking-processing'),/処理時間.*2\.00/);
assert.match(legend('tuning-speed-processing'),/処理時間.*1\.00/);
assert.match(legend('tuning-tracking-age'),/自己位置.*20\.00/);
assert.match(legend('tuning-speed-age'),/目標指令.*15\.00.*速度計測.*5\.00/);
assert.match(element('tuning-tracking-processing-chart').innerHTML,/d="M/);
assert.match(element('tuning-speed-processing-chart').innerHTML,/d="M/);

await filter('tracking-timing');
assert.equal(element('tuning-speed-panel').hidden,true);
assert.equal(element('tuning-tracking-timing-panel').hidden,false);
assert.equal(element('tuning-speed-timing-panel').hidden,true);
const speedChart=element('tuning-speed-chart').innerHTML;
const speedTimingChart=element('tuning-speed-processing-chart').innerHTML;
tuning.status.speed_history={now:100.1,samples:[{t:100,target:3,measured:2.25}]};
tuning.status.timing_history={...tuning.status.timing_history,speed:{now:100.1,samples:[{t:100,processing_ms:4,target_age_ms:25,measurement_age_ms:6}]}};
emit('tuning');await flush();
assert.equal(element('tuning-speed-chart').innerHTML,speedChart,'hidden speed card does not redraw');
assert.equal(element('tuning-speed-processing-chart').innerHTML,speedTimingChart,'hidden timing card does not redraw');
await filter('speed-timing');
assert.equal(element('tuning-tracking-timing-panel').hidden,true);
assert.equal(element('tuning-speed-timing-panel').hidden,false);
assert.match(legend('tuning-speed-processing'),/4\.00/,'showing a card uses its latest independent history');
await filter('speed');
assert.equal(element('tuning-speed-panel').hidden,false);
assert.equal(element('tuning-speed-target').textContent,'3.00');
for(const id of panels.slice(1))assert.equal(element(id).hidden,true);

await filter('all');
monotonic=600;wall+=600;tuning.received=wall;await render();
assert.ok(legend('tuning-tracking-processing').includes('—'),'timing retains its half-second freshness boundary');
assert.equal(element('tuning-speed-target').textContent,'3.00','speed retains its two-second freshness boundary');
monotonic=2200;wall+=1600;tuning.received=wall;await render();
assert.equal(element('tuning-speed-target').textContent,'—');
assert.match(element('tuning-speed-note').textContent,/受信停止/);
tuning.connected=false;await render();
for(const id of ['tuning-speed-note','tuning-tracking-timing-status','tuning-speed-timing-status'])assert.match(element(id).textContent,/未接続/);
element('plots-page').hidden=true;await render();
for(const id of panels)assert.equal(element(id).hidden,true,'leaving the page hides every independent card');
element('plots-page').hidden=false;await render();
for(const id of panels)assert.equal(element(id).hidden,false);
console.log('Independent live speed and timing cards honor filters, hidden rendering and their existing freshness boundaries');
