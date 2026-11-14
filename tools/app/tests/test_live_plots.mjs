import assert from 'node:assert/strict';

const elements=new Map();
function element(id) {
  if(!elements.has(id)) {
    const attributes={},classes=new Set();
    elements.set(id,{id,value:'',hidden:false,disabled:false,textContent:'',dataset:{},attributes,
      classList:{toggle(name,value){if(value)classes.add(name);else classes.delete(name);},
        contains:name=>classes.has(name)},
      setAttribute(name,value){attributes[name]=value;},
      addEventListener(name,fn){this[name]=fn;}});
  }
  return elements.get(id);
}
globalThis.document={getElementById:element};
globalThis.CustomEvent=class extends Event {constructor(name,{detail}){super(name);this.detail=detail;}};
const intervals=[],timers=new Map();let sequence=0;
globalThis.setInterval=fn=>{intervals.push(fn);return intervals.length;};
globalThis.setTimeout=(fn,delay)=>{const id=++sequence;timers.set(id,{fn,delay});return id;};
globalThis.clearTimeout=id=>timers.delete(id);
let now=10000;Date.now=()=>now;
element('map-page').hidden=true;
element('plots-page').hidden=false;
const {state,emit,on}=await import('../frontend/api.js');
const {tuning}=await import('../frontend/tuning.js');
const {plotVisible}=await import('../frontend/live_plots.js');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
state.config={connection:{user:'kart',host:'192.168.1.2',port:2222}};
emit('ready');await flush();
assert.equal(element('plots-connect').disabled,true);
assert.equal(element('plots-map-name').textContent,'未選択');
assert.match(element('plots-connection-status').textContent,/地図未選択.*地図を選ぶ/);
assert.equal(element('plots-filter-all').attributes['aria-pressed'],'true');
for(const kind of ['lookahead','speed','tracking-timing','speed-timing'])assert.equal(plotVisible(kind),true);
let views=0;on('plots-view',()=>views++);
element('plots-filter-lookahead').click();await flush();
assert.equal(plotVisible('lookahead'),true);
assert.equal(plotVisible('speed'),false);
assert.equal(element('plots-filter-all').attributes['aria-pressed'],'false');
assert.equal(element('plots-filter-lookahead').classList.contains('active'),true);
assert.equal(views,1);
element('plots-filter-all').click();await flush();
assert.equal(plotVisible('speed-timing'),true);

let mapOpened=0;
element('nav-map').click=()=>{
  mapOpened++;
  element('map-page').hidden=false;element('plots-page').hidden=true;
  emit('page','map');
};
await element('plots-open-map').click();await flush();
assert.equal(mapOpened,1);
assert.equal(plotVisible('lookahead'),false,'plots do not render on the map page');
state.id='course';state.doc={title:'テストコース',revision:1,lanes:[]};
emit('loaded');await flush();
assert.equal(element('plots-map-name').textContent,'テストコース');
assert.equal(element('plots-connect').disabled,false);
assert.equal(element('plots-open-map').textContent,'地図を開く');
const requests=[];
let pendingResponse=null;
globalThis.fetch=async(url,options)=>{
  requests.push({url,body:JSON.parse(options.body)});
  if(pendingResponse)await pendingResponse;
  return {ok:true,json:async()=>({revision:'r1',mode:3,speed_mps:0,localized:false,
    apply_issue:'STOP required',can_rollback:false})};
};
element('map-page').hidden=true;element('plots-page').hidden=false;emit('page','plots');await flush();
let release;
pendingResponse=new Promise(resolve=>release=resolve);
const first=element('plots-connect').click();
assert.equal(tuning.busy,true);
assert.equal(element('plots-connect').disabled,true);
await element('tuning-connect').click();
assert.equal(requests.length,1,'both connection buttons share one in-flight status request');
release();await first;await flush();pendingResponse=null;
assert.equal(tuning.connected,true);
assert.match(element('plots-connection-status').textContent,/接続中.*kart@192.168.1.2:2222/);
assert.equal(element('plots-connect').textContent,'切断');
assert.equal(requests[0].body.id,'course');
assert.equal(requests[0].body.connection.port,2222);
const epoch=tuning.epoch,pollTimer=tuning.timer;
element('plots-page').hidden=true;element('map-page').hidden=false;emit('page','map');await flush();
assert.equal(tuning.connected,true);
assert.equal(tuning.timer,pollTimer);
element('plots-page').hidden=false;element('map-page').hidden=true;emit('page','plots');await flush();
assert.equal(tuning.epoch,epoch);
await timers.get(pollTimer).fn();await flush();
assert.equal(requests.length,2,'live plots continue the existing poll without another connection');
now+=3100;intervals.at(-1)();
assert.match(element('plots-connection-status').textContent,/受信停止/);
await element('plots-connect').click();await flush();
assert.equal(tuning.connected,false);
assert.equal(tuning.timer,null);
assert.match(element('plots-connection-status').textContent,/未接続/);
assert.equal(requests.length,2,'disconnecting sends no ROS operation or additional request');

globalThis.fetch=async()=>{throw Error('SSH connection refused');};
await element('plots-connect').click();await flush();
assert.match(element('plots-connection-status').textContent,/接続エラー.*SSH connection refused/);
assert.equal(tuning.connected,false);
delete state.doc.title;emit('document');await flush();
assert.equal(element('plots-map-name').textContent,'course','an unnamed map falls back to its ID');
state.id=null;state.doc=null;emit('loaded');await flush();
assert.equal(element('plots-connect').disabled,true);
assert.match(element('plots-connection-status').textContent,/地図未選択/);
assert.ok(requests.every(request=>request.url==='/api/tuning/status'),'the plots screen reads status only');
console.log('Live plot filters, missing-map guidance, connection reuse, navigation, stale/error states and single polling passed');
