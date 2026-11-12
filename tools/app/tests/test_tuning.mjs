import assert from 'node:assert/strict';
const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {value:'', hidden:false, disabled:false, textContent:'',dataset:{},
    addEventListener(name, fn){this[name]=fn;}});
  return elements.get(id);
}
globalThis.document = {getElementById:element};
globalThis.CustomEvent = class extends Event {constructor(name,{detail}){super(name);this.detail=detail;}};
const intervalCallbacks=[];
globalThis.setInterval = callback => {intervalCallbacks.push(callback);return 1;};
globalThis.setTimeout = () => 1;
globalThis.clearTimeout = () => {};
let now=10000;
Date.now=()=>now;
const {state, emit} = await import('../frontend/api.js');
const {tuning, paintTuning} = await import('../frontend/tuning.js');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
state.id='course'; state.laneId='main'; state.dirty=false;
state.config={connection:{user:'kart',host:'192.168.1.2',port:2222}};
state.doc={revision:1,lanes:[{id:'main',closed:false,lines:{customline:{points:[[0,0],[1,0]]}}}]};
element('tuning-line').value='customline'; element('tuning-speed').value='0';
const active={document:state.doc,lane_id:'main',line_type:'customline'};
const calls=[];
const prepared=[];
let prepareOverride;
globalThis.fetch=async(url, options)=> {
  const body=JSON.parse(options.body);
  calls.push([url,body]);
  let candidate;
  if(url.endsWith('/prepare')) {
    // Python's JSON spelling must survive the browser without numeric coercion.
    candidate=`{"revision":"preview-${prepared.length+1}","document":${JSON.stringify(state.doc)},"lane_id":"main","line_type":"customline","centerline_speed_mps":${body.centerline_speed_mps},"numbers":[0.0,-0.0,1e-07,1e+21,9007199254740993]}`;
    if(prepareOverride===undefined)prepared.push(candidate);
  }
  const result=url.endsWith('/prepare') ? prepareOverride ?? {snapshot_json:candidate}
    : {map_id:'map-a',revision:url.endsWith('/apply')?'r2':'r1',mode:1,speed_mps:2,stop_issue:'STOP required',localized:true,pose:{x:1,y:2,yaw:0},active,pose_stream:'test-stream',pose_cursor:90,pose_reset:false,
       perception:{detections:{available:true,fresh:true,age_s:.1,frame_id:'camera',stamp_ns:'1000000000',count:0,objects:[],truncated:false},
         tracks:{available:true,fresh:true,age_s:.1,fresh_for_s:.4,frame_id:'camera',stamp_ns:'1000000000',count:1,objects:[{id:'17',class_id:'2',score:.9,bbox:[100,100,30,30]}],truncated:false}},
       pose_samples:Array.from({length:90},(_,i)=>({seq:i+1,stamp_ns:String(1000000000+i*11111112),x:i/90,y:(i/90)**2,yaw:0,break_before:i===0||i===45})),apply_issue:'',can_rollback:true};
  return {ok:true,json:async()=>result};
};
emit('ready'); await flush();
assert.match(element('tuning-target').textContent,/kart@192.168.1.2:2222/);
await element('tuning-connect').click(); await flush();
assert.equal(tuning.connected,true);
assert.equal(element('perception-detections-status').textContent,'検出: 0件');
assert.match(element('perception-tracks-objects').textContent,/ID 17 · class ID 2 · score 0.90/);
assert.equal(tuning.trail.filter(Boolean).length,90,'all 90 TF samples reach the trail in one HTTP response');
assert.equal(calls[0][1].connection.port,2222);
now+=500;
for(const callback of intervalCallbacks)callback();
assert.equal(tuning.connected,true);
assert.equal(element('tuning-status').textContent.includes('適用可能'),true,'the overall tuning connection remains fresh');
assert.equal(element('perception-tracks-status').textContent,'追跡: 受信停止','the timer expires channel data before the three-second connection timeout');
assert.equal(element('perception-tracks-objects').textContent,'');
assert.match(element('perception-tracks-source').textContent,/Jetson受信後 0.60 秒/);
now-=500;
for(const invalid of [{},{snapshot_json:42},{snapshot_json:{revision:'invalid'}}]) {
  prepareOverride=invalid;
  await element('tuning-preview').click(); await flush();
  assert.equal(tuning.preview,null,'invalid prepare responses must not create an applicable candidate');
  assert.equal(tuning.previewJson,null,'invalid prepare responses must discard the original payload');
  assert.equal(element('tuning-apply').disabled,true);
  assert.match(element('toast').textContent,/候補|プレビュー|形式|データ/,'invalid responses explain the preview error');
}
prepareOverride=undefined;
await element('tuning-preview').click(); await flush();
assert.equal(element('tuning-apply').disabled,false);
assert.equal(tuning.previewJson,prepared[0]);
assert.notEqual(JSON.stringify(JSON.parse(prepared[0])),prepared[0],'the fixture exposes browser numeric spelling changes');
let arrows=0, lines=[];
const ctx=new Proxy({}, {get:(_,name)=>()=>{if(name==='fill')arrows++;},set:()=>true});
paintTuning(ctx,p=>p,(...args)=>lines.push(args));
assert.equal(arrows,1);
assert.ok(lines.some(args=>args[1]==='#ffffff'));
assert.ok(lines.some(args=>args[1]==='#1599df'));
tuning.status.tracking_reference={origin:[1,2],target:[1.8,2],lookahead_m:.8,target_distance_m:.8};
let referenceFills=0;
const referenceCtx=new Proxy({}, {get:(_,name)=>()=>{if(name==='fill')referenceFills++;},set:()=>true});
const referenceLines=[];
paintTuning(referenceCtx,p=>p,(...args)=>referenceLines.push(args));
assert.equal(referenceFills,2,'vehicle and actual target are drawn');
assert.ok(referenceLines.some(args=>args[1]==='#efb62d' && args[0][1][0]===1.8));
emit('status'); await flush();
assert.match(element('tuning-lookahead').textContent,/0.80 m（経路長）/);
now+=3100;
paintTuning(ctx,p=>p,()=>{});
assert.equal(arrows,1,'stale pose and reference must not be drawn');
emit('status'); await flush();
assert.equal(element('tuning-apply').disabled,true);
assert.match(element('tuning-lookahead').textContent,/受信停止/);
assert.equal(element('perception-tracks-objects').textContent,'','stale tuning connection hides previously received tracks');
assert.match(element('perception-detections-status').textContent,/未接続・受信停止/);
now-=3100;
state.dirty=true; emit('document'); await flush();
assert.equal(tuning.preview,null);
assert.equal(tuning.previewJson,null,'editing discards both the display candidate and its original payload');
assert.equal(element('tuning-apply').disabled,true);
// Regenerating a line changes the document revision and requires a new preview.
state.doc.revision=2;
state.doc.lanes[0].lines.customline.points=[[0,0],[2,0]];
state.dirty=false; emit('document'); await flush();
await element('tuning-preview').click(); await flush();
assert.equal(prepared.length,2);
assert.notEqual(prepared[1],prepared[0]);
assert.equal(tuning.preview.document.revision,2);
assert.equal(tuning.previewJson,prepared[1]);
await element('tuning-apply').click(); await flush();
assert.equal(tuning.status.revision,'r2');
assert.match(element('perception-tracks-objects').textContent,/ID 17/,'fresh status restores the track summary');
assert.equal(tuning.status.mode,1,'apply does not require stopping or switch mode');
assert.equal(tuning.trail.filter(Boolean).length,90,'repeated batches do not duplicate the trail');
assert.equal(calls.at(-1)[1].pose_after,90);
assert.equal(calls.at(-1)[1].pose_stream,'test-stream');
assert.equal(calls.at(-1)[1].expected_revision,'r1');
assert.equal(calls.at(-1)[1].snapshot,prepared[1],'apply sends the latest original Python JSON string exactly');
assert.match(calls.at(-1)[1].snapshot,/\[0\.0,-0\.0,1e-07,1e\+21,9007199254740993\]/,'integer, signed-zero and exponent spelling is preserved');
assert.equal(calls.at(-1)[0],'/api/tuning/apply');
emit('connection'); await flush();
assert.equal(tuning.connected,false);
assert.equal(tuning.active,null);
assert.equal(element('perception-tracks-objects').textContent,'','disconnect clears the track summary');
assert.equal(tuning.previewJson,null,'disconnect clears the original preview payload');
assert.equal(element('tuning-apply').disabled,true);
assert.ok(calls.every(([url])=>url.startsWith('/api/tuning/')));
console.log('Tuning connection reuse, opaque snapshot transport, preview guards, stale pose hiding and revisioned apply passed');
