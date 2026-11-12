import assert from 'node:assert/strict';

const elements=new Map();
function element(id) {
  if(!elements.has(id))elements.set(id,{
    textContent:'',hidden:false,dataset:{},
    set innerHTML(value){throw Error('Remote perception data must use textContent');},
  });
  return elements.get(id);
}
globalThis.document={getElementById:element};
const {renderPerception}=await import('../frontend/perception_status.js');
const el=(kind,field)=>element(`perception-${kind}-${field}`);
const channel=(overrides={})=>({available:true,fresh:true,age_s:.125,
  frame_id:'camera_color_optical_frame',stamp_ns:'1234567890123456789',count:0,objects:[],truncated:false,...overrides});
const track={id:'17',class_id:'0',score:.876,bbox:[100,200,40,50]};

renderPerception({detections:channel(),tracks:channel({count:1,objects:[track]})},true);
assert.equal(el('detections','status').textContent,'検出: 0件','zero means an actual fresh empty message');
assert.equal(el('detections','objects').hidden,true);
assert.equal(el('tracks','status').textContent,'追跡: 1件');
assert.equal(el('tracks','objects').textContent,'ID 17 · class ID 0 · score 0.88');
assert.doesNotMatch(el('tracks','objects').textContent,/person|opponent/,'class zero has no assumed class name');
assert.match(el('tracks','source').textContent,/Jetson受信後 0.13 秒/);
assert.match(el('tracks','source').textContent,/frame: camera_color_optical_frame/);
assert.match(el('tracks','source').textContent,/1234567890123456789 ns/,'sensor stamp remains an exact string');

const expiring=channel({count:1,objects:[track],fresh_for_s:.4});
renderPerception({tracks:expiring},true,.25);
assert.equal(el('tracks','status').textContent,'追跡: 1件');
assert.match(el('tracks','source').textContent,/Jetson受信後 0.38 秒/,'display age advances between HTTP responses');
renderPerception({tracks:expiring},true,.4);
assert.equal(el('tracks','status').textContent,'追跡: 受信停止','source freshness can expire before the tuning connection');
assert.equal(el('tracks','objects').textContent,'');
assert.equal(el('tracks','objects').hidden,true);
renderPerception({detections:channel({age_s:1.99,count:0,fresh_for_s:.01})},true,.02);
assert.equal(el('detections','status').textContent,'検出: 受信停止','an expired empty message must not keep claiming zero detections');
renderPerception({tracks:channel({age_s:1.5,count:1,objects:[track]})},true,.5);
assert.equal(el('tracks','status').textContent,'追跡: 受信停止','older payloads fall back to the two-second receipt lifetime');
assert.match(el('tracks','source').textContent,/Jetson受信後 2.00 秒/);

for(const [state,expected] of [
  [channel({available:false,fresh:false,age_s:null,count:null}),'配信なし'],
  [channel({fresh:false,age_s:null,count:null}),'受信待ち'],
  [channel({fresh:false,age_s:1.5,count:null,objects:[track]}),'受信停止'],
]) {
  renderPerception({detections:state,tracks:state},true);
  for(const kind of ['detections','tracks']) {
    assert.match(el(kind,'status').textContent,new RegExp(expected));
    assert.doesNotMatch(el(kind,'status').textContent,/0件/,'missing or stale input must not claim zero objects');
    assert.equal(el(kind,'objects').textContent,'','old objects are cleared');
    assert.equal(el(kind,'objects').hidden,true);
  }
}
renderPerception({},true);
assert.match(el('detections','status').textContent,/状態未取得/,'older status responses remain distinguishable from empty detections');

const many=Array.from({length:12},(_,i)=>({...track,id:String(i+1)}));
renderPerception({tracks:channel({count:12,objects:many,truncated:true})},true);
assert.equal(el('tracks','status').textContent,'追跡: 12件（10件を表示）');
assert.equal(el('tracks','objects').textContent.split('\n').length,10);
assert.doesNotMatch(el('tracks','objects').textContent,/ID 11 ·/);

const hostile='<img src=x onerror=alert(1)>';
renderPerception({detections:channel({count:1,frame_id:hostile,objects:[{...track,id:hostile,class_id:'<script>alert(1)</script>'}]})},true);
assert.ok(el('detections','source').textContent.includes(hostile));
assert.ok(el('detections','objects').textContent.includes(hostile),'remote IDs are rendered as literal text');
assert.ok(el('detections','objects').textContent.includes('<script>alert(1)</script>'));

renderPerception({detections:channel({count:1,objects:[track]})},false);
assert.match(el('detections','status').textContent,/未接続・受信停止/);
assert.equal(el('detections','source').textContent,'');
assert.equal(el('detections','objects').textContent,'');
assert.equal(el('detections','objects').hidden,true);
console.log('Perception panel distinguishes fresh empty, absent, waiting and stale channels, bounds lists and renders remote fields as text');
