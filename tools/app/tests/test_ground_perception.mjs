import assert from 'node:assert/strict';

const elements=new Map();
function element(id) {
  if(!elements.has(id))elements.set(id,{textContent:'',dataset:{},
    set innerHTML(value){throw Error('Remote ground status must use textContent');}});
  return elements.get(id);
}
globalThis.document={getElementById:element};
const {groundPerceptionView,renderGroundPerception,paintGroundPerception}=await import('../frontend/ground_perception.js');
const diagnostic=(overrides={})=>({available:true,fresh:true,age_s:.1,fresh_for_s:2.4,
  reason:'projected',message:'mapへ投影しました',level:0,...overrides});
const rc={detection_id:'17',class_id:'2',kind:'rc_car',score:.9,x:4,y:7,z:0};
const obstacle={detection_id:'18',class_id:'1',kind:'obstacle',score:.8,x:-3,y:2,z:.1};
const channel=(overrides={})=>({available:true,fresh:true,age_s:.1,fresh_for_s:.9,
  frame_id:'map',stamp_ns:'1234567890123456789',count:2,objects:[rc,obstacle],truncated:false,
  diagnostic:diagnostic(),...overrides});
const status=()=>element('ground-perception-status').textContent;

let view=renderGroundPerception(channel(),true,.2);
assert.equal(status(),'地図上の接地点: RCカー 1件 · 障害物 1件');
assert.equal(view.objects[0].class_id,'2','COCO class 2 may explicitly identify the RC kind');
assert.match(element('ground-perception-source').textContent,/0.30 秒.*frame: map.*1234567890123456789 ns/);
assert.match(element('ground-perception-diagnostic').textContent,/projected/);

const projected=[],fills=[],labels=[],arcs=[],vertices=[];
const ctx={save(){},restore(){},beginPath(){},closePath(){},stroke(){},strokeText(){},
  fill(){fills.push(this.fillStyle);},fillText(text){labels.push(text);},
  arc(...args){arcs.push(args);},moveTo(...args){vertices.push(args);},lineTo(...args){vertices.push(args);}};
const project=point=>{projected.push(point);return [100+point[0]*10,200-point[1]*10];};
assert.equal(paintGroundPerception(ctx,project,channel(),true),2);
assert.deepEqual(projected,[[4,7],[-3,2]],'only the received map XY is passed to the map canvas projection');
assert.deepEqual(fills,['#39dcb0','#d16ba5']);
assert.deepEqual(labels,['RC','障']);
assert.deepEqual(arcs[0].slice(0,3),[140,130,6]);
assert.deepEqual(vertices[0],[70,173],'obstacles have a diamond at their own map position');
assert.equal(paintGroundPerception(ctx,()=>[NaN,1],channel(),true),0,'invalid screen projections never reach canvas methods');

renderGroundPerception(channel(),true,.9);
assert.match(status(),/受信停止・表示失効/);
assert.equal(paintGroundPerception(ctx,project,channel(),true,.9),0,'points expire before the overall three-second connection');
assert.match(element('ground-perception-source').textContent,/1.00 秒/);
assert.equal(groundPerceptionView(channel({fresh_for_s:.1}),true,.1).objects.length,0,'source-header expiry can be earlier than receipt expiry');
assert.equal(groundPerceptionView(channel({fresh_for_s:undefined,age_s:.8}),true,.2).objects.length,0,'older payloads use a one-second receipt lifetime');

for(const [payload,connected,message] of [
  [undefined,true,/未対応・状態未取得/],
  [channel(),false,/未接続・受信停止/],
  [channel({available:false}),true,/配信なし/],
  [channel({fresh:false,age_s:null,count:null}),true,/受信待ち/],
  [channel({fresh:false}),true,/受信停止/],
  [channel({frame_id:'base_link'}),true,/frame不一致/],
  [channel({frame_id:'/map'}),true,/frame不一致/],
  [channel({age_s:NaN}),true,/受信待ち/],
  [channel({count:null}),true,/不正/],
  [channel({count:0}),true,/不正/],
  [channel({diagnostic:diagnostic({reason:'tf_unavailable',message:'撮影時刻のTFがありません',level:1})}),true,/投影できません/],
  [channel({fresh:false,count:null,objects:[],diagnostic:diagnostic({reason:'tf_unavailable',message:'撮影時刻のTFがありません',level:1})}),true,/投影できません/],
]) {
  view=renderGroundPerception(payload,connected);
  assert.match(status(),message);
  assert.equal(view.objects.length,0);
  assert.doesNotMatch(status(),/RCカー 0件/,'unknown, stale or failed input must not claim an observed zero');
  assert.equal(paintGroundPerception(ctx,project,payload,connected),0);
}
assert.match(groundPerceptionView(channel({diagnostic:diagnostic({reason:'tf_unavailable',message:'TFがありません',level:1})}),true).diagnostic,/tf_unavailable/);
renderGroundPerception(channel({count:0,objects:[],diagnostic:diagnostic({reason:'zero_detections',message:'検出0件'})}),true);
assert.equal(status(),'地図上の接地点: RCカー 0件 · 障害物 0件','fresh empty projection is distinguishable from absent input');

const invalid=[{...rc,x:NaN},{...rc,y:Infinity},{...rc,z:null},{...rc,score:NaN},
  {...rc,score:1.1},{...rc,kind:undefined,class_id:'0'},{...rc,kind:'person',class_id:'0'},null];
view=renderGroundPerception(channel({objects:[...invalid,obstacle],count:invalid.length+1}),true);
assert.deepEqual(view.objects,[obstacle],'neither malformed numbers nor class ID zero imply an RC point');
assert.equal(view.rc,0);assert.equal(view.obstacles,1);
assert.match(status(),/不正な点を除外/);
const many=Array.from({length:105},(_,i)=>({...rc,detection_id:String(i)}));
view=renderGroundPerception(channel({objects:many,count:105,truncated:true}),true);
assert.equal(view.objects.length,100);
assert.match(status(),/表示 100 \/ 受信 105件/);

const hostile='<img src=x onerror=alert(1)>';
renderGroundPerception(channel({frame_id:hostile,diagnostic:diagnostic({message:hostile,reason:hostile})}),true);
assert.ok(element('ground-perception-source').textContent.includes(hostile));
assert.ok(element('ground-perception-diagnostic').textContent.includes(hostile),'diagnostic text is literal, never HTML');
view=groundPerceptionView(channel({diagnostic:diagnostic({age_s:2.4,fresh_for_s:.1})}),true,.11);
assert.match(view.diagnostic,/未受信・受信停止/);
view=groundPerceptionView(channel({diagnostic:diagnostic({age_s:2.4,fresh_for_s:undefined})}),true,.11);
assert.match(view.diagnostic,/未受信・受信停止/,'older diagnostic payloads use 2.5 seconds');
renderGroundPerception(channel(),false);
assert.equal(element('ground-perception-source').textContent,'');
assert.equal(element('ground-perception-diagnostic').textContent,'');
console.log('Map ground points preserve map coordinates and kinds, distinguish empty/stale/TF failure, bound and validate points, and render remote status as text');
