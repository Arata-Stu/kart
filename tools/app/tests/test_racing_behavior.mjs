import assert from 'node:assert/strict';

const elements=new Map();
function element(id) {
  if(!elements.has(id))elements.set(id,{textContent:'',dataset:{},
    set innerHTML(value){throw Error('Remote behavior fields must use textContent');}});
  return elements.get(id);
}
globalThis.document={getElementById:element};
const {behaviorView,renderBehavior,paintBehavior}=await import('../frontend/racing_behavior.js');
const geometry={vehicle_length_m:.35,vehicle_width_m:.19,rear_axle_to_rear_m:.1065,target_length_m:.5,target_width_m:.4,merge_clearance_m:.2};
const body=(overrides={})=>({mode:'active',state:'AVOID_OBSTACLE',reason:'static_target',valid:true,target_speed:.6,
  frame_id:'map',stamp_ns:'1234567890123456789',side:'left',target:{id:'17',kind:'obstacle',x:6,y:7,speed:0},
  path:[[4,7],[5,7],[7,7]],geometry,candidates:[
    {side:'left',feasible:true,reason:'clear',path:[[4,7],[5,8],[7,7]],min_clearance_m:.15,merge_start:[6,8],merge_end:[7,7],merge_start_s:6,merge_end_s:8},
    {side:'right',feasible:false,reason:'boundary_clearance',path:[[4,7],[5,6],[7,7]],min_clearance_m:.01}],...overrides});
const channel=(overrides={})=>({available:true,fresh:true,age_s:.05,fresh_for_s:.45,status:body(),...overrides});
const status=()=>element('behavior-status').textContent;

let view=renderBehavior(channel(),true,.1);
assert.match(status(),/active.*AVOID_OBSTACLE（障害物を回避）/);
assert.match(element('behavior-source').textContent,/0.15 秒.*frame: map.*1234567890123456789 ns/);
assert.match(element('behavior-target').textContent,/障害物.*ID 17.*推定速度 0.00 m\/s/);
assert.match(element('behavior-candidates').textContent,/左: 成立.*最小余裕 0.15 m.*復帰区間 2.00 m\n右: 不成立.*boundary_clearance/);
assert.match(element('behavior-geometry').textContent,/自車 0.35 × 0.19 m.*対象 0.50 × 0.40 m.*復帰余裕 0.20 m/);
const lines=[],projected=[];
const ctx={save(){},restore(){},beginPath(){},arc(){},fill(){}};
const project=point=>{projected.push(point);return [point[0]*10,point[1]*-10];};
assert.equal(paintBehavior(ctx,project,(...args)=>lines.push(args),channel(),true),true);
assert.deepEqual(lines[0],[[[4,7],[5,8],[7,7]],'#45a77b',false,false,2]);
assert.ok(lines.some(args=>args[1]==='#de9340' && args[3]===true),'rejected right geometry uses a dashed candidate');
assert.ok(lines.some(args=>args[1]==='#168ddd' && args[4]===4 && args[3]===false),'active selected path is solid');
assert.deepEqual(projected,[[6,8],[7,7]],'merge endpoints are map coordinates, not a car-pose transform');
const vehicle=lines.find(args=>args[1]==='#168ddd' && args[2]===true);
assert.ok(Math.abs(vehicle[0][0][0]-(4-.1065))<1e-10,'vehicle rear end is behind the rear axle by the explicit offset');
assert.ok(Math.abs(vehicle[0][1][0]-(4+.35-.1065))<1e-10,'vehicle front end extends by length minus rear offset');
assert.ok(Math.abs(vehicle[0][0][1]-(7-.19/2))<1e-10);
assert.ok(lines.some(args=>args[1]==='#d16ba5' && args[2]===true),'obstacle occupancy assumption is distinct from the car');
const missingRear=[];
paintBehavior(ctx,project,(...args)=>missingRear.push(args),channel({status:body({geometry:{...geometry,rear_axle_to_rear_m:undefined}})}),true);
assert.ok(!missingRear.some(args=>args[1]==='#168ddd' && args[2]===true),'old geometry without a rear offset does not draw a symmetric vehicle assumption');
assert.ok(missingRear.some(args=>args[1]==='#d16ba5' && args[2]===true),'target occupancy remains a centre assumption');
view=renderBehavior(channel({status:body({state:'CLEAR',target:null,
  geometry:{...geometry,target_length_m:0,target_width_m:0}})}),true);
assert.match(view.geometryText,/自車 0.35 × 0.19 m · 対象 なし/,'core CLEAR geometry retains vehicle dimensions while target dimensions are zero');
assert.equal(view.object,null);assert.ok(view.geometry);
lines.length=0;
paintBehavior(ctx,project,(...args)=>lines.push(args),channel({status:body({mode:'shadow'})}),true);
assert.ok(lines.some(args=>args[1]==='#168ddd' && args[4]===4 && args[3]===true),'shadow selected path is dashed');
assert.equal(paintBehavior(ctx,project,()=>{throw Error('off must not draw');},channel({status:body({mode:'off'})}),true),false);
view=renderBehavior(channel({status:body({state:'FAULT',valid:false,reason:'external_input:perception_timeout'})}),true);
assert.match(status(),/FAULT/);
assert.match(view.reason,/perception_timeout/);
assert.equal(view.candidates.length,0,'fresh fault diagnostics do not re-display previous sensor-dependent candidates');
assert.match(view.candidatesText,/異常のため非表示/);
assert.equal(paintBehavior(ctx,project,()=>{throw Error('source fault must hide old candidate geometry');},channel({status:body({state:'FAULT',valid:false})}),true),false);
view=renderBehavior(channel({status:body({reason:'adopted_swept_candidate',candidates:[
  {side:'left',feasible:true,reason:'swept_assumed_geometry_clear',path:[[0,0],[1,0]]},
  {side:'right',feasible:false,reason:'boundary_or_registered_obstacle',path:[]}]})}),true);
assert.match(view.reason,/占有範囲を検査した候補経路を採用（adopted_swept_candidate）/);
assert.match(view.candidatesText,/仮定した占有範囲で通過可能（swept_assumed_geometry_clear）/);
assert.match(view.candidatesText,/境界または登録障害物との余裕不足（boundary_or_registered_obstacle）/);

for(const state of ['CLEAR','FOLLOW','OBSERVE','PASS_MOVING','AVOID_SLOW','AVOID_OBSTACLE','MERGE','COAST_PASS','HOLD_SIDE','BLOCKED','FAULT']) {
  renderBehavior(channel({status:body({state,valid:!['FAULT','BLOCKED'].includes(state)})}),true);
  assert.ok(status().includes(state));
}
view=renderBehavior(channel({status:body({state:'COAST_PASS',planning_mode:'learned',
  learning:{available:true,ready:true,coverage:.8,reason:'ready'},permission:{zone_id:'zone_001',side:'left',remaining_m:4.2},
  target_observation:{mode:'predicted',age_s:.25,uncertainty_m:.14}})}),true);
assert.match(status(),/COAST_PASS（欠測中に並走継続）/);
assert.match(view.context,/計画方式: 学習モデル.*\n学習: 利用可能.*coverage 0.80/);
assert.match(view.context,/許可区間: zone_001.*左.*残り 4.20 m/);
assert.match(view.context,/予測で保持.*観測後 0.25 秒.*不確かさ 0.14 m/);
view=renderBehavior(channel({status:body({planning_mode:'simple',learning:{available:false,ready:false,coverage:0,reason:'no_rc_model'}})}),true);
assert.match(view.context,/この簡易方式は周回学習不要/);
assert.match(view.context,/学習（参考）: 利用不可.*coverage 0.00/);
view=renderBehavior(channel({status:body({reason:'prediction_uncertainty_exceeded'})}),true);
assert.match(view.reason,/対象位置の予測不確かさが上限を超過（prediction_uncertainty_exceeded）/);
view=renderBehavior(channel({status:body({planning_mode:'follow',permission:null,reason:'passing_not_authorized'})}),true);
assert.match(view.context,/許可区間: なし/);
assert.match(view.reason,/追越し許可区間外（passing_not_authorized）/);
renderBehavior(channel(),true);
assert.equal(element('behavior-context').textContent,'','old payloads remain compatible without optional diagnostics');
for(const [payload,connected,expected] of [
  [undefined,true,/未対応・状態未取得/], [channel(),false,/未接続・受信停止/],
  [channel({available:false}),true,/配信なし/], [channel({fresh:false,age_s:null}),true,/受信待ち/],
  [channel({fresh:false}),true,/表示失効/], [channel({age_s:.7,fresh_for_s:10}),true,/表示失効/],
  [channel({status:'<script>bad</script>'}),true,/状態データが不正/],
  [channel({status:body({mode:{bad:1}})}),true,/状態データが不正/],
  [channel({status:body({state:'unknown'})}),true,/状態データが不正/],
  [channel({status:body({valid:'true'})}),true,/状態データが不正/],
  [channel({status:body({target_speed:NaN})}),true,/状態データが不正/],
  [channel({status:body({frame_id:'base_link'})}),true,/frame不一致/],
]) {
  view=renderBehavior(payload,connected);
  assert.match(status(),expected);assert.equal(view.path.length,0);assert.equal(view.candidates.length,0);
  assert.equal(element('behavior-reason').textContent,'');
  assert.equal(element('behavior-target').textContent,'');
  assert.equal(paintBehavior(ctx,project,()=>{throw Error('invalid or stale status must not draw');},payload,connected),false);
}
view=renderBehavior(channel(),true,.45);
assert.match(status(),/表示失効/);
assert.equal(view.path.length,0);assert.equal(view.candidates.length,0);
assert.equal(element('behavior-candidates').textContent,'');
assert.equal(element('behavior-geometry').textContent,'');
assert.equal(behaviorView(channel({fresh_for_s:.1}),true,.1).path.length,0,'source-header expiry can precede receipt expiry');
assert.equal(behaviorView(channel({fresh_for_s:undefined,age_s:.4}),true,.11).path.length,0,'older payload lifetime is half a second');

const invalid=body({path:[[NaN,1],[2,2]],candidates:[{side:'left',feasible:false,reason:'invalid',path:[[1,Infinity],[2,3]]}]});
assert.equal(paintBehavior(ctx,project,()=>{throw Error('nonfinite geometry must not reach the map');},channel({status:invalid}),true),false);
assert.equal(behaviorView(channel({status:body({path:Array.from({length:301},()=>[0,0]),candidates:[]})}),true).path.length,0);
const hostile='<img src=x onerror=alert(1)>';
renderBehavior(channel({status:body({reason:hostile,target:{id:hostile,kind:'rc_car',x:1,y:2,speed:.3},
  candidates:[{side:'left',feasible:false,reason:hostile,path:[]}]})}),true);
assert.ok(element('behavior-reason').textContent.includes(hostile));
assert.ok(element('behavior-target').textContent.includes(hostile));
assert.ok(element('behavior-candidates').textContent.includes(hostile));
assert.equal(behaviorView(channel({status:body({target:{id:'a',kind:'person',x:1,y:2,speed:0}})}),true).object,null,'raw IDs never imply a target kind');
console.log('Behavior states, modes, candidate reasons, map routes and occupancy assumptions render with bounded types and clear on source expiry');
