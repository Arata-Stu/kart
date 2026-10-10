import assert from 'node:assert/strict';
import {reverseLane} from '../frontend/direction.js';
import {createSimulation} from '../frontend/simulation_model.js';
for (const closed of [true,false]) {
  const lane={closed,left:[[0,1],[3,1],[3,4]],right:[[0,-1],[3,-1],[5,4]],custom:[[0,0],[3,0],[3,4]],custom_speeds:[{start:1,end:3,speed:.4}],lines:{raceline:{}}};
  const original=structuredClone(lane);reverseLane(lane);
  assert.deepEqual(lane.lines,{});
  if(closed)assert.deepEqual(lane.custom[0],original.custom[0]);
  else assert.deepEqual(lane.custom[0],original.custom.at(-1));
  assert.equal(lane.custom_speeds[0].start,closed?9:4);
  reverseLane(lane);delete original.lines;delete lane.lines;assert.deepEqual(lane,original);
}
const wrap={closed:true,left:[],right:[],custom:[[0,0],[3,0],[3,4]],custom_speeds:[{start:10,end:2,speed:.5},{start:0,end:12,speed:1}],lines:{}};
reverseLane(wrap);assert.deepEqual(wrap.custom_speeds,[{start:10,end:2,speed:.5},{start:0,end:12,speed:1}]);
for(const reverse of [false,true]) {
 const points=Array.from({length:150},(_,i)=>{const a=i*2*Math.PI/150;return [4*Math.cos(a),4*Math.sin(a)];});
 if(reverse)points.reverse();
 const sim=createSimulation({points},true);
 for(let i=0;i<15000&&!sim.car.done;i++)sim.step();
 assert.equal(sim.car.reason,'1周完了');assert.ok(sim.car.maxError<.08);assert.ok(sim.car.time>20);
}
const points=Array.from({length:41},(_,i)=>[i/10,0]);
const sim=createSimulation({points,profile:points.map((p,i)=>[i/10,...p,0,0,i===0||i===40?0:.6,0])},false);
for(let i=0;i<15000&&!sim.car.done;i++){sim.step();assert.ok(sim.car.speed<=.60001);}
assert.equal(sim.car.reason,'終点到達');assert.ok(sim.car.maxError<1e-8);
assert.throws(()=>createSimulation({points},false,{lookahead:0}));
assert.throws(()=>createSimulation({points:[[0,0],[0,0]]},false));
console.log('Direction reversal and simulation tests passed');
// Speed caps stay at the same physical stations, including a wrapped interval.
const speedLane={closed:true,left:[],right:[],custom:[[0,0],[3,0],[3,4]],custom_speeds:[{start:9,end:2,speed:.4},{start:3,end:5,speed:.8}],lines:{}};
const oldSections=structuredClone(speedLane.custom_speeds);
reverseLane(speedLane);
const cap=(s,rows)=>rows.reduce((v,r)=>((r.start<r.end?s>=r.start&&s<=r.end:s>=r.start||s<=r.end)?Math.min(v,r.speed):v),3);
for(let s=.1;s<12;s+=.13)assert.equal(cap(s,oldSections),cap(12-s,speedLane.custom_speeds));
