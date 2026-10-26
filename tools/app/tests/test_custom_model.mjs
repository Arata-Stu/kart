import assert from 'node:assert/strict';
import {copyCustom, partitionSections, splitSection, mergeSection, covers, hasOverlap, isPartition} from '../frontend/custom_model.js';
const lane={closed:true,lines:{centerline:{points:[[0,0],[4,0],[4,3],[0,0]]},raceline:{points:[[0,0],[3,0],[3,2]]}}};
const copied=copyCustom(lane,'centerline');
assert.deepEqual(copied,[[0,0],[4,0],[4,3]]);
copied[0][0]=9;assert.equal(lane.lines.centerline.points[0][0],0);
assert.deepEqual(copyCustom(lane,'manual'),[]);
assert.throws(()=>copyCustom({closed:false,lines:{}},'raceline'));
const rows=[{start:8,end:2,speed:.8},{start:0,end:1,speed:.4}];
assert.ok(hasOverlap(rows,10));
const partition=partitionSections(rows,10,1.5);
assert.deepEqual(partition,[{start:0,end:1,speed:.4},{start:1,end:2,speed:.8},{start:2,end:8,speed:1.5},{start:8,end:10,speed:.8}]);
assert.ok(!hasOverlap(partition,10));
assert.ok(isPartition(partition,10));
assert.ok(!isPartition(rows,10));
assert.ok(!isPartition([{start:0,end:2,speed:1},{start:3,end:10,speed:1}],10));
for(let s=.01;s<10;s+=.13){
 const matches=rows.filter(r=>covers(r,s));
 const effective=matches.length?Math.min(...matches.map(r=>r.speed)):1.5;
 assert.equal(partition.find(r=>covers(r,s)).speed,effective);
}
const split=splitSection(partition,4.25);
assert.deepEqual(split.slice(2,4),[{start:2,end:4.25,speed:1.5},{start:4.25,end:8,speed:1.5}]);
assert.deepEqual(mergeSection(split,3),partition);
assert.deepEqual(splitSection(partition,2),partition);
assert.equal(partition.length,4,'immutable edits');
assert.deepEqual(mergeSection([{start:0,end:1,speed:2},{start:1,end:2,speed:1}],0),[{start:0,end:2,speed:1}]);
assert.throws(()=>splitSection(Array.from({length:40},(_,i)=>({start:i,end:i+1,speed:1})),.5));
console.log('Custom copy isolation, loop endpoint, range conversion, overlap, split and merge passed');
