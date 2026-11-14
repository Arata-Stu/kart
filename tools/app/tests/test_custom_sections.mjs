import assert from 'node:assert/strict';
import { suggestCustomSections } from '../frontend/custom_sections.js';
import { covers, splitSection, mergeSection } from '../frontend/custom_model.js';

// Two straights connected by a known 3 m radius quarter-circle.
function bend(step = 0.1) {
  const pts=[];
  for(let i=0;i<=Math.round(10/step);i++)pts.push([10*i/Math.round(10/step),0]);
  const count=Math.round(3*Math.PI/2/step);
  for(let i=1;i<=count;i++){const t=i*Math.PI/(2*count);pts.push([10+3*Math.sin(t),3-3*Math.cos(t)]);}
  for(let i=1;i<=Math.round(10/step);i++)pts.push([13,3+10*i/Math.round(10/step)]);
  return pts;
}
const pts=bend(), original=structuredClone(pts);
const rows=suggestCustomSections(pts,false);
assert.deepEqual(rows.map(r=>r.kind),['straight','left','straight']);
assert.ok(Math.abs(rows[1].start-10)<0.3);
assert.ok(Math.abs(rows[1].end-(10+3*Math.PI/2))<0.3);
assert.equal(rows[0].start,0);
assert.ok(rows.every((r,i)=>r.end>r.start && (!i || r.start===rows[i-1].end)));
assert.deepEqual(pts,original,'geometry remains unchanged');
assert.deepEqual(suggestCustomSections(pts.map(([x,y])=>[x,-y]),false).map(r=>r.kind),['straight','right','straight']);
const coarse=suggestCustomSections(bend(0.5),false);
assert.deepEqual(coarse.map(r=>r.kind),rows.map(r=>r.kind));
assert.ok(Math.abs(coarse[1].start-rows[1].start)<0.6,'distance-based neighborhood handles different densities');
assert.deepEqual(suggestCustomSections(pts,false,{threshold:1}).map(r=>r.kind),['straight']);
assert.deepEqual(suggestCustomSections(pts,false,{minLength:6}).map(r=>r.kind),['straight']);
const straight=Array.from({length:100},(_,i)=>[i/10,0.002*Math.sin(i*1.7)]);
assert.deepEqual(suggestCustomSections(straight,false).map(r=>r.kind),['straight'],'small positional noise is suppressed');

const circle=Array.from({length:120},(_,i)=>[4*Math.cos(i*Math.PI/60),4*Math.sin(i*Math.PI/60)]);
assert.deepEqual(suggestCustomSections(circle,true).map(r=>r.kind),['left']);
assert.deepEqual(suggestCustomSections([...circle,circle[0]],true),suggestCustomSections(circle,true));
assert.deepEqual(suggestCustomSections(circle.toReversed(),true).map(r=>r.kind),['right']);
const stadium=[];
for(let i=0;i<50;i++)stadium.push([i*.2,-3]);
for(let i=0;i<60;i++){const t=-Math.PI/2+i*Math.PI/60;stadium.push([10+3*Math.cos(t),3*Math.sin(t)]);}
for(let i=0;i<50;i++)stadium.push([10-i*.2,3]);
for(let i=0;i<60;i++){const t=Math.PI/2+i*Math.PI/60;stadium.push([3*Math.cos(t),3*Math.sin(t)]);}
const seam=suggestCustomSections(stadium.slice(25).concat(stadium.slice(0,25)),true);
assert.deepEqual(seam.map(r=>r.kind),['straight','left','straight','left','straight']);
assert.ok(Math.abs(seam[0].end+seam.at(-1).end-seam.at(-1).start-10)<0.1,'straight crossing origin is preserved');
// S-bend retains the distinction between left and right turns.
const sBend=Array.from({length:241},(_,i)=>{const x=i/10;return [x,2*Math.sin(x/2)];});
const sRows=suggestCustomSections(sBend,false);
assert.ok(sRows.some(r=>r.kind==='left') && sRows.some(r=>r.kind==='right'));

// Speed inheritance respects overlaps, gaps and closed-loop wrapped ranges.
const line=Array.from({length:101},(_,i)=>[i/10,0]);
const old=[{start:2,end:4,speed:0.7},{start:3,end:5,speed:0.4}];
const inherited=suggestCustomSections(line,false,{},old,1.5);
assert.equal(inherited.length,1);assert.equal(inherited[0].speed,0.4);
assert.equal(suggestCustomSections(line,false,{},[{start:0,end:10,speed:2}],1.5)[0].speed,2);
assert.equal(suggestCustomSections(line,false,{},[{start:0,end:9,speed:2}],1.5)[0].speed,1.5);
const wrapped=[{start:20,end:3,speed:0.3},{start:0,end:2,speed:0.2}];
assert.equal(suggestCustomSections(circle,true,{},wrapped)[0].speed,0.2);
for (const r of suggestCustomSections(pts,false,{},old)) {
  for(let s=r.start+0.001;s<r.end;s+=0.03) {
    const hits=old.filter(v=>covers(v,s));
    assert.ok(r.speed<=(hits.length ? Math.min(...hits.map(v=>v.speed)) : 1.5));
  }
}
const divided=splitSection(rows,5);
assert.equal(divided[1].kind,'straight');
assert.equal(mergeSection(divided,1)[0].kind,'straight');
assert.equal(mergeSection(rows,1)[0].kind,undefined,'mixed manual merge discards classification');
assert.deepEqual(suggestCustomSections([[0,0],[0,0],[1,0],[2,0]],false).map(r=>r.kind),['straight']);
for(const options of [{threshold:0},{window:NaN},{minLength:-1}])assert.throws(()=>suggestCustomSections(pts,false,options));
assert.throws(()=>suggestCustomSections([[0,0],[0,0]],false));
assert.throws(()=>suggestCustomSections([[0,0],[Infinity,0]],false));
assert.throws(()=>suggestCustomSections(pts,false,{},[{start:0,end:2,speed:NaN}]));
const manyTurns=Array.from({length:2001},(_,i)=>[i/10,Math.sin(i/10)]);
assert.throws(()=>suggestCustomSections(manyTurns,false,{window:.5,minLength:.01}),/40区間/);
console.log('Custom section suggestions: known geometry, density, noise, S-bends, loops, speed inheritance, edits and invalid inputs passed');
