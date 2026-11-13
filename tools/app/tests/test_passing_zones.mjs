import assert from 'node:assert/strict';
import {zoneIndices,zonePath,newZone,nearestWaypoint} from '../frontend/passing_model.js';

assert.deepEqual(zoneIndices(1,3,5,false),[1,2,3]);
assert.deepEqual(zoneIndices(3,1,5,true),[3,4,0,1]);
for(const args of [[1,1,5,true],[3,1,5,false],[true,3,5,true],[1,5,5,true],[-1,2,5,false]])assert.throws(()=>zoneIndices(...args));
const geometry='a'.repeat(64),points=[[0,0],[1,0],[2,0],[3,0],[4,0]];
const line={points,geometry_revision:geometry,passing_zones:[]};
const zone=newZone(line,false,1,3,'left');
assert.deepEqual(zonePath(zone,line,false),points.slice(1,4));
assert.equal(zonePath({...zone,geometry_revision:'b'.repeat(64)},line,false),null);
assert.equal(zonePath({...zone,start_index:1,end_index:1},line,false),null);
assert.equal(nearestWaypoint(points,[2.1,.05],.2),2);
assert.equal(nearestWaypoint(points,[20,0],.2),-1);
assert.deepEqual(line.passing_zones,[],'model helpers never grant permission automatically');

class Element {
  constructor(id=''){this.id=id;this.value='';this.children=[];this.options=[];this.disabled=false;this.checked=false;this.open=false;this.textContent='';this.dataset={};this.style={};this.listeners={};}
  append(...children){this.children.push(...children);}
  replaceChildren(...children){this.children=children;}
  addEventListener(name,handler){this.listeners[name]=handler;}
  setAttribute(name,value){this[name]=value;}
  click(){return this.listeners.click?.();}
}
const elements=new Map();
const el=id=>{if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id);};
globalThis.document={getElementById:el,createElement:()=>new Element()};
globalThis.CustomEvent=class extends Event{constructor(name,{detail}){super(name);this.detail=detail;}};
globalThis.setTimeout=()=>1;globalThis.clearTimeout=()=>{};
const {state,emit,undo}=await import('../frontend/api.js');
el('passing-line').value='centerline';el('passing-line').options=['centerline','raceline','customline'].map(value=>({value}));
el('passing-show').checked=true;el('passing-editor').open=true;
const {paintPassingZones}=await import('../frontend/passing_zones.js');
state.id='course';state.laneId='main';state.step='lines';state.jobs=[];
state.doc={revision:1,lanes:[{id:'main',closed:false,lines:{centerline:structuredClone(line)}}]};
const flush=()=>new Promise(resolve=>setImmediate(resolve));
emit('document');await flush();
assert.equal(state.lane.lines.centerline.passing_zones.length,0);
assert.equal(state.dirty,false);
el('passing-start').value='2';el('passing-end').value='4';el('passing-side').value='left';
await el('passing-add').click();await flush();
assert.equal(state.dirty,true);
assert.deepEqual(state.lane.lines.centerline.passing_zones,[zone]);
assert.match(el('passing-summary').textContent,/有効 1区間.*未保存/);
const drawn=[];const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});
paintPassingZones(ctx,p=>p,(...args)=>drawn.push(args));
assert.ok(drawn.some(args=>args[1]==='#248d6d55' && args[4]===14));
assert.ok(drawn.some(args=>args[3]===true),'uncommitted endpoint draft remains a dashed preview');
undo();await flush();assert.equal(state.lane.lines.centerline.passing_zones.length,0);
undo(true);await flush();assert.equal(state.lane.lines.centerline.passing_zones.length,1);

await el('passing-pick-start').click();await flush();
assert.equal(state.target,null);assert.equal(state.passingPick,'start');
emit('passing-click',{point:[2.02,0],tolerance:.2});await flush();
assert.equal(el('passing-start').value,'3');assert.equal(state.passingPick,'end');
emit('passing-click',{point:[4.02,0],tolerance:.2});await flush();
assert.equal(el('passing-end').value,'5');assert.equal(state.passingPick,null);
el('passing-side').value='right';await el('passing-add').click();await flush();
assert.equal(state.lane.lines.centerline.passing_zones.length,2);
assert.equal(state.lane.lines.centerline.passing_zones[1].side,'right');
assert.equal(state.lane.lines.centerline.passing_zones[1].id,'zone_002');
el('passing-list').children[1].children[1].click();await flush();
el('passing-start').value='2';el('passing-end').value='4';el('passing-side').value='both';
await el('passing-add').click();await flush();
assert.equal(state.lane.lines.centerline.passing_zones.length,2,'editing updates a row without granting an extra interval');
assert.deepEqual(state.lane.lines.centerline.passing_zones[1],{id:'zone_002',start_index:1,end_index:3,side:'both',geometry_revision:geometry});
el('passing-list').children[0].children[2].click();await flush();
assert.equal(state.lane.lines.centerline.passing_zones.length,1);

// A new line geometry cannot silently reuse the old index range or click draft.
await el('passing-pick-start').click();await flush();
state.lane.lines.centerline.geometry_revision='b'.repeat(64);
emit('document');await flush();
assert.equal(state.passingPick,null);
assert.match(el('passing-notice').textContent,/失効/);
const stale=[];paintPassingZones(ctx,p=>p,(...args)=>stale.push(args));
assert.ok(!stale.some(args=>args[4]===14),'stale permission bands are hidden');
await el('passing-clear').click();await flush();
assert.equal(state.lane.lines.centerline.passing_zones.length,0);
el('passing-start').value='1';el('passing-end').value='1';
await el('passing-add').click();await flush();
assert.equal(state.lane.lines.centerline.passing_zones.length,0);
assert.match(el('toast').textContent,/異なる点/);
state.lane.closed=true;
el('passing-start').value='5';el('passing-end').value='2';el('passing-side').value='both';
await el('passing-add').click();await flush();
assert.deepEqual(zonePath(state.lane.lines.centerline.passing_zones[0],state.lane.lines.centerline,true),[points[4],points[0],points[1]]);
console.log('Passing zones default to none, bind server geometry, support numbered/map-picked endpoints, multiple sides, wrap, undo, deletion and stale-band hiding');
