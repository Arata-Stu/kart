import assert from 'node:assert/strict';
const elements=new Map();
function el(id){if(!elements.has(id))elements.set(id,{_value:'',get value(){return this._value;},set value(v){this._value=String(v);},textContent:'',open:true,addEventListener(name,fn){this[name]=fn;},querySelectorAll(){return [];}});return elements.get(id);}
globalThis.document={getElementById:el};
globalThis.CustomEvent=class extends Event{constructor(name,{detail}){super(name);this.detail=detail;}};
globalThis.setTimeout=()=>1;globalThis.clearTimeout=()=>{};
const {state,emit}=await import('../frontend/api.js');
await import('../frontend/parking.js');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
state.id='fixture';state.doc={lanes:[],parking_areas:[]};el('parking-id').value='P1';emit('document');await flush();
await el('parking-pick-polygon').click();await flush();
for(const p of [[0,0],[2,0],[2,1],[0,1]]){emit('parking-click',p);await flush();}
assert.equal(state.parkingDraft.polygon.length,4);
await el('parking-pick-entrance').click();await flush();emit('parking-click',[1,0]);await flush();
assert.deepEqual(state.parkingDraft.entrance,[[0,0],[2,0]]);
emit('parking-drag',{handle:{field:'polygon',index:1},p:[3,0]});await flush();
assert.deepEqual(state.parkingDraft.entrance,[[0,0],[3,0]]);
await el('parking-pick-goal').click();await flush();emit('parking-click',[1,.5]);await flush();
assert.equal(state.parkingPick,'heading');emit('parking-click',[1,1]);await flush();
assert.ok(Math.abs(state.parkingDraft.goal_pose[2]-Math.PI/2)<1e-6);
emit('parking-drag',{handle:{field:'goal'},p:[1.5,.5]});await flush();
assert.equal(state.parkingDraft.goal_pose[0],1.5);
assert.deepEqual(state.doc.parking_areas,[]); // Draft edits never silently commit.
emit('parking-end');await flush();
assert.equal(state.parkingPick,null);
await el('parking-clear-maneuver').click();await flush();
assert.deepEqual(state.parkingDraft.maneuver,[]);
assert.equal(state.parkingPick,null);
