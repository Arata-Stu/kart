import assert from 'node:assert/strict';
const elements=new Map();
function make(id=''){return {id,value:'',disabled:false,children:[],textContent:'',
  get options(){return this.children;},addEventListener(name,fn){this[name]=fn;},
  replaceChildren(...rows){this.children=rows;},append(...rows){this.children.push(...rows);}};}
function el(id){if(!elements.has(id))elements.set(id,make(id));return elements.get(id);}
globalThis.document={getElementById:el,createElement:()=>make()};
globalThis.CustomEvent=class extends Event{constructor(name,{detail}){super(name);this.detail=detail;}};
globalThis.setTimeout=()=>1;globalThis.clearTimeout=()=>{};
const {state,emit,undo}=await import('../frontend/api.js');
await import('../frontend/lanes.js');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
state.laneId='main';
state.doc={lanes:[{id:'main',centerline_speed_mps:1.5,lines:{centerline:{points:[[0,0],[1,0]]}}},
                 {id:'second',centerline_speed_mps:.8,lines:{}}]};
emit('document');await flush();
assert.equal(el('centerline-speed').value,1.5);
el('centerline-speed').value='0.7';await el('centerline-speed').input();await flush();
assert.equal(state.lane.centerline_speed_mps,.7);
assert.equal(state.dirty,true);
assert.ok(state.lane.lines.centerline,'Speed-only edits preserve generated geometry');
undo();await flush();assert.equal(state.lane.centerline_speed_mps,1.5);
for(const invalid of ['', '-1', 'NaN', '21']){
 el('centerline-speed').value=invalid;await el('centerline-speed').change();await flush();
 assert.equal(state.lane.centerline_speed_mps,1.5);
}
el('centerline-speed').value='0';await el('centerline-speed').change();await flush();
assert.equal(state.lane.centerline_speed_mps,0,'Zero is a stopped target');
state.laneId='second';emit('document');await flush();assert.equal(el('centerline-speed').value,.8);
state.doc.snapshot_status='pending';emit('status');await flush();assert.equal(el('centerline-speed').disabled,true);
console.log('Lane Center speed: per-lane edits, undo, zero target, validation and geometry retention passed');
