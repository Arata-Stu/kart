import assert from 'node:assert/strict';
const elements = new Map();
globalThis.document = {getElementById(id) {
 if (!elements.has(id)) elements.set(id,{checked:true,addEventListener(name,fn){this[name]=fn;}});
 return elements.get(id);
}};
globalThis.CustomEvent = class extends Event {constructor(name,{detail}){super(name);this.detail=detail;}};
const {state,emit}=await import('../frontend/api.js');
const {lineVisible,showLine}=await import('../frontend/line_visibility.js');
state.doc={lanes:[{id:'main',lines:{centerline:{points:[[0,0],[1,0]]}}}]};
const original=structuredClone(state.doc);
state.dirty=false;state.selected=3;
for (const kind of ['centerline','raceline','customline']) {
 const input=elements.get('show-'+kind);
 input.checked=false;input.change();
 assert.equal(lineVisible(kind),false);
 assert.equal(state.dirty,false);
 assert.deepEqual(state.doc,original);
}
assert.equal(state.selected,-1,'hidden editing points are deselected');
elements.get('show-line-speeds').checked=false;
elements.get('show-line-speeds').change();
assert.equal(state.showLineSpeeds,false);
assert.equal(state.dirty,false);
assert.deepEqual(state.doc,original);
showLine('customline');
assert.equal(lineVisible('customline'),true);
assert.equal(lineVisible('centerline'),false);
emit('document');await new Promise(resolve=>setImmediate(resolve));
assert.equal(elements.get('show-centerline').checked,false,'document refresh preserves visibility');
console.log('Independent line visibility, hidden-point deselection, edit reveal and document retention passed');
