import assert from 'node:assert/strict';
class Element {
  constructor(){this.children=[];this.value='';this.disabled=false;this.hidden=false;this.textContent='';}
  set id(value){this._id=value;elements.set(value,this);}
  get id(){return this._id;}
  append(...items){this.children.push(...items);}
  replaceChildren(...items){this.children=items;this.value=items[0]?.value || '';}
  querySelectorAll(){return [];}
}
const elements=new Map();
const el=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);};
globalThis.document={getElementById:el,createElement:()=>new Element()};
globalThis.Option=class extends Element {constructor(label,value){super();this.textContent=label;this.value=value;}};
globalThis.CustomEvent=class extends Event {constructor(n,{detail}){super(n);this.detail=detail;}};
const timers=[];globalThis.setTimeout=fn=>{timers.push(fn);return timers.length;};globalThis.clearTimeout=()=>{};
const {state,emit}=await import('../frontend/api.js');
state.config={connection:{user:'kart',host:'jetson',port:22}};
const id='a'.repeat(64), calls=[];
let snapshot={phase:'idle',session:false,ownership:{locked:false},legacy:[]};
let delayedStatus=null,failStatus=false;
globalThis.fetch=async(url,options)=>{
  const body=JSON.parse(options.body);calls.push({url,body});
  const action=url.split('/').at(-1);
  if(action==='status' && delayedStatus)return new Promise(resolve=>delayedStatus.resolve=resolve);
  if(action==='status' && failStatus)throw Error('offline');
  let result;
  if(action==='containers')result={containers:[{id,name:'kart',running:true,root:'/repo'}]};
  else if(action==='preset_get')result={preset:null};
  else if(action==='catalog')result={modes:{'esc-trial':'ESC試験',collect:'記録'},fields:[{key:'device',kind:'text'}],status:snapshot};
  else if(action==='plan')result={revision:'reviewed',review:'ESC plan'};
  else if(action==='start'){assert.equal(body.revision,'reviewed');snapshot={...snapshot,run_id:'r1',phase:'running',session:true,ownership:{locked:true},managed:true};result=snapshot;}
  else if(action==='stop'){assert.equal(body.run_id,'r1');result={...snapshot,stop_requested:true};}
  else result=snapshot;
  return {ok:true,json:async()=>result};
};
await import('../frontend/bringup.js');
const flush=async()=>{for(let i=0;i<5;i++)await new Promise(resolve=>setImmediate(resolve));};
assert.equal(calls.length,0,'opening page must not SSH');
el('bringup-connect').onclick();await flush();
assert.equal(el('bringup-workspace').hidden,false);
assert.equal(el('bringup-start').disabled,true);
el('bringup-plan').onclick();await flush();
assert.equal(el('bringup-start').disabled,false);
el('bringup-field-device').value='/dev/new';el('bringup-field-device').onchange();
assert.equal(el('bringup-start').disabled,true,'edits invalidate review');
el('bringup-plan').onclick();await flush();
el('bringup-start').onclick();await flush();
assert.equal(el('bringup-start').disabled,true,'running session cannot start again');
assert.equal(el('bringup-stop').disabled,false);
delayedStatus={};el('bringup-refresh').onclick();await flush();
await el('bringup-stop').onclick();
assert.equal(calls.at(-1).url,'/api/bringup/stop','stop must not await status');
// Changing connection invalidates old in-flight responses and disables commands.
emit('connection');await flush();
delayedStatus.resolve({ok:true,json:async()=>snapshot});delayedStatus=null;await flush();
assert.equal(el('bringup-workspace').hidden,true);
assert.equal(el('bringup-start').disabled,true);
assert.equal(el('bringup-stop').disabled,true);
console.log('Bringup UI: explicit connection, review invalidation, duplicate prevention, independent stop, stale response suppression passed');
