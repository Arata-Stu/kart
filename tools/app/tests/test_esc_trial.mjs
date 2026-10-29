import assert from 'node:assert/strict';
class Element {
  constructor(){this.children=[];this.value='';this.disabled=false;this.width=1100;this.height=260;}
  append(x){this.children.push(x);x.parent=this;}
  get firstChild(){return this.children[0];}
  setAttribute(){}
  remove(){this.parent.children=this.parent.children.filter(x=>x!==this);}
  replaceChildren(){this.children=[];}
  querySelectorAll(){return this.children.flatMap(x=>x.children.flatMap(y=>y.children));}
  getContext(){return new Proxy({}, {get:()=>()=>{},set:()=>true});}
}
const elements=new Map();
const el=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);};
globalThis.document={getElementById:el,createElement:()=>new Element()};
globalThis.CustomEvent=class extends Event{constructor(n,{detail}){super(n);this.detail=detail;}};
const timers=[];
globalThis.setTimeout=fn=>{timers.push(fn);return timers.length;};
globalThis.clearTimeout=()=>{};globalThis.setInterval=()=>{};
const {state}=await import('../frontend/api.js');
state.config={connection:{user:'kart',host:'jetson',port:22}};
const calls=[];let pendingStart;
const snapshot={stream:'s',generation:0,cursor:1,events:[],phase:'idle',mode:3,healthy:true,bridge:{},max_output:.3,ui_timeout_s:3,index:-1,output:0};
globalThis.fetch=async(url,options)=>{
  calls.push({url,body:JSON.parse(options.body)});
  if(url.endsWith('/start')) return new Promise(resolve=>pendingStart=()=>resolve({ok:true,json:async()=>({...snapshot,phase:'arming',cursor:2})}));
  return {ok:true,json:async()=>url.endsWith('/stop')?{stop_requested:true}:snapshot};
};
await import('../frontend/esc_trial.js');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
assert.equal(el('esc-steps').children.length,4);
el('esc-connect').onclick();await flush();
assert.equal(el('esc-start').disabled,false);
const starting=el('esc-start').onclick();await flush();
assert.deepEqual(calls.at(-1).body.steps.map(x=>x.output),[.3,-.3,0,-.3]);
assert.equal(calls.at(-1).body.generation,0);
await el('esc-stop').onclick();
assert.equal(calls.at(-1).url,'/api/esc/stop','STOP must not await the unresolved start request');
pendingStart();await starting;
assert.equal(el('esc-start').disabled,true,'late start reply cannot re-enable after STOP');
await timers[0]();
assert.match(el('esc-message').textContent,/STOP状態/);
assert.equal(calls.at(-1).body.run_id,null,'STOP revokes the execution lease');
assert.equal(calls[0].body.id,undefined,'no map or localization dependency');
console.log('ESC UI: editable sequence, mapless transport, independent STOP, late response ordering passed');
