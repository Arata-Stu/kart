import assert from 'node:assert/strict';
const elements=new Map();
function make(id=''){return {id,value:'',hidden:false,disabled:false,textContent:'',children:[],dataset:{},
 addEventListener(name,fn){this[name]=fn;},append(...items){this.children.push(...items);},
 replaceChildren(...items){this.children=items;}};}
function el(id){if(!elements.has(id))elements.set(id,make(id));return elements.get(id);}
globalThis.document={getElementById:el,createElement:()=>make()};
globalThis.CustomEvent=class extends Event{constructor(name,{detail}){super(name);this.detail=detail;}};
globalThis.setTimeout=()=>1;globalThis.clearTimeout=()=>{};
const {state,emit}=await import('../frontend/api.js');
const {refreshAudit,reportURL}=await import('../frontend/audit.js');
const flush=()=>new Promise(resolve=>setImmediate(resolve));
state.config={records:'/workspaces/record'};
let results=[],runResolve=null;
const calls=[];
globalThis.fetch=async(url,options)=>{
 const body=options?.body?JSON.parse(options.body):null;calls.push({url,body});
 let value;
 if(url==='/api/records')value=[{id:'2026-10-21/run',name:'run'}];
 if(url==='/api/audits')value={results};
 if(url==='/api/audit/run')return await new Promise(resolve=>{runResolve=()=>resolve({ok:true,json:async()=>({id:'job',status:'running'})});});
 if(url==='/api/stop')value={};
 return {ok:true,json:async()=>value};
};
await refreshAudit();
assert.equal(el('audit-bag').children[1].textContent,'2026-10-21/run');
assert.equal(el('audit-run').disabled,true);
el('audit-bag').value='2026-10-21/run';el('audit-bag').change();
el('audit-source').value='custom';el('audit-source').change();assert.equal(el('audit-custom-label').hidden,false);
el('audit-custom').value='/camera/rgb';el('audit-spike').value='80';
const submit=el('audit-form').submit({preventDefault(){}});
await el('audit-form').submit({preventDefault(){}});
assert.equal(calls.filter(c=>c.url==='/api/audit/run').length,1,'Double submit is refused');
assert.deepEqual(calls.at(-1).body,{bag:'2026-10-21/run',source_topic:'/camera/rgb',age_spike_ms:80});
runResolve();await submit;
state.jobs=[{id:'job',status:'running'}];emit('status');await flush();
assert.equal(el('audit-run').disabled,false);assert.equal(el('audit-stop').hidden,false);
await el('audit-stop').click();assert.equal(calls.at(-1).url,'/api/stop');
results=[{id:'2026-10-21/12-00-00/audit_12345678',created:1,bag:'run',source_topic:'/camera/rgb',age_spike_ms:80,
 final:{count:5,p95:42,max:60}}];
state.jobs=[];emit('job-finished',{id:'job',status:'succeeded',result:{audit:results[0].id}});await flush();await flush();
assert.equal(el('audit-report').src,reportURL(results[0].id)+'#timeseries');assert.equal(el('audit-report').hidden,false);
assert.match(el('audit-summary').textContent,/P95 42.0 ms/);
assert.equal(el('audit-json').href,reportURL(results[0].id,'summary.json'));
assert.equal(el('audit-run').disabled,false);
console.log('Audit UI: bag selection, atomic submit, cancellation, completed report and downloads passed');
