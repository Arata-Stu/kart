import assert from 'node:assert/strict';
const elements=new Map(), saved=new Map([['kart.job.selected','second'],['kart.job.drawer','true']]);
function make(id=''){return {id,hidden:false,disabled:false,textContent:'',dataset:{},children:[],scrollHeight:0,scrollTop:0,clientHeight:0,
 classList:{toggle(){}},addEventListener(name,fn){this[name]=fn;},append(...items){this.children.push(...items);},replaceChildren(...items){this.children=items;}};}
function el(id){if(!elements.has(id))elements.set(id,make(id));return elements.get(id);}
globalThis.document={getElementById:el,createElement:()=>make()};
globalThis.localStorage={getItem:k=>saved.get(k),setItem:(k,v)=>saved.set(k,v)};
globalThis.CustomEvent=class extends Event{constructor(name,{detail}){super(name);this.detail=detail;}};
globalThis.setTimeout=()=>1;globalThis.setInterval=()=>1;globalThis.clearTimeout=()=>{};
const rows=[{id:'first',title:'VGL',status:'running',started:1,elapsed_s:5,worker_pid:100,process_pid:101},
 {id:'second',title:'学習',status:'running',started:2,elapsed_s:4,worker_pid:200,process_pid:201}];
const calls=[];
globalThis.fetch=async(url,options)=>{
 calls.push({url,body:options?.body?JSON.parse(options.body):null});
 if(url==='/api/jobs')return {ok:true,json:async()=>rows};
 if(url.startsWith('/api/log'))return {ok:true,json:async()=>({text:'persistent output'})};
 if(url==='/api/stop')return {ok:true,json:async()=>({...rows[1],cancel_requested:true})};
 throw new Error(url);
};
const {poll}=await import('../frontend/jobs.js');
await poll();
assert.equal(el('jobs-drawer').hidden,false,'Reload restores the drawer');
assert.match(el('job-log-status').textContent,/実行PID 201/,'Reload restores the selected job');
assert.match(el('job-summary').textContent,/2件の処理中/);
assert.match(el('job-summary').textContent,/実行PID 101/);
assert.equal(el('job-list').children.length,2);
await el('job-stop-selected').click();
assert.deepEqual(calls.find(c=>c.url==='/api/stop').body,{id:'second'});
assert.equal(rows[0].cancel_requested,undefined,'Stopping training does not stop VGL');
rows[1].status='cancelled';rows[1].process_pid=null;rows[1].last_process_pid=201;
await poll();
assert.match(el('job-summary').textContent,/1件の処理中/);
assert.match(el('job-log-status').textContent,/最終実行PID 201/);
assert.equal(el('job-stop-selected').hidden,true);
await el('jobs-close').click();assert.equal(saved.get('kart.job.drawer'),'false');
console.log('Persistent job UI: selected PID/log restored, parallel jobs listed, per-job cancellation isolated');
