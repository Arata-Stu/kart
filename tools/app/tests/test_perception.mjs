import assert from 'node:assert/strict';
const elements=new Map(),windowEvents=new Map();
const context=new Proxy({},{get:()=>()=>{},set:()=>true});
class Element {
  constructor(id=''){this.id=id;this.children=[];this.value='';this.textContent='';this.hidden=false;this.disabled=false;this.dataset={};this.events=new Map();this.classList={toggle(){}};}
  addEventListener(name,fn){if(!this.events.has(name))this.events.set(name,[]);this.events.get(name).push(fn);}
  async trigger(name,event={}){for(const fn of this.events.get(name) || [])await fn({target:this,preventDefault(){},...event});}
  click(){return this.trigger('click');}
  append(...children){this.children.push(...children);}
  replaceChildren(...children){this.children=children;this.value=children[0]?.value || '';}
  setAttribute(key,value){this[key]=value;}
  set innerHTML(value){throw Error('Remote names, selection reasons and commands must stay text');}
  getBoundingClientRect(){return {left:50,top:100,width:1000,height:600};}
  getContext(){return context;}
  focus(){}setPointerCapture(){}
}
const el=id=>{if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id);};
globalThis.document={getElementById:el,createElement:()=>new Element()};
globalThis.Option=class extends Element{constructor(text,value){super();this.textContent=text;this.value=value;}};
globalThis.Image=class {constructor(){this.naturalWidth=640;this.naturalHeight=480;}set src(value){this.url=value;queueMicrotask(()=>this.onload?.());}};
globalThis.CustomEvent=class extends Event{constructor(name,{detail}){super(name);this.detail=detail;}};
globalThis.addEventListener=(name,fn)=>windowEvents.set(name,fn);
globalThis.ResizeObserver=class{observe(){}};
globalThis.setTimeout=()=>1;globalThis.clearTimeout=()=>{};globalThis.devicePixelRatio=2;
let now=10000;Date.now=()=>now;
const defaults={filter:'all','filter-split':'all','extract-split':'train',topic:'/realsense/color/image_raw',
  'sample-hz':'2','max-frames':'500',confidence:'.1',epochs:'50',batch:'8',imgsz:'640',lr0:'.001','new-class':'0',
  'selection-method':'yolo','selection-limit':'100','selection-max-images':'500',
  'selection-review-confidence':'0.10','selection-expected-max-detections':'3','selection-large-bbox-area-ratio':'0.40',
  'model-root':'/home/kart/workspaces/kart/weights/racing/yolov8s/finetuned','container-model-root':'/workspaces/weights/racing/yolov8s/finetuned'};
for(const [key,value] of Object.entries(defaults))el('perception-'+key).value=value;
el('perception-page').hidden=false;el('jobs-drawer').hidden=true;
let detail={id:'project-a',name:'室内 <test>',class_names:['rc_car','obstacle'],class_schema_upgraded:true,revision:1,frames:[
  {id:'f1',width:640,height:480,bag:'record-a',stamp_ns:'111',split:'train',revision:1,reviewed:false,boxes:[],suggestions:[]},
  {id:'f2',width:640,height:480,bag:'record-b',stamp_ns:'222',split:'val',revision:1,reviewed:false,boxes:[],suggestions:[[50,60,100,120]]},
]};
let failSave=false;
let livePublished=false;
let selectionResult={method:'yolo',ranked:[],project_revision:1,stale:false};
const calls=[];
globalThis.fetch=async(url,options)=>{
  const body=options?.body?JSON.parse(options.body):undefined;calls.push({url,body});let result;
  if(url==='/api/perception')result={settings:{python:'/env/python',weights:'/weights/yolov8s.pt',device:'cpu'},environment:{ready:true,message:'準備完了'},
    tools_settings:{model_root:'/host/default',container_model_root:'/docker/default',encoder_repo:'',encoder_weights:''},
    projects:[{id:detail.id,name:detail.name,frames_count:2,reviewed_count:detail.frames.filter(f=>f.reviewed).length}],records:[{key:'record-a',name:'録画A'},{key:'record-b',name:'録画B'}],
    runs:[...(livePublished?[{id:'run-live',name:'新規学習',status:'succeeded',metrics:{epoch:50,map50:.9}}]:[]),{id:'run-a',name:'実験A',status:'succeeded',metrics:{epoch:50,map50:.8}}],models:[{id:'model-a',name:'モデルA',status:'succeeded',downloads:[{name:'opponent.onnx',url:'/perception-file?id=model-a&file=opponent.onnx'}]}]};
  else if(url.startsWith('/api/perception/project?'))result=detail;
  else if(url.startsWith('/api/perception/metrics?'))result={id:'run-a',metrics:[{epoch:1,precision:.6,recall:.5,map50:.4,map50_95:.2}]};
  else if(url.startsWith('/api/perception/selection?'))result=selectionResult;
  else if(url==='/api/perception/annotate'){
    if(failSave){detail.revision++;return {ok:false,json:async()=>({error:'プロジェクトの版が変わりました'})};}
    assert.equal(body.revision,detail.revision);const frame=detail.frames.find(f=>f.id===body.frame);assert.equal(body.frame_revision,frame.revision);
    assert.deepEqual(body.class_names,['rc_car','obstacle']);assert.equal(body.class_ids.length,body.boxes.length);
    Object.assign(frame,{boxes:body.boxes,class_ids:body.class_ids,reviewed:body.reviewed,revision:frame.revision+1});detail.revision++;result=detail;
  } else if(url==='/api/perception/settings')result=body;
  else if(url==='/api/perception/train')result={id:'job-live',title:'RCカー検出を学習',status:'running',artifact_id:'run-live'};
  else if(url==='/api/perception/push')result={id:'push-job',title:'RCカーONNX転送',status:'queued'};
  else result={id:'job-a',title:'RCカー学習',status:'running'};
  return {ok:true,json:async()=>structuredClone(result)};
};
const {emit,state}=await import('../frontend/api.js');
state.config={connection:{user:'kart',host:'jetson',port:2222}};
const {refreshPerception}=await import('../frontend/perception.js');
const flush=async()=>{for(let i=0;i<4;i++)await new Promise(resolve=>setImmediate(resolve));};
await refreshPerception();
assert.equal(calls.length,1,'opening the local detector catalog never connects to Jetson');
assert.equal(el('perception-project').children[1].textContent,'室内 <test> · 0/2確認済み','project names are literal text');
el('perception-project').value='project-a';await el('perception-project').trigger('change');await flush();
assert.match(el('perception-frame-info').textContent,/record-a.*111/);
assert.equal(el('perception-schema-warning').hidden,false);assert.match(el('perception-schema-warning').textContent,/旧1クラス.*RCカー.*再確認/);
assert.equal(el('perception-save').disabled,true,'unreviewed empty is not implicitly a negative label');
const canvas=el('perception-canvas');
const point=(x,y)=>({button:0,pointerId:1,clientX:150+x*1.25,clientY:100+y*1.25});
async function createBox(){await el('perception-tool-box').click();await canvas.trigger('pointerdown',point(20,30));await canvas.trigger('pointermove',point(120,130));await canvas.trigger('pointerup',point(120,130));}
await createBox();
assert.match(el('perception-boxes').children[0].textContent,/RCカー \(rc_car\)/);
el('perception-selected-class').value='1';await el('perception-selected-class').trigger('change');
assert.match(el('perception-boxes').children[0].textContent,/障害物 \(obstacle\)/);
await canvas.trigger('pointerdown',point(70,80));await canvas.trigger('pointermove',point(80,100));await canvas.trigger('pointerup',point(80,100));
assert.match(el('perception-boxes').children[0].textContent,/obstacle.*30.0, 50.0, 130.0, 150.0/);
await canvas.trigger('pointerdown',point(130,150));await canvas.trigger('pointermove',point(140,160));await canvas.trigger('pointerup',point(140,160));
assert.match(el('perception-boxes').children[0].textContent,/obstacle.*140.0, 160.0/);
await el('perception-undo').click();await el('perception-undo').click();await el('perception-undo').click();
assert.match(el('perception-boxes').children[0].textContent,/rc_car/,'undo restores a class-only edit together with geometry');
await el('perception-redo').click();assert.match(el('perception-boxes').children[0].textContent,/obstacle/);
assert.match(el('perception-box-count').textContent,/1\/100/);assert.equal(el('perception-draft-warning').hidden,false);
assert.equal(canvas.width,2000,'DPR only scales the backing store, not annotation pixels');
await el('perception-next').click();assert.match(el('perception-frame-info').textContent,/record-a/);
assert.match(el('perception-message').textContent,/未保存/);
el('perception-project').value='other-project';const beforeProject=calls.length;await el('perception-project').trigger('change');
assert.equal(calls.length,beforeProject,'dirty project switches never fetch and replace the draft');
assert.equal(el('perception-project').value,'project-a');
el('perception-page').hidden=true;await refreshPerception();el('perception-page').hidden=false;
assert.match(el('perception-box-count').textContent,/1\/100/,'navigation refresh keeps unsaved local boxes');
assert.equal(calls.filter(call=>call.url.startsWith('/api/perception/project?')).length,1);
let unloadPrevented=false;windowEvents.get('beforeunload')({preventDefault(){unloadPrevented=true;}});assert.equal(unloadPrevented,true);
failSave=true;await el('perception-save').click();
let annotation=calls.filter(call=>call.url==='/api/perception/annotate').at(-1).body;
assert.deepEqual(annotation.boxes,[[20,30,120,130]],'CSS letterbox and DPR still send original image pixel coordinates');
assert.deepEqual(annotation.class_ids,[1]);assert.deepEqual(annotation.class_names,['rc_car','obstacle']);
assert.equal(annotation.frame_revision,1);assert.equal(annotation.revision,1);assert.equal(annotation.reviewed,true);
assert.equal(el('perception-draft-warning').hidden,false,'a rejected revision retains the editable draft');
assert.match(el('perception-boxes').children[0].textContent,/obstacle/,'a save conflict preserves class IDs as well as coordinates');
assert.match(el('perception-message').textContent,/編集は保持.*一覧を更新/);
await el('perception-discard').click();failSave=false;await el('perception-refresh').click();await flush();
await createBox();await el('perception-save-next').click();await flush();
assert.deepEqual(calls.filter(call=>call.url==='/api/perception/annotate').at(-1).body.class_ids,[0]);
assert.match(el('perception-frame-info').textContent,/record-b.*222/);
assert.match(el('perception-box-count').textContent,/0\/100/,'prelabels remain suggestions instead of silently becoming ground truth');
assert.equal(el('perception-save').disabled,true);
await el('perception-negative').click();
annotation=calls.filter(call=>call.url==='/api/perception/annotate').at(-1).body;
assert.equal(annotation.frame,'f2');assert.deepEqual(annotation.boxes,[]);assert.deepEqual(annotation.class_ids,[]);assert.equal(annotation.reviewed,true);
el('perception-new-class').value='1';
await el('perception-accept').click();assert.match(el('perception-box-count').textContent,/1\/100/);
assert.match(el('perception-boxes').children[0].textContent,/obstacle/,'unclassified suggestions use the explicitly selected class in the draft');
assert.equal(el('perception-boxes').children[0].disabled,false,'sidebar selectors are enabled after an async action releases the busy state');
assert.equal(el('perception-draft-warning').hidden,false,'explicitly accepted candidates still need manual review and save');
await el('perception-boxes').children[0].click();await el('perception-delete').click();assert.match(el('perception-box-count').textContent,/0\/100/);
await el('perception-undo').click();assert.match(el('perception-boxes').children[0].textContent,/obstacle/,'delete undo restores the matching class');
await el('perception-undo').click();assert.match(el('perception-box-count').textContent,/0\/100/);assert.equal(el('perception-draft-warning').hidden,true);
const beforeInput=calls.length;windowEvents.get('keydown')({key:'Delete',target:{closest(){return true;}},preventDefault(){throw Error('input key must be ignored');}});assert.equal(calls.length,beforeInput);
await el('perception-train').click();assert.equal(calls.at(-1).url,'/api/perception/train');
assert.deepEqual(calls.at(-1).body,{id:'project-a',name:'',epochs:50,batch:8,imgsz:640,lr0:.001});
assert.equal(el('perception-run').value,'run-live','new training is selected before its artifact appears in the catalog');
assert.match(el('perception-run-status').textContent,/実行中/);assert.equal(el('perception-export').disabled,true);
await refreshPerception();assert.equal(el('perception-run').value,'run-live','a navigation/catalog refresh retains the pending run');
const metricCalls=calls.filter(call=>call.url.startsWith('/api/perception/metrics?')).length;
emit('status');await flush();assert.equal(calls.filter(call=>call.url.startsWith('/api/perception/metrics?')).length,metricCalls,'the new run waits four seconds for its first metric output');
now+=4001;emit('status');await flush();
assert.equal(calls.at(-1).url,'/api/perception/metrics?id=run-live');
assert.equal(el('perception-metrics').children[0].children[3].textContent,'0.400');
livePublished=true;const catalogs=calls.filter(call=>call.url==='/api/perception').length;
emit('job-finished',{id:'remote-job',title:'学習処理',resources:['perception:compute'],status:'succeeded'});await flush();
assert.equal(calls.filter(call=>call.url==='/api/perception').length,catalogs+1,'perception resource completion refreshes artifacts without a locally remembered job ID');
assert.equal(el('perception-run').value,'run-live');assert.equal(el('perception-export').disabled,false);
emit('job-finished',{id:'another-remote-job',title:'RCカー検出を学習',status:'succeeded'});await flush();
assert.equal(calls.filter(call=>call.url==='/api/perception').length,catalogs+2,'detector job titles from a restored page also refresh the catalog');
await el('perception-export').click();assert.deepEqual(calls.at(-1).body,{run:'run-live',name:''});
assert.equal(el('perception-models').children[0].children[1].href,'/perception-file?id=model-a&file=opponent.onnx');
// Selection is a navigation aid, never an annotation or a replacement for negatives.
detail.frames.push({id:'f3',width:640,height:480,bag:'record-c',stamp_ns:'333',split:'train',revision:1,reviewed:false,boxes:[],suggestions:[]});detail.revision++;
selectionResult={method:'yolo',project_revision:detail.revision,stale:false,ranked:[{frame_id:'f3',rank:1,score:.321,reason:'未検出探索 <tag>'}]};
await refreshPerception();
el('perception-selection-order').checked=true;await el('perception-selection-order').trigger('change');
assert.deepEqual(el('perception-frame').children.map(child=>child.value),['','f3','f1','f2'],'ranking keeps unranked positives and negative val images accessible');
assert.match(el('perception-frame-info').textContent,/record-c.*333/,'enabling ranking with a clean frame opens the highest-priority visible image');
el('perception-frame').value='f3';await el('perception-frame').trigger('change');await flush();
assert.match(el('perception-selection-frame').textContent,/#1.*0.321.*未検出探索 <tag>/);
assert.match(el('perception-selection-frame').textContent,/確率ではありません/);
const annotationsBeforeSelection=calls.filter(call=>call.url==='/api/perception/annotate').length;
selectionResult.annotations_changed=true;selectionResult.project_revision--;
await el('perception-selection-refresh').click();
assert.equal(el('perception-selection-order').checked,true,'annotation revisions inform the user without invalidating a usable ranking');
assert.match(el('perception-selection-summary').textContent,/前回順位を継続/);
el('perception-frame').value='f2';await el('perception-frame').trigger('change');await flush();
el('perception-selection-order').checked=false;await el('perception-selection-order').trigger('change');
await el('perception-tool-box').click();await canvas.trigger('pointerdown',point(20,30));
el('perception-selection-order').checked=true;await el('perception-selection-order').trigger('change');
assert.match(el('perception-frame-info').textContent,/record-b.*222/,'enabling ranking during a gesture must not switch away from the current frame');
await canvas.trigger('pointercancel');
el('perception-selection-order').checked=false;await el('perception-selection-order').trigger('change');
await createBox();const selectionsBeforeDirty=calls.filter(call=>call.url==='/api/perception/select').length;
el('perception-selection-order').checked=true;await el('perception-selection-order').trigger('change');
assert.match(el('perception-frame-info').textContent,/record-b.*222/,'enabling ranking keeps a dirty frame instead of losing its unsaved annotation');
assert.equal(el('perception-draft-warning').hidden,false);
await el('perception-select').click();assert.equal(calls.filter(call=>call.url==='/api/perception/select').length,selectionsBeforeDirty,'selection jobs cannot overwrite work associated with an unsaved draft');
selectionResult.stale=true;await el('perception-selection-refresh').click();
assert.equal(el('perception-selection-order').checked,false);assert.equal(el('perception-selection-order').disabled,true);
assert.match(el('perception-selection-summary').textContent,/古い結果/);
assert.equal(el('perception-draft-warning').hidden,false,'stale selection refresh preserves draft boxes');
assert.equal(calls.filter(call=>call.url==='/api/perception/annotate').length,annotationsBeforeSelection,'selection never writes annotations or reviewed flags');
await el('perception-discard').click();
el('perception-selection-limit').value='20';el('perception-selection-max-images').value='100';
await el('perception-select').click();assert.deepEqual(calls.at(-1).body,{id:'project-a',method:'yolo',limit:20,max_images:100,weights_source:'official',review_confidence:.10,expected_max_detections:3,large_bbox_area_ratio:.40});
for(const [key,value] of [['review-confidence','0.35'],['expected-max-detections','0'],['large-bbox-area-ratio','0.55']])el('perception-selection-'+key).value=value;
el('perception-selection-weights-source').value='run-live';await el('perception-select').click();
assert.equal(calls.at(-1).body.weights_source,'run-live','selection can explicitly use a completed local fine-tuned run');
assert.deepEqual([calls.at(-1).body.review_confidence,calls.at(-1).body.expected_max_detections,calls.at(-1).body.large_bbox_area_ratio],[.35,0,.55],'review thresholds preserve changed fractions and an explicit zero count');
el('perception-selection-method').value='dinov3';await el('perception-selection-method').trigger('change');
assert.equal(el('perception-selection-dino-fields').hidden,false);assert.equal(el('perception-selection-weights-source').disabled,true);
assert.equal(el('perception-selection-yolo-details').hidden,true);
for(const key of ['review-confidence','expected-max-detections','large-bbox-area-ratio'])assert.equal(el('perception-selection-'+key).disabled,true);
for(const [key,value] of [['encoder-repo','/models/dinov3'],['encoder-weights','/weights/dinov3.pth']]){el('perception-'+key).value=value;await el('perception-'+key).trigger('input');}
await el('perception-selection-environment').click();
assert.equal(el('perception-pane-train').hidden,false);
assert.equal(el('perception-environment-details').open,true,'DINO selection opens the environment setup controls');
assert.equal(el('perception-encoder-repo').value,'/models/dinov3','opening settings preserves DINO inputs');
await el('perception-tab-data').click();
await el('perception-select').click();assert.deepEqual(calls.at(-1).body,{id:'project-a',method:'dinov3',limit:20,max_images:100,encoder_repo:'/models/dinov3',encoder_weights:'/weights/dinov3.pth'});
// Transfer uses the shared SSH connection and only shows the returned build command.
for(const [key,value] of [['model-root','/host/custom'],['container-model-root','/workspaces/custom']]){el('perception-'+key).value=value;await el('perception-'+key).trigger('input');}
await refreshPerception();assert.equal(el('perception-model-root').value,'/host/custom','catalog refresh does not erase edits to tool-specific settings');
assert.equal(el('perception-encoder-repo').value,'/models/dinov3');
const push=el('perception-models').children[0].children.find(child=>child.dataset.model==='model-a');
assert.equal(push.disabled,false);await push.click();
assert.equal(calls.at(-1).url,'/api/perception/push');assert.deepEqual(calls.at(-1).body,{id:'model-a',connection:state.config.connection,model_root:'/host/custom',container_model_root:'/workspaces/custom'});
assert.match(el('perception-job-status').textContent,/待機中/);
const transfer={host_path:'/host/custom/model-unique/opponent.onnx',container_path:'/workspaces/custom/model-unique/opponent.onnx',build_command:'bash scripts/racing_trt.sh --source /workspaces/custom/model-unique/opponent.onnx'};
emit('job-finished',{id:'push-job',title:'RCカーONNX転送',status:'succeeded',resources:['read:perception:model:model-a'],result:transfer});await flush();
assert.equal(el('perception-push-result').hidden,false);assert.equal(el('perception-push-build-command').value,transfer.build_command);
assert.equal(el('perception-push-host-path').textContent,transfer.host_path);
let copied='';Object.defineProperty(globalThis,'navigator',{configurable:true,value:{clipboard:{async writeText(value){copied=value;}}}});
const beforeCopy=calls.length;await el('perception-push-copy').click();assert.equal(copied,transfer.build_command);assert.equal(calls.length,beforeCopy,'copying never executes a command or contacts Jetson');
state.jobs=[{id:'persisted-push',status:'succeeded',result:{...transfer,container_path:'/restored/opponent.onnx'}}];emit('status');await flush();
assert.equal(el('perception-push-container-path').textContent,'/restored/opponent.onnx','durable job results restore the latest completed transfer');
const hostOnly={host_path:'/host/only/model-unique/opponent.onnx',container_path:null,build_command:null};
emit('job-finished',{id:'push-host-only',title:'RCカーONNX転送',status:'succeeded',resources:['read:perception:model:model-a'],result:hostOnly});await flush();
assert.equal(el('perception-push-result').hidden,false);
assert.equal(el('perception-push-host-path').textContent,hostOnly.host_path,'a successful host-only transfer is visible without a build command');
assert.equal(el('perception-push-container-path').textContent,'未指定');
assert.equal(el('perception-push-build-command').value,'');assert.equal(el('perception-push-copy').disabled,true);
state.jobs=[{id:'restored-host-only',status:'succeeded',result:{...hostOnly,host_path:'/restored/host-only/opponent.onnx'}}];emit('status');await flush();
assert.equal(el('perception-push-host-path').textContent,'/restored/host-only/opponent.onnx','host-only durable results are also restored after reconnecting');
assert.equal(el('perception-push-build-command').value,'');assert.equal(el('perception-push-copy').disabled,true);
// Negative-and-next only navigates after a successful save, preserving the last image.
el('perception-frame').value='f1';await el('perception-frame').trigger('change');await flush();
failSave=true;await el('perception-negative-next').click();
assert.match(el('perception-frame-info').textContent,/record-a.*111/,'failed negative save stays on the same image');
assert.equal(el('perception-draft-warning').hidden,false);
failSave=false;await el('perception-discard').click();await el('perception-refresh').click();await flush();
await el('perception-negative-next').click();await flush();
annotation=calls.filter(call=>call.url==='/api/perception/annotate').at(-1).body;
assert.equal(annotation.frame,'f1');assert.deepEqual(annotation.boxes,[]);assert.deepEqual(annotation.class_ids,[]);assert.equal(annotation.reviewed,true);
assert.match(el('perception-frame-info').textContent,/record-b.*222/,'successful negative save advances to the next image');
await el('perception-negative-next').click();await flush();
assert.match(el('perception-frame-info').textContent,/record-c.*333/,'negative-and-next follows the current image order');
await el('perception-negative-next').click();await flush();
assert.match(el('perception-frame-info').textContent,/record-c.*333/,'last image stays selected after negative-and-next');
console.log('RC-car UI: drafts, class labels, negative-and-next, environment navigation, selection, learning/export and transfer passed');
