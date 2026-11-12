import { $, state, api, on, emit, toast } from './api.js';
import { AnnotationDraft, MAX_BOXES, copyBoxes, normalizeBox, fitView, imagePoint,
  screenPoint, zoomView, boxHandles, hitBox, moveBox, resizeBox, filteredFrames, orderedFrames, projectProgress } from './perception_model.js';

let catalog=null, project=null, frameId=null, draft=null, image=null, imageEpoch=0;
let busy=false, selected=-1, tool='edit', view=null, gesture=null, space=false;
let settingsDirty=false, settingsLoaded=false, catalogEpoch=0, projectEpoch=0, currentJob=null, metricAt=0, metricsBusy=false;
let step='data';
const pendingRuns=new Map();
let selection=null, selectionOrder=false, selectionEpoch=0, lastPushJob=null;
const pushButtons=new Set(), toolEdits=new Set();
const toolFields={model_root:'model-root',container_model_root:'container-model-root',encoder_repo:'encoder-repo',encoder_weights:'encoder-weights'};
const canvas=$('perception-canvas'), ctx=canvas.getContext('2d');
const frame=()=>project?.frames.find(value=>value.id===frameId);
const visibleFrames=()=>orderedFrames(filteredFrames(project?.frames,$('perception-filter').value,$('perception-filter-split').value),selection,selectionOrder);
const number=id=>Number($(id).value);
const message=text=>{$('perception-message').textContent=text;};
const jobLabels={queued:'待機中',running:'実行中',succeeded:'完了',failed:'失敗',cancelled:'中止',interrupted:'中断',orphaned:'管理喪失'};

function controls() {
  const available=!!frame(), dirty=!!draft?.dirty;
  $('perception-draft-warning').hidden=!dirty;
  $('perception-save-state').textContent=dirty?'未保存':available?(frame().reviewed?'確認済み・保存済み':'未確認・未review'):'画像なし';
  for(const id of ['create','project','extract','prelabel','train','select','selection-refresh','selection-save-settings','settings-save','setup','export','refresh'])
    $('perception-'+id).disabled=busy || (['extract','prelabel','train','select','selection-refresh'].includes(id) && !project);
  $('perception-export').disabled=busy || !(catalog?.runs || []).some(run=>run.id===$('perception-run').value && run.status==='succeeded');
  for(const id of ['save','save-next'])$('perception-'+id).disabled=busy || !available || !draft.boxes.length;
  for(const id of ['negative','fit','tool-edit','tool-box'])$('perception-'+id).disabled=busy || !available;
  for(const id of ['prev','next','frame','filter','filter-split'])$('perception-'+id).disabled=busy || !project?.frames.length;
  $('perception-discard').disabled=busy || !dirty;
  $('perception-delete').disabled=busy || !draft?.boxes[selected];
  $('perception-undo').disabled=busy || !draft?.undoStack.length;
  $('perception-redo').disabled=busy || !draft?.redoStack.length;
  $('perception-accept').disabled=busy || !frame()?.suggestions?.length;
  for(const name of ['edit','box'])$('perception-tool-'+name).setAttribute('aria-pressed',String(tool===name));
  for(const button of $('perception-boxes').children)button.disabled=busy;
  for(const id of ['project-name','bag','extract-split','topic','sample-hz','max-frames','confidence',
    'python','weights','device','train-name','epochs','batch','imgsz','lr0','run','export-name',
    'selection-method','selection-limit','selection-max-images','model-root','container-model-root'])$('perception-'+id).disabled=busy;
  const dino=$('perception-selection-method').value==='dinov3';
  $('perception-selection-dino-fields').hidden=!dino;$('perception-selection-yolo-fields').hidden=dino;
  $('perception-selection-yolo-details').hidden=dino;
  for(const id of ['encoder-repo','encoder-weights'])$('perception-'+id).disabled=busy || !dino;
  $('perception-selection-weights-source').disabled=busy || dino;
  for(const id of ['selection-review-confidence','selection-expected-max-detections','selection-large-bbox-area-ratio'])$('perception-'+id).disabled=busy || dino;
  $('perception-selection-order').disabled=busy || !selection?.ranked?.length || !!selection?.stale;
  $('perception-push-copy').disabled=busy || !$('perception-push-build-command').value;
  const connection=state.config?.connection;
  for(const button of pushButtons)button.disabled=busy || !connection?.host || !connection?.user;
  $('perception-push-target').textContent=connection?.host && connection?.user
    ? `${connection.user}@${connection.host}:${connection.port || 22} · 上部「接続設定」と共通`
    : '上部「接続設定」でJetsonの接続先を指定してください。';
}
async function perform(fn) {
  if(busy)return;
  busy=true;controls();
  try {await fn();}
  catch(error) {message(error.message);toast(error.message,true);}
  finally {busy=false;controls();}
}
function bind(id,fn) {$('perception-'+id).addEventListener('click',()=>perform(fn));}
function cleanRequired() {
  if(gesture)throw Error('画像のドラッグ操作を終えてから切り替えてください。');
  if(draft?.dirty)throw Error('未保存の編集を保持しています。先に枠を保存するか「この画像の編集を破棄」を選んでください。');
}
function options(id,rows,selectedValue,empty='選択してください') {
  const select=$(id);
  select.replaceChildren(new Option(empty,''),...rows.map(([value,label])=>new Option(label,value)));
  select.value=rows.some(row=>row[0]===selectedValue)?selectedValue:'';
}
function renderCatalog() {
  options('perception-project',(catalog.projects || []).map(p=>[p.id,
    `${p.name} · ${p.reviewed_count ?? p.reviewed ?? 0}/${p.frames_count ?? p.frames ?? 0}確認済み`]),project?.id);
  options('perception-bag',(catalog.records || []).map(record=>[record.key || record.id,record.name]),$('perception-bag').value,'Notebookへ転送したbagを選択');
  options('perception-run',(catalog.runs || []).map(run=>[run.id,`${run.name || run.title || run.id} · ${jobLabels[run.status] || run.status}`]),$('perception-run').value || catalog.runs?.[0]?.id,'学習実験なし');
  options('perception-selection-weights-source',[
    ['official','公式YOLOv8sの重み'],...(catalog.runs || []).filter(run=>run.status==='succeeded').map(run=>[run.id,run.name || run.title || run.id]),
  ],$('perception-selection-weights-source').value || 'official','重みを選択');
  for(const [key,id] of Object.entries(toolFields))if(!toolEdits.has(key) && typeof catalog.tools_settings?.[key]==='string')$('perception-'+id).value=catalog.tools_settings[key];
  if(!settingsLoaded || !settingsDirty) {
    for(const key of ['python','weights','device'])$('perception-'+key).value=catalog.settings?.[key] || '';
    settingsLoaded=true;
  }
  $('perception-environment').textContent=catalog.environment?.message || '環境の準備状況は未確認です';
  const models=$('perception-models');models.replaceChildren();pushButtons.clear();
  for(const model of catalog.models || []) {
    const row=document.createElement('div');row.className='perception-model';
    const title=document.createElement('strong');title.textContent=`${model.name || model.title || model.id} · ${jobLabels[model.status] || model.status}`;row.append(title);
    for(const download of model.status==='succeeded'?model.downloads || []:[]) {
      // Download routes are served by this local application, never arbitrary URLs.
      if(typeof download.url!=='string' || !download.url.startsWith('/perception-file?'))continue;
      const link=document.createElement('a');link.href=download.url;link.textContent=download.name;link.setAttribute('download','');row.append(link);
    }
    if(model.status==='succeeded') {
      const push=document.createElement('button');push.textContent='JetsonへONNX転送';push.dataset.model=model.id;
      push.addEventListener('click',()=>perform(()=>startJob('push',{id:model.id,connection:structuredClone(state.config.connection),
        model_root:$('perception-model-root').value.trim(),container_model_root:$('perception-container-model-root').value.trim()})));
      pushButtons.add(push);row.append(push);
    }
    models.append(row);
  }
  if(!catalog.models?.length)models.textContent='ONNXを書き出すと、ここからダウンロードできます。';
  renderRun();controls();
}
export async function refreshPerception() {
  const token=++catalogEpoch,result=await api('perception');
  if(token!==catalogEpoch)return;
  result.runs ||= [];
  for(const [id,run] of pendingRuns) {
    if(result.runs.some(value=>value.id===id))pendingRuns.delete(id);
    else result.runs.unshift(run);
  }
  catalog=result;renderCatalog();
  if(project && !draft?.dirty && !gesture)await loadProject(project.id,true);
  else if(project)await refreshSelection();
  else if(!project)message('プロジェクトを作成するか、保存済みのプロジェクトを選んでください。');
}
async function loadProject(id,preserve=false) {
  cleanRequired();
  const token=++projectEpoch;
  const detail=await api('perception/project?id='+encodeURIComponent(id));
  if(token!==projectEpoch)return;
  cleanRequired();
  useProject(detail,preserve?frameId:null);
  await refreshSelection();
  if(!preserve && detail.frames.length)showStep('annotate');
}
function showStep(name) {
  step=name;$('perception-page').dataset.step=name;
  for(const value of ['data','annotate','train']) {
    $('perception-pane-'+value).hidden=value!==name;
    $('perception-tab-'+value).setAttribute('aria-pressed',String(value===name));
  }
  if(name==='annotate')fit();
}
function useProject(detail,preferred=null) {
  projectEpoch++;
  selectionEpoch++;
  if(project?.id!==detail.id){selection=null;selectionOrder=false;}
  project=detail;
  const chosen=detail.frames.find(value=>value.id===preferred) || filteredFrames(detail.frames,$('perception-filter').value,$('perception-filter-split').value)[0] || detail.frames[0];
  frameId=null;draft=null;selected=-1;image=null;gesture=null;
  $('perception-project').value=detail.id;
  setFrame(chosen?.id || null);
  message(`${detail.name} · rc_car 1クラス`);
}
function renderFrames() {
  const progress=projectProgress(project?.frames || []);
  $('perception-progress').textContent=`${progress.reviewed}/${progress.total}確認済み · RCカーあり ${progress.positive} · 対象なし ${progress.negative}`;
  $('perception-project-summary').textContent=project?`${project.name} · train ${progress.train}画像 / val ${progress.val}画像 · 版 ${project.revision}`:'';
  const rows=visibleFrames(),outside=frame() && !rows.some(value=>value.id===frameId);
  if(outside)rows.unshift(frame());
  options('perception-frame',rows.map((value,i)=>[value.id,`${outside && value.id===frameId?'現在の画像（表示条件外）':`${i+1}/${rows.length}`} · ${value.split} · ${value.reviewed?(value.boxes.length?'RCカーあり':'対象なし'):'未確認'} · ${value.id}`]),frameId,'該当する画像なし');
  const source=frame();
  $('perception-frame-info').textContent=source
    ? `${source.split==='val'?'検証用 val':'学習用 train'} · bag: ${source.bag} · stamp: ${source.stamp_ns} ns · ${source.width}×${source.height}px · ${source.reviewed?'確認済み':'未確認'}`
    : 'bagから画像を抽出してください。';
  renderBoxes();controls();
  renderSelection();
}
function renderSelection() {
  const ranked=selection?.ranked || [],changed=selection?.annotations_changed ||
    (selection?.project_revision!=null && project?.revision!==selection.project_revision);
  if(selection?.stale)selectionOrder=false;
  $('perception-selection-order').checked=selectionOrder;
  $('perception-selection-summary').textContent=!ranked.length?'選定結果はまだありません。通常の全画像からも確認できます。'
    : `${selection.method==='dinov3'?'DINOv3':'YOLO'} · train未確認 ${ranked.length}画像の優先順位 · `+
      (selection.stale?'画像集合が更新されたため古い結果です。再計算してください。現在は通常順です。'
        : (changed?'枠の確認結果が更新されています。前回順位を継続できます。再選定すると新しい確認結果を反映します。':'選定結果を取得済みです。')+
          ' 未選定・対象なし・valの画像も通常どおり確認できます。');
  const row=ranked.find(value=>value.frame_id===frameId);
  $('perception-selection-frame').textContent=row
    ? `${selection.stale?'古い選定結果 · ':''}選定 #${row.rank} · score ${Number.isFinite(row.score)?row.score.toFixed(3):'—'} · 理由: ${Array.isArray(row.reason)?row.reason.join(' / '):row.reason || '記録なし'}（scoreは確率ではありません）`
    : selectionOrder?'選定順位外の画像です。対象なし・未選定・valの画像も確認できます。':'';
}
async function refreshSelection() {
  if(!project)return;
  const id=project.id,token=++selectionEpoch;
  const result=await api('perception/selection?id='+encodeURIComponent(id));
  if(token!==selectionEpoch || project?.id!==id)return;
  selection={...result,ranked:Array.isArray(result.ranked)?result.ranked:[]};
  if(selection.stale)selectionOrder=false;
  renderFrames();
}
function renderPushResult(job) {
  const result=job?.result;
  if(job?.status!=='succeeded' || !result || typeof result.host_path!=='string' || !result.host_path || job.id===lastPushJob)return;
  lastPushJob=job.id;
  $('perception-push-host-path').textContent=result.host_path || '';
  $('perception-push-container-path').textContent=result.container_path || '未指定';
  $('perception-push-build-command').value=typeof result.build_command==='string'?result.build_command:'';
  $('perception-push-copy').disabled=busy || !$('perception-push-build-command').value;
  $('perception-push-result').hidden=false;
}
function renderBoxes() {
  const list=$('perception-boxes');list.replaceChildren();
  for(const [index,box] of (draft?.boxes || []).entries()) {
    const button=document.createElement('button');button.textContent=`${index+1}. rc_car [${box.map(value=>value.toFixed(1)).join(', ')}]`;
    button.className=index===selected?'active':'';button.disabled=busy;
    button.addEventListener('click',()=>{selected=index;tool='edit';renderBoxes();controls();draw();});list.append(button);
  }
  $('perception-box-count').textContent=`正解枠 ${draft?.boxes.length || 0}/${MAX_BOXES}個`;
  $('perception-suggestions-note').textContent=frame()?.suggestions?.length?`自動候補 ${frame().suggestions.length}個（まだ正解ではありません）`:'自動候補なし';
}
function setFrame(id) {
  cleanRequired();
  frameId=id;draft=frame()?new AnnotationDraft(frame()):null;selected=-1;image=null;view=null;gesture=null;
  const token=++imageEpoch;
  $('perception-image-empty').hidden=false;
  $('perception-image-empty').textContent=id?'画像を読み込み中…':'画像を抽出し、確認する画像を選んでください。';
  renderFrames();draw();
  if(!id)return;
  const loading=new Image();
  loading.onload=()=>{
    if(token!==imageEpoch)return;
    if(loading.naturalWidth!==frame().width || loading.naturalHeight!==frame().height) {
      $('perception-image-empty').textContent='画像サイズが保存データと一致しません。画像を再取得してください。';return;
    }
    image=loading;$('perception-image-empty').hidden=true;fit();
  };
  loading.onerror=()=>{if(token===imageEpoch)$('perception-image-empty').textContent='画像を読み込めません。一覧を更新して確認してください。';};
  loading.src='/perception-image?id='+encodeURIComponent(project.id)+'&frame='+encodeURIComponent(id);
}
function navigateFrame(delta) {
  cleanRequired();
  const rows=visibleFrames(),index=rows.findIndex(value=>value.id===frameId);
  const next=rows[index+delta];if(next)setFrame(next.id);
}
function dimensions() {
  const rect=canvas.getBoundingClientRect();return {width:Math.max(1,rect.width),height:Math.max(1,rect.height)};
}
function fit() {
  if(!frame())return;
  const size=dimensions();view=fitView(frame().width,frame().height,size.width,size.height);draw();
}
function drawBox(box,color,dashed=false,chosen=false) {
  const [x,y]=screenPoint(view,box[0],box[1]),width=(box[2]-box[0])*view.scale,height=(box[3]-box[1])*view.scale;
  ctx.strokeStyle=color;ctx.lineWidth=chosen?2.5:1.5;ctx.setLineDash(dashed?[6,4]:[]);ctx.strokeRect(x,y,width,height);
  if(chosen)for(const point of Object.values(boxHandles(box))) {
    const p=screenPoint(view,...point);ctx.fillStyle='white';ctx.fillRect(p[0]-4,p[1]-4,8,8);ctx.strokeRect(p[0]-4,p[1]-4,8,8);
  }
}
function draw() {
  const size=dimensions(),ratio=globalThis.devicePixelRatio || 1;
  canvas.width=Math.round(size.width*ratio);canvas.height=Math.round(size.height*ratio);
  ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,size.width,size.height);
  if(!image || !frame())return;
  if(!view)view=fitView(frame().width,frame().height,size.width,size.height);
  ctx.drawImage(image,view.x,view.y,frame().width*view.scale,frame().height*view.scale);
  for(const box of frame().suggestions || [])drawBox(box,'#ffb950',true);
  const boxes=gesture?.boxes || draft.boxes;
  boxes.forEach((box,index)=>drawBox(box,index===selected?'#75f7d0':'#39dcb0',false,index===selected));
  if(gesture?.kind==='create' && gesture.preview)drawBox(gesture.preview,'#75f7d0');
  ctx.setLineDash([]);
}
const pointer=event=>{const rect=canvas.getBoundingClientRect();return [event.clientX-rect.left,event.clientY-rect.top];};
canvas.addEventListener('pointerdown',event=>{
  if(busy || !image || !draft || (event.button!==0 && event.button!==1))return;
  const point=pointer(event),position=imagePoint(view,...point);
  canvas.focus();canvas.setPointerCapture(event.pointerId);
  if(space || event.button===1)gesture={kind:'pan',start:point,view:{...view}};
  else if(tool==='box') {
    if(draft.boxes.length>=MAX_BOXES)return toast(`矩形は${MAX_BOXES}個までです`,true);
    gesture={kind:'create',start:position,preview:null};selected=-1;
  } else {
    const hit=hitBox(draft.boxes,position,selected,7/view.scale);
    selected=hit?.index ?? -1;
    if(hit)gesture={kind:hit.handle?'resize':'move',handle:hit.handle,start:position,original:copyBoxes(draft.boxes),boxes:copyBoxes(draft.boxes)};
  }
  event.preventDefault();renderBoxes();controls();draw();
});
canvas.addEventListener('pointermove',event=>{
  if(!gesture)return;
  const point=pointer(event),position=imagePoint(view,...point);
  if(gesture.kind==='pan')view={...gesture.view,x:gesture.view.x+point[0]-gesture.start[0],y:gesture.view.y+point[1]-gesture.start[1]};
  else if(gesture.kind==='create')gesture.preview=normalizeBox([...gesture.start,...position],frame().width,frame().height);
  else if(gesture.kind==='move')gesture.boxes[selected]=moveBox(gesture.original[selected],position[0]-gesture.start[0],position[1]-gesture.start[1],frame().width,frame().height);
  else {
    const box=resizeBox(gesture.original[selected],gesture.handle,position,frame().width,frame().height);
    if(box)gesture.boxes[selected]=box;
  }
  draw();
});
function finishGesture(cancel=false) {
  if(!gesture)return;
  const value=gesture;gesture=null;
  if(!cancel) {
    if(value.kind==='create' && value.preview) {draft.change([...draft.boxes,value.preview]);selected=draft.boxes.length-1;tool='edit';}
    else if(value.boxes)draft.change(value.boxes);
  }
  renderBoxes();controls();draw();
}
canvas.addEventListener('pointerup',()=>finishGesture());
canvas.addEventListener('pointercancel',()=>finishGesture(true));
canvas.addEventListener('wheel',event=>{
  if(!image || !view || busy || gesture)return;
  event.preventDefault();const size=dimensions(),base=fitView(frame().width,frame().height,size.width,size.height).scale;
  view=zoomView(view,Math.exp(-event.deltaY*.001),pointer(event),base);draw();
},{passive:false});
function deleteSelected() {
  if(!draft?.boxes[selected])return;
  draft.change(draft.boxes.filter((_,index)=>index!==selected));selected=-1;renderBoxes();controls();draw();
}
function undo(redo=false) {draft?.undo(redo);selected=-1;renderBoxes();controls();draw();}
globalThis.addEventListener?.('keydown',event=>{
  if($('perception-page').hidden || busy || event.target?.closest?.('input,textarea,select,[contenteditable=true]'))return;
  if(event.code==='Space'){space=true;event.preventDefault();return;}
  if(event.key==='Escape'){finishGesture(true);selected=-1;tool='edit';renderBoxes();controls();draw();return;}
  if(gesture)return;
  if((event.ctrlKey || event.metaKey) && event.key.toLowerCase()==='z'){event.preventDefault();undo(event.shiftKey);return;}
  if(event.key==='Delete' || event.key==='Backspace'){event.preventDefault();deleteSelected();}
  if(event.key==='[' || event.key===']') {event.preventDefault();try{navigateFrame(event.key==='['?-1:1);}catch(error){message(error.message);toast(error.message,true);}}
});
globalThis.addEventListener?.('keyup',event=>{if(event.code==='Space')space=false;});
globalThis.addEventListener?.('blur',()=>{space=false;finishGesture(true);});
globalThis.addEventListener?.('beforeunload',event=>{if(draft?.dirty){event.preventDefault();event.returnValue='';}});
if(globalThis.ResizeObserver)new ResizeObserver(()=>{if(!gesture)fit();}).observe(canvas.parentElement);
else globalThis.addEventListener?.('resize',()=>fit());

async function saveReviewed(negative=false,next=false) {
  if(!draft || !frame())throw Error('画像を選択してください');
  if(!negative && !draft.boxes.length)throw Error('枠がない画像は「対象なしとして保存」で確認してください');
  finishGesture();
  const rows=visibleFrames(),position=rows.findIndex(value=>value.id===frameId),nextId=rows[position+1]?.id;
  draft.change(negative?[]:draft.boxes,true);controls();
  let result;
  try {
    result=await api('perception/annotate',{id:project.id,frame:frameId,revision:project.revision,
      frame_revision:frame().revision,boxes:copyBoxes(draft.boxes),reviewed:true});
  } catch(error) {
    throw Error(error.message+' 編集は保持しています。版の競合時は枠の座標を控え、編集を破棄して「一覧を更新」から保存済みの内容を再取得してください。');
  }
  useProject(result,next?(nextId || frameId):frameId);
  message(negative?'対象なしの確認結果を保存しました。':'rc_carの枠を確認済みとして保存しました。');
}
async function saveSettings() {
  const values=Object.fromEntries(['python','weights','device'].map(key=>[key,$('perception-'+key).value.trim()]));
  await api('perception/settings',values);settingsDirty=false;
}
async function startJob(action,body) {
  if(draft?.dirty && ['extract','prelabel','train','select'].includes(action))cleanRequired();
  if(settingsDirty && action!=='push')throw Error('変更した学習環境の設定を先に保存してください');
  const job=await api('perception/'+action,body);currentJob=job.id;emit('job-started',job);
  if(action==='train' && typeof job.artifact_id==='string') {
    const run={id:job.artifact_id,name:body.name || '新しいRCカー学習',status:'running',job_id:job.id};
    pendingRuns.set(run.id,run);
    catalog.runs=[run,...(catalog.runs || []).filter(value=>value.id!==run.id)];
    renderCatalog();$('perception-run').value=run.id;renderRun();controls();
  }
  $('perception-job-status').textContent=`${job.title || action}: ${jobLabels[job.status] || job.status} · 詳細は「ログ・履歴」`;
  message(job.status==='queued'?'処理をキューへ追加しました。ログ・履歴から待機状況と中止を確認できます。':'処理を開始しました。ログ・履歴から進捗と中止を確認できます。');
}
function renderRun() {
  const run=catalog?.runs?.find(value=>value.id===$('perception-run').value);
  $('perception-run-status').textContent=run?`${run.name || run.title || run.id} · ${jobLabels[run.status] || run.status}`:'学習実験を選択してください。';
  renderMetrics(run?.metrics?[run.metrics]:[]);
  metricAt=pendingRuns.has(run?.id)?Date.now():0;
}
function renderMetrics(rows) {
  const body=$('perception-metrics');body.replaceChildren();
  for(const row of rows.slice(-20)) {
    const tr=document.createElement('tr');
    for(const key of ['epoch','precision','recall','map50','map50_95']) {
      const td=document.createElement('td');td.textContent=Number.isFinite(row[key])?(key==='epoch'?String(row[key]):row[key].toFixed(3)):'—';tr.append(td);
    }
    body.append(tr);
  }
}
async function refreshMetrics() {
  const id=$('perception-run').value;
  if(!id || metricsBusy || Date.now()-metricAt<4000)return;
  metricsBusy=true;metricAt=Date.now();
  try {
    const result=await api('perception/metrics?id='+encodeURIComponent(id));
    if(pendingRuns.has(id) && result.metrics?.length)pendingRuns.get(id).metrics=result.metrics.at(-1);
    if(id===$('perception-run').value)renderMetrics(result.metrics || []);
  }
  catch(error){$('perception-run-status').textContent='検証値の取得に失敗: '+error.message;}
  finally {metricsBusy=false;}
}

bind('refresh',refreshPerception);
for(const name of ['data','annotate','train'])bind('tab-'+name,()=>showStep(name));
bind('create',async()=>{cleanRequired();const result=await api('perception/create',{name:$('perception-project-name').value.trim()});useProject(result);await refreshPerception();});
bind('extract',()=>startJob('extract',{id:project.id,bag:$('perception-bag').value,split:$('perception-extract-split').value,
  topic:$('perception-topic').value.trim(),sample_hz:number('perception-sample-hz'),max_frames:number('perception-max-frames')}));
bind('prelabel',()=>startJob('prelabel',{id:project.id,confidence:number('perception-confidence')}));
bind('select',()=>{
  const method=$('perception-selection-method').value;
  return startJob('select',{id:project.id,method,limit:number('perception-selection-limit'),max_images:number('perception-selection-max-images'),
    ...(method==='dinov3'?{encoder_repo:$('perception-encoder-repo').value.trim(),encoder_weights:$('perception-encoder-weights').value.trim()}
      : {weights_source:$('perception-selection-weights-source').value,
        review_confidence:number('perception-selection-review-confidence'),
        expected_max_detections:number('perception-selection-expected-max-detections'),
        large_bbox_area_ratio:number('perception-selection-large-bbox-area-ratio')})});
});
bind('selection-refresh',refreshSelection);
bind('selection-save-settings',async()=>{
  const values=Object.fromEntries(Object.entries(toolFields).map(([key,id])=>[key,$('perception-'+id).value.trim()]));
  await api('perception/tools-settings',values);toolEdits.clear();message('選定・転送先の設定を保存しました。');
});
bind('train',()=>startJob('train',{id:project.id,name:$('perception-train-name').value.trim(),
  epochs:number('perception-epochs'),batch:number('perception-batch'),imgsz:number('perception-imgsz'),lr0:number('perception-lr0')}));
bind('export',()=>startJob('export',{run:$('perception-run').value,name:$('perception-export-name').value.trim()}));
bind('settings-save',async()=>{await saveSettings();await refreshPerception();message('学習環境の設定を保存しました。');});
bind('setup',async()=>{await saveSettings();await startJob('setup',{});});
bind('prev',()=>navigateFrame(-1));bind('next',()=>navigateFrame(1));
bind('save',()=>saveReviewed());bind('save-next',()=>saveReviewed(false,true));bind('negative',()=>saveReviewed(true));
bind('discard',()=>{draft?.discard();selected=-1;renderBoxes();controls();draw();message('この画像の編集を保存済みの状態へ戻しました。');});
bind('accept',()=>{draft.change(copyBoxes(frame().suggestions));selected=-1;renderBoxes();draw();message('候補を正解枠へ取り込みました。枠を確認・修正してから保存してください。');});
bind('undo',()=>undo());bind('redo',()=>undo(true));bind('delete',deleteSelected);bind('fit',fit);
for(const name of ['edit','box'])bind('tool-'+name,()=>{tool=name;});
bind('open-bringup',()=>$('nav-bringup').click());bind('open-transfer',()=>$('nav-transfer').click());
bind('jobs',()=>{if($('jobs-drawer').hidden)$('jobs-toggle').click();});
bind('push-connection',()=>$('connection-open').click());
bind('push-copy',async()=>{
  const input=$('perception-push-build-command');
  if(!input.value)throw Error('コピーするbuildコマンドがありません');
  if(globalThis.navigator?.clipboard?.writeText)await navigator.clipboard.writeText(input.value);
  else {input.focus();input.select();if(!document.execCommand?.('copy'))throw Error('コマンド欄を選択してコピーしてください');}
  toast('buildコマンドをコピーしました。Jetson上での実行は手動です。');
});
for(const [key,id] of Object.entries(toolFields))$('perception-'+id).addEventListener('input',()=>toolEdits.add(key));
$('perception-selection-method').addEventListener('change',controls);
$('perception-selection-order').addEventListener('change',()=>{
  const previous=selectionOrder;
  selectionOrder=$('perception-selection-order').checked && !selection?.stale;
  if(selectionOrder && !previous && !draft?.dirty && !gesture) {
    const first=visibleFrames()[0];
    if(first && first.id!==frameId){setFrame(first.id);return;}
  }
  renderFrames();
});
for(const key of ['python','weights','device'])$('perception-'+key).addEventListener('input',()=>{settingsDirty=true;});
$('perception-project').addEventListener('change',async()=>{
  if(busy)return;
  busy=true;controls();
  try {
    if($('perception-project').value)await loadProject($('perception-project').value);
    else $('perception-project').value=project?.id || '';
  }
  catch(error){$('perception-project').value=project?.id || '';message(error.message);toast(error.message,true);}
  finally {busy=false;controls();}
});
$('perception-frame').addEventListener('change',()=>{
  try {setFrame($('perception-frame').value || null);}
  catch(error){$('perception-frame').value=frameId;message(error.message);toast(error.message,true);}
});
for(const id of ['filter','filter-split'])$('perception-'+id).addEventListener('change',()=>{
  renderFrames();
  if(!visibleFrames().some(value=>value.id===frameId) && !draft?.dirty)setFrame(visibleFrames()[0]?.id || null);
});
$('perception-run').addEventListener('change',()=>{renderRun();controls();refreshMetrics();});
on('status',()=>{
  controls();if($('perception-page').hidden)return;
  for(const job of state.jobs)if(job.status==='succeeded' && typeof job.result?.host_path==='string'){renderPushResult(job);break;}
  const job=state.jobs.find(value=>value.id===currentJob);
  if(job)$('perception-job-status').textContent=`${job.title}: ${jobLabels[job.status] || job.status}${job.message?' · '+job.message:''}`;
  refreshMetrics();
});
on('job-finished',event=>{
  const job=event.detail;
  renderPushResult(job);
  if(!job?.resources?.some(value=>typeof value==='string' && value.includes('perception:')) &&
      !/認識|検出|選定|ONNX転送/.test(job?.title || '') && job?.id!==currentJob)return;
  for(const [id,run] of pendingRuns)if(run.job_id===job.id)pendingRuns.delete(id);
  if(!$('perception-page').hidden)refreshPerception().catch(error=>message(error.message));
});
showStep(step);controls();
