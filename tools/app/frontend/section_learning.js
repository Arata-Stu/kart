import { $, api, action, toast } from './api.js';
import { artifactOption, datasetOption, nextVersion } from './learning_names.js';

export function headCoverage(rows) {
  const routed = rows.filter(r => r.routing_sha256);
  if (!routed.length) return '単一head';
  if (routed.length !== rows.length || new Set(routed.map(r=>r.routing_sha256)).size !== 1)
    return '割当が異なります。全周データには先に「*」を割り当ててください';
  const routing=routed[0].routing;
  const counts=Object.fromEntries(routing.heads.map(h=>[h,0]));
  const mapping=Object.fromEntries(routing.sections.map(r=>[r.id,r.head]));
  for (const row of routed) for (const [section,count] of Object.entries(row.section_counts || {}))
    if (mapping[section] in counts) counts[mapping[section]] += count;
  return routing.heads.map(h=>`${h}: ${counts[h].toLocaleString()}${counts[h] ? '' : '（不足）'}`).join(' / ');
}

export function installSections({options,start}) {
  let catalog=null, maps=[], document=null, selectionKey=null;
  $("section-dataset-name").dataset.suggestion="section-data";
  $("section-pose-name").dataset.suggestion="section-poses";
  function route() {return catalog?.routings.find(r=>r.id===$('section-routing').value);}
  function currentLine() {
    const [id,kind]=$('section-line').value.split('/');
    const lane=document?.lanes.find(r=>r.id===id);
    return {lane,kind};
  }
  function mapSections() {
    const {lane,kind}=currentLine();
    return kind==='customline' ? (lane?.custom_speeds || []) : [];
  }
  function nextSectionId() {
    const used=new Set([...$('section-assignment-rows').querySelectorAll('.section-id')].map(input=>input.value));
    for(let i=1;i<1000;i++) {const id='s'+String(i).padStart(2,'0');if(!used.has(id))return id;}
    throw new Error('sectionの追加数が多すぎます');
  }
  function addRow(value={}) {
    const row=window.document.createElement('div');row.className='section-assignment-row';
    const label=window.document.createElement('label');label.textContent='対象区間';
    const target=window.document.createElement('select');target.className='section-target';
    target.add(new Option('距離で指定','manual'));target.add(new Option('* 全周（pose不要）','*'));
    mapSections().forEach((s,i)=>target.add(new Option(`map区間 ${i+1} · ${s.start.toFixed(2)}–${s.end.toFixed(2)} m`,String(i))));
    target.value=value.id==='*' ? '*' : value.map_section!==undefined ? String(value.map_section) : 'manual';
    label.append(target);row.append(label);
    for (const [key,title,initial] of [['id','section ID',value.id || nextSectionId()],
       ['head','head名',value.head || 'curve'],['start','開始 m',value.start_m??0],['end','終了 m',value.end_m??1]]) {
      const field=window.document.createElement('label');field.textContent=title;
      const input=window.document.createElement('input');input.className='section-'+key;input.value=initial;
      if (['start','end'].includes(key)) {input.type='number';input.step='0.01';input.min='0';}
      else input.maxLength=64;
      field.append(input);row.append(field);
    }
    const remove=window.document.createElement('button');remove.textContent='削除';remove.type='button';remove.className='quiet';
    remove.addEventListener('click',()=>row.remove());row.append(remove);
    function update() {
      const wildcard=target.value==='*', manual=target.value==='manual';
      row.querySelector('.section-id').disabled=wildcard;
      if(wildcard)row.querySelector('.section-id').value='*';
      else if(row.querySelector('.section-id').value==='*')row.querySelector('.section-id').value=nextSectionId();
      for(const key of ['start','end']) {
        const input=row.querySelector('.section-'+key);input.parentElement.hidden=!manual;
        if(!wildcard&&!manual)input.value=mapSections()[Number(target.value)][key];
      }
    }
    target.addEventListener('change',update);$('section-assignment-rows').append(row);update();
  }
  function assignments() {
    return [...$('section-assignment-rows').children].map(row=>{
      const target=row.querySelector('.section-target').value;
      const data={id:row.querySelector('.section-id').value.trim(),head:row.querySelector('.section-head').value.trim()};
      if(target==='*')return {id:'*',head:data.head};
      if(target==='manual')return {...data,start_m:Number(row.querySelector('.section-start').value),end_m:Number(row.querySelector('.section-end').value)};
      return {...data,map_section:Number(target)};
    });
  }
  async function loadMap() {
    const key=$('section-map').value;
    document=key ? await api('map?id='+encodeURIComponent(key)) : null;
    const lines=(document?.lanes || []).flatMap(lane=>Object.keys(lane.lines).map(kind=>({id:`${lane.id}/${kind}`,label:`${lane.id} / ${kind}`})));
    options('section-line',lines,'生成済みラインを選択');
    if(lines.length)$('section-line').value=lines.find(r=>r.id.endsWith('/customline'))?.id || lines[0].id;
    resetRows();
    if(!$('section-routing-name').value)$('section-routing-name').value=(key || 'course')+'-heads';
  }
  function resetRows() {
    $('section-assignment-rows').replaceChildren();
    const existing=mapSections();
    if(existing.length)existing.forEach((s,i)=>addRow({id:`s${String(i+1).padStart(2,'0')}`,map_section:i,head:'curve'}));
    else {
      const {lane,kind}=currentLine(), points=lane?.lines[kind]?.points || [];
      let length=0;
      for(let i=1;i<points.length;i++)length+=Math.hypot(points[i][0]-points[i-1][0],points[i][1]-points[i-1][1]);
      if(lane?.closed&&points.length)length+=Math.hypot(points[0][0]-points.at(-1)[0],points[0][1]-points.at(-1)[1]);
      addRow({id:'s01',head:'straight',start_m:0,end_m:length || 1});
    }
    addRow({id:'*',head:'stable'});
  }
  function suggestDatasetName() {
    const input=$('section-dataset-name');
    if(input.value && input.value!==input.dataset.suggestion)return;
    const source=catalog?.datasets.find(r=>r.id===$('section-source-dataset').value);
    if(!source)return;
    const suffix=$('section-operation').value==='assign' ? 'full-lap' : 'sections';
    input.value=nextVersion((source.name || source.id.split('/').at(-1)).slice(0,44)+'-'+suffix,catalog.datasets.map(r=>r.id));
    input.dataset.suggestion=input.value;
  }
  function refreshPicks() {
    const routing=route()?.routing, operation=$('section-operation').value;
    const sourceRows=(catalog?.datasets || []).filter(row=>operation==='extract'
      ? row.routing_sha256===route()?.routing_sha256 && Boolean(row.routing_sha256)
      : row.pipeline!=='multi' && !row.routing_sha256);
    options('section-source-dataset',sourceRows.map(datasetOption),operation==='extract'?'同じ割当のdatasetを選択':'元の教師datasetを選択');
    $('section-pose-label').hidden=operation!=='split';
    $('section-offline-step').hidden=operation!=='split';
    const key=$('section-routing').value+'/'+operation;
    const previous=key===selectionKey ? new Set([...$('section-picks').querySelectorAll('input:checked')].map(box=>box.value)) : null;
    selectionKey=key;
    $('section-picks').replaceChildren();
    const rows=operation==='assign' ? (routing?.sections || []).filter(r=>r.id==='*') : (routing?.sections || []).filter(r=>r.id!=='*');
    for(const row of rows) {
      const label=window.document.createElement('label');label.className='check';
      const box=window.document.createElement('input');box.type='checkbox';box.value=row.id;box.checked=previous ? previous.has(row.id) : true;box.disabled=operation==='assign';
      label.append(box,window.document.createTextNode(`${row.id==='*'?'* 全周':row.id} → ${row.head}`));$('section-picks').append(label);
    }
    $('section-routing-summary').textContent=routing ? `${routing.heads.length} heads · ${routing.heads.join(' / ')} · 保存元map: ${route().map}` : '使用する割当を選択してください';
    $('section-selection-help').textContent=operation==='assign' ? (rows.length ? '全周用headへ全教師サンプルを割り当てます。poseは使いません。' : 'この割当に「*」がありません。①で全周用headを追加して保存してください。') : '採用するsectionをチェック。専門head用の元datasetはheader時刻で作成してください。';
    suggestDatasetName();
    const poses=(catalog?.trajectories || []).filter(r=>r.routing_sha256===route()?.routing_sha256);
    options('section-trajectory',poses.map(r=>({...artifactOption(r),label:`${artifactOption(r).label} · 有効pose ${r.valid_poses}`})),'同じbagのposeを選択');
  }
  action('section-add-row',()=>addRow());
  action('section-save-routing',()=>{
    const [lane,line]=$('section-line').value.split('/');
    return start('section-routing',{name:$('section-routing-name').value,map:$('section-map').value,lane,line,sections:assignments()});
  });
  action('section-start-offline',()=>start('section-offline',{name:$('section-pose-name').value,
    routing:$('section-routing').value,map:route()?.map,bag:$('section-bag').value,tracking_mode:$('section-tracking-mode').value}));
  action('section-create-dataset',()=>start('section-dataset',{name:$('section-dataset-name').value,
    routing:$('section-routing').value,dataset:$('section-source-dataset').value,operation:$('section-operation').value,
    trajectory:$('section-trajectory').value,sections:[...$('section-picks').querySelectorAll('input:checked')].map(box=>box.value)}));
  $('section-map').addEventListener('change',()=>loadMap().catch(e=>toast(e.message,true)));
  $('section-line').addEventListener('change',resetRows);
  $('section-routing').addEventListener('change',refreshPicks);
  $('section-operation').addEventListener('change',refreshPicks);
  $('section-source-dataset').addEventListener('change',suggestDatasetName);
  $('section-bag').addEventListener('change',()=>{
    const input=$('section-pose-name');
    if(!input.value || input.value===input.dataset.suggestion) {
      const base=$('section-bag').value.split('/').at(-1).slice(0,48)+'-poses';
      input.value=nextVersion(base,(catalog?.trajectories || []).map(r=>r.id));input.dataset.suggestion=input.value;
    }
  });
  return async function refresh(data) {
    catalog=data;
    options('section-routing',(data.routings || []).map(artifactOption),'割当を選択');
    maps=await api('maps');options('section-map',maps.filter(r=>r.has_vslam&&r.snapshot_status!=='pending').map(r=>({...r,label:r.title || r.id})),'既存mapを選択');
    refreshPicks();
  };
}
