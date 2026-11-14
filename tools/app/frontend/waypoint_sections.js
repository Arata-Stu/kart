import { $, state } from "./api.js";
import { covers, sectionColor, hasOverlap } from "./custom_model.js";
import { sectionKindLabel } from "./custom_sections.js";
import { fitViewport, projectViewport, unprojectViewport, zoomViewport, panViewport, pointAtStation, nearestPathStation } from "./map_viewport.js";
export function waypointStations(points, closed) {
  const stations = [0];
  for (let i = 1; i < points.length; i++)
    stations.push(
      stations.at(-1) +
        Math.hypot(
          points[i][0] - points[i - 1][0],
          points[i][1] - points[i - 1][1],
        ),
    );
  if (closed && points.length)
    stations.push(
      stations.at(-1) +
        Math.hypot(
          points[0][0] - points.at(-1)[0],
          points[0][1] - points.at(-1)[1],
        ),
    );
  return stations;
}
export function snapWaypointStation(stations, station) {
  return stations.reduce((best, s) => Math.abs(s-station) < Math.abs(best-station) ? s : best, stations[0]);
}
let active = null;
let split = null;
export function setSectionSplit(callback) { split = callback; }
export function selectSpeedRow(row) {
  const next = row.querySelector("select");
  if (active?.parentElement === row) return;
  active = next;
  renderWaypointPicker();
}
export function speedRows() {
  return [...$("speed-rows").children].map(row => ({...Object.fromEntries(
    [...row.querySelectorAll("[data-key]")].map(el => [el.dataset.key, Number(el.value)])),
    ...(row.dataset.kind ? {kind: row.dataset.kind} : {})}));
}
export function resetWaypointPicker() {
  active = null;
}
export function waypointSelect(key, value) {
  const select = document.createElement("select");
  select.dataset.key = key;
  select.setAttribute(
    "aria-label",
    key === "start" ? "開始waypoint" : "終了waypoint",
  );
  const pts = state.lane.custom;
  const stations = waypointStations(pts, state.lane.closed);
  stations.forEach((s, i) =>
    select.add(
      new Option(
        i === pts.length ? "WP 1（周回末尾）" : `WP ${i + 1}`,
        String(s),
      ),
    ),
  );
  // Preserve older distance-based sections exactly, without silently snapping.
  if (!stations.some((s) => s === value))
    select.add(new Option(`${value.toFixed(2)} m`, String(value)));
  select.value = String(value);
  select.addEventListener("focus", () => {
    active = select;
    renderWaypointPicker();
  });
  select.addEventListener("change", () => {
    delete select.parentElement.dataset.kind;
    renderWaypointPicker();
  });
  return select;
}
let viewport = null, hoverIndex = -1, gesture = null;
const dimensions = () => {
  const rect=$("speed-waypoints").getBoundingClientRect();
  return [Math.max(1,rect.width),Math.max(1,rect.height)];
};
export function resetWaypointView() { viewport=null; hoverIndex=-1; }
export function fitWaypointPicker() {
  const pts=state.lane?.custom;
  if(!pts?.length)return;
  viewport=fitViewport(pts,...dimensions());
  renderWaypointPicker();
}
export function focusWaypointSection(index = null) {
  const rows=[...$("speed-rows").children], sections=speedRows();
  const selected=index ?? rows.indexOf(active?.parentElement);
  const section=sections[selected];
  if(!section)return;
  active=rows[selected].querySelector("select");
  const pts=state.lane.custom, stations=waypointStations(pts,state.lane.closed);
  const subset=pts.filter((_,i)=>covers(section,stations[i]));
  subset.push(pointAtStation(pts,stations,section.start),pointAtStation(pts,stations,section.end));
  viewport=fitViewport(subset,...dimensions());
  renderWaypointPicker();
}
function pick(station) {
  if(split) { split(station); return; }
  if(!active)return;
  delete active.parentElement.dataset.kind;
  active.value=String(station);
  if(active.dataset.key==='start')active=active.parentElement.querySelector('[data-key="end"]');
  renderWaypointPicker();
}
function nearest(event) {
  const svg=$("speed-waypoints"), rect=svg.getBoundingClientRect(), pts=state.lane.custom;
  const position=[event.clientX-rect.left,event.clientY-rect.top];
  const stations=waypointStations(pts,state.lane.closed);
  const hit=nearestPathStation(pts,state.lane.closed,stations,unprojectViewport(viewport,position,...dimensions()));
  return {...hit,station:snapWaypointStation(stations,hit.station),stations};
}
function showHover(index) {
  hoverIndex=index;
  const svg=$("speed-waypoints"), group=svg.querySelector('[data-hover]');
  if(!group)return;
  group.replaceChildren();
  if(index<0)return;
  const pts=state.lane.custom, stations=waypointStations(pts,state.lane.closed);
  const point=index===pts.length?pts[0]:pts[index], [x,y]=projectViewport(viewport,point,...dimensions());
  const create=(tag,attrs)=>{const el=document.createElementNS(svg.namespaceURI,tag);for(const [k,v] of Object.entries(attrs))el.setAttribute(k,v);group.append(el);return el;};
  create('circle',{cx:x,cy:y,r:6,fill:'#ffe08a',stroke:'#263b49','stroke-width':2});
  const [w,h]=dimensions();
  const text=create('text',{x:Math.max(4,Math.min(w-100,x+10)),y:Math.max(16,Math.min(h-4,y-12)),fill:'#243e50','font-size':12,'font-weight':'bold',stroke:'white','stroke-width':4,'paint-order':'stroke'});
  text.textContent=`WP ${index===pts.length?1:index+1} · ${stations[index].toFixed(2)} m`;
  $("speed-pick-status").textContent=`WP ${index===pts.length?1:index+1} · ${stations[index].toFixed(2)} m：クリックで${split?'分割':'選択'}`;
}
export function renderWaypointPicker() {
  const svg=$("speed-waypoints"), pts=state.lane?.custom;
  svg.replaceChildren();
  if(!pts?.length || !svg.clientWidth)return;
  const [width,height]=dimensions();
  svg.setAttribute('viewBox',`0 0 ${width} ${height}`);
  viewport ??= fitViewport(pts,width,height);
  const project=p=>projectViewport(viewport,p,width,height);
  const rows=[...$("speed-rows").children], sections=speedRows();
  if(!active?.isConnected)active=rows[0]?.querySelector('select');
  const selected=rows.indexOf(active?.parentElement);
  rows.forEach((row,i)=>{
    row.style.setProperty('--section-color',sectionColor(i));
    row.classList.toggle('selected',i===selected);
    row.setAttribute('aria-label',`区間 ${i+1}${row.dataset.kind?` · 推定${sectionKindLabel(row.dataset.kind)}`:''}`);
  });
  $("speed-pick-status").textContent=split
    ? `${rows.length} 区間 · 線付近をクリックして最寄りWPで分割`
    : active ? `${selected+1}番の区間：${active.dataset.key==='start'?'開始':'終了'}WPを線付近のクリックで選択`
    : '「区間を追加」で開始します';
  const stations=waypointStations(pts,state.lane.closed), total=stations.at(-1);
  $("speed-overlap").textContent=hasOverlap(sections,total)
    ? '重複あり：重なった部分は低い速度を採用します。'
    : '重複なし · 線と同じ色・番号の行で編集します';
  function element(tag,attrs) {
    const el=document.createElementNS('http://www.w3.org/2000/svg',tag);
    for(const [k,v] of Object.entries(attrs))el.setAttribute(k,v);
    svg.append(el);return el;
  }
  const at=s=>project(pointAtStation(pts,stations,s));
  const cuts=[...new Set([...stations,...sections.flatMap(r=>[r.start,r.end])])]
    .filter(s=>Number.isFinite(s)&&s>=0&&s<=total).sort((a,b)=>a-b);
  const paths=[];
  for(let i=1;i<cuts.length;i++) {
    const a=cuts[i-1], b=cuts[i];if(b-a<1e-9)continue;
    const hits=sections.map((r,index)=>({r,index})).filter(({r})=>covers(r,(a+b)/2));
    const chosen=hits.reduce((best,item)=>!best||item.r.speed<best.r.speed?item:best,null)?.index ?? -1;
    const p=at(a), q=at(b);
    if(paths.at(-1)?.index===chosen) {paths.at(-1).d+=` L ${q.join(' ')}`;paths.at(-1).end=b;}
    else paths.push({index:chosen,start:a,end:b,d:`M ${p.join(' ')} L ${q.join(' ')}`});
  }
  for(const path of paths) {
    const chosen=path.index, color=chosen<0?'#bac6ce':sectionColor(chosen);
    const line=element('path',{d:path.d,fill:'none',stroke:color,'stroke-width':chosen===selected?11:8,
      'stroke-linejoin':'round','stroke-linecap':'round',tabindex:0,role:'button',
      'data-section':chosen,'aria-label':chosen<0?'未設定区間':`区間 ${chosen+1} · ${sections[chosen].speed} m/s`});
    line.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();pick(snapWaypointStation(stations,(path.start+path.end)/2));}});
  }
  if($("speed-show-waypoints").checked || $("speed-show-numbers").checked) {
    let previous=null;
    pts.forEach((p,i)=>{
      const [x,y]=project(p);
      if(x<0||y<0||x>width||y>height)return;
      if($("speed-show-waypoints").checked)element('circle',{cx:x,cy:y,r:2,fill:'#fff','pointer-events':'none','data-waypoint':i});
      if($("speed-show-numbers").checked && (!previous||Math.hypot(x-previous[0],y-previous[1])>35)) {
        const label=element('text',{x:x+5,y:y-8,'font-size':10,fill:'#354959',stroke:'white','stroke-width':3,'paint-order':'stroke','pointer-events':'none'});
        label.textContent=String(i+1);previous=[x,y];
      }
    });
  }
  const occupied=[];
  // Draw the selected section first so its label always has priority.
  const indices=sections.map((_,i)=>i).sort((a,b)=>(b===selected)-(a===selected));
  for(const i of indices) {
    const r=sections[i], mid=r.start<r.end?(r.start+r.end)/2:((r.start+r.end+total)/2)%total;
    let [x,y]=at(mid);
    if(x<0||y<0||x>width||y>height)continue;
    x=Math.max(8,Math.min(width-(i===selected?135:35),x+8));y=Math.max(18,Math.min(height-8,y-12));
    if(i!==selected && occupied.some(p=>Math.abs(p[0]-x)<65 && Math.abs(p[1]-y)<22))continue;
    occupied.push([x,y]);
    const label=element('text',{x,y,'font-size':i===selected?13:12,'font-weight':'bold',fill:sectionColor(i),'paint-order':'stroke',stroke:'white','stroke-width':4,'pointer-events':'none','data-section-label':i});
    label.textContent=i===selected?`${i+1} · ${r.speed} m/s${r.kind?` · ${sectionKindLabel(r.kind)}`:''}`:String(i+1);
  }
  sections.forEach((r,i)=>{
    const [x,y]=at(r.start);
    element('path',{d:`M ${x} ${y-5} l 5 5 -5 5 -5 -5 Z`,fill:sectionColor(i),stroke:'white','stroke-width':1.5,'pointer-events':'none','data-boundary':i});
  });
  element('g',{'data-hover':'','pointer-events':'none'});
  showHover(hoverIndex);
}
// Root listeners survive redraws. A drag pans; only a stationary release edits a boundary.
const picker=typeof document==='undefined'?null:$("speed-waypoints");
if(picker) {
  const zoom=factor=>{if(!viewport)return;const [w,h]=dimensions();viewport=zoomViewport(viewport,[w/2,h/2],factor,w,h);showHover(-1);renderWaypointPicker();};
  $("speed-zoom-in").addEventListener('click',()=>zoom(1.4));
  $("speed-zoom-out").addEventListener('click',()=>zoom(1/1.4));
  $("speed-view-fit").addEventListener('click',fitWaypointPicker);
  $("speed-view-selected").addEventListener('click',()=>focusWaypointSection());
  $("speed-view-expand").addEventListener('click',()=>{
    const expanded=$("speed-dialog").classList.toggle('map-expanded');
    $("speed-view-expand").textContent=expanded?'設定も表示':'図を広く';
    $("speed-view-expand").setAttribute('aria-pressed',String(expanded));
  });
  for(const id of ['speed-show-waypoints','speed-show-numbers'])$(id).addEventListener('change',renderWaypointPicker);
  picker.addEventListener('wheel',e=>{
    if(!viewport)return;e.preventDefault();
    const rect=picker.getBoundingClientRect();
    viewport=zoomViewport(viewport,[e.clientX-rect.left,e.clientY-rect.top],Math.exp(-e.deltaY*.0015),...dimensions());
    hoverIndex=-1;renderWaypointPicker();
  },{passive:false});
  picker.addEventListener('contextmenu',e=>e.preventDefault());
  picker.addEventListener('pointerdown',e=>{
    if(!viewport||![0,1,2].includes(e.button))return;
    gesture={id:e.pointerId,x:e.clientX,y:e.clientY,view:viewport,moved:false,select:e.button===0&&!e.altKey};
    picker.setPointerCapture(e.pointerId);
  });
  picker.addEventListener('pointermove',e=>{
    if(!viewport)return;
    if(gesture) {
      const dx=e.clientX-gesture.x,dy=e.clientY-gesture.y;
      gesture.moved ||= Math.hypot(dx,dy)>4;
      if(gesture.moved){viewport=panViewport(gesture.view,dx,dy);hoverIndex=-1;renderWaypointPicker();}
      return;
    }
    const hit=nearest(e);
    showHover(hit.distance*viewport.scale<18?hit.stations.indexOf(hit.station):-1);
  });
  picker.addEventListener('pointerup',e=>{
    if(!gesture||gesture.id!==e.pointerId)return;
    const select=gesture.select&&!gesture.moved;gesture=null;
    if(picker.hasPointerCapture(e.pointerId))picker.releasePointerCapture(e.pointerId);
    if(select){const hit=nearest(e);if(hit.distance*viewport.scale<20)pick(hit.station);}
  });
  picker.addEventListener('pointercancel',()=>{gesture=null;});
  picker.addEventListener('lostpointercapture',()=>{gesture=null;});
  picker.addEventListener('pointerleave',()=>{if(!gesture)showHover(-1);});
  picker.addEventListener('keydown',e=>{
    if(e.target!==picker)return;
    if(['+','=','-','0','ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key))e.preventDefault();else return;
    if(e.key==='0'){fitWaypointPicker();return;}
    if(e.key==='+'||e.key==='='){zoom(1.4);return;}if(e.key==='-'){zoom(1/1.4);return;}
    if(viewport){viewport=panViewport(viewport,...({ArrowLeft:[40,0],ArrowRight:[-40,0],ArrowUp:[0,40],ArrowDown:[0,-40]}[e.key]));renderWaypointPicker();}
  });
  if(typeof ResizeObserver!=='undefined')new ResizeObserver(()=>{if($("speed-dialog").open)renderWaypointPicker();}).observe(picker);
}
