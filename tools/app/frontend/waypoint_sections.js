import { $, state } from "./api.js";
import { covers, sectionColor, hasOverlap } from "./custom_model.js";
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
  return [...$("speed-rows").children].map(row => Object.fromEntries(
    [...row.querySelectorAll("[data-key]")].map(el => [el.dataset.key, Number(el.value)])));
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
  select.addEventListener("change", renderWaypointPicker);
  return select;
}
export function renderWaypointPicker() {
  const svg = $("speed-waypoints"), pts = state.lane.custom;
  svg.replaceChildren();
  const rows = [...$("speed-rows").children], sections = speedRows();
  if (!active?.isConnected) active = rows[0]?.querySelector("select");
  const selected = rows.indexOf(active?.parentElement);
  rows.forEach((row, i) => {
    row.style.setProperty("--section-color", sectionColor(i));
    row.classList.toggle("selected", i === selected);
    row.setAttribute("aria-label", `区間 ${i + 1}`);
  });
  $("speed-pick-status").textContent = split
    ? `${rows.length} 区間 · 白いWP点をクリックして分割。線上のクリックも最寄りWPに合わせます`
    : active ? `${selected + 1}番の区間：${active.dataset.key === "start" ? "開始" : "終了"}WPをクリック`
    : "「区間を追加」で開始します";
  if (!pts.length) return;
  const stations = waypointStations(pts, state.lane.closed), total = stations.at(-1);
  $("speed-overlap").textContent = hasOverlap(sections, total)
    ? "重複あり：重なった部分は低い速度を採用します。分割モードへ切り替えると重複のない区間に整理します。"
    : "重複なし · すべての区間を色と番号で表示しています";
  const xs = pts.map(p => p[0]), ys = pts.map(p => p[1]);
  const xmin = Math.min(...xs), ymin = Math.min(...ys);
  const sx = Math.max(...xs) - xmin, sy = Math.max(...ys) - ymin;
  const scale = Math.min(460 / Math.max(sx, 0.01), 180 / Math.max(sy, 0.01));
  const project = p => [260 + (p[0]-xmin-sx/2)*scale, 120-(p[1]-ymin-sy/2)*scale];
  function element(tag, attrs) {
    const el = document.createElementNS("http://www.w3.org/2000/svg", tag);
    for (const [k,v] of Object.entries(attrs)) el.setAttribute(k,v);
    svg.append(el); return el;
  }
  function pick(station) {
    if (split) { split(snapWaypointStation(stations, station)); return; }
    if (!active) return;
    active.value = String(station);
    if (active.dataset.key === "start") active = active.parentElement.querySelector('[data-key="end"]');
    renderWaypointPicker();
  }
  function keyboard(el, callback) {
    el.addEventListener("keydown", e => {
      if (e.key === "Enter" || e.key === " ") { e.preventDefault(); callback(); }
    });
  }
  for (let i=0; i<pts.length-(state.lane.closed?0:1); i++) {
    const j=(i+1)%pts.length;
    const cuts=[...new Set([stations[i],stations[i+1],...sections.flatMap(r=>[r.start,r.end])])]
      .filter(s=>s>=stations[i] && s<=stations[i+1]).sort((a,b)=>a-b);
    const interp=s=>project(pts[i].map((v,n)=>v+(pts[j][n]-v)*(s-stations[i])/Math.max(1e-12,stations[i+1]-stations[i])));
    for(let k=1;k<cuts.length;k++) {
      const a=cuts[k-1], b=cuts[k]; if(a===b)continue;
      const hits=sections.map((r,index)=>({r,index})).filter(({r})=>covers(r,(a+b)/2));
      const chosen=hits.reduce((best,item)=>!best||item.r.speed<best.r.speed?item:best,null);
      const p=interp(a),q=interp(b);
      const line=element("line",{x1:p[0],y1:p[1],x2:q[0],y2:q[1],
        stroke:chosen?sectionColor(chosen.index):"#bac6ce","stroke-width":chosen?.index===selected?9:6,
        "stroke-linecap":"round", tabindex:0,role:"button",
        "aria-label":chosen?`区間 ${chosen.index+1}: ${chosen.r.speed} m/s。最寄りWPで分割`:"未設定区間。最寄りWPで分割"});
      const title=document.createElementNS("http://www.w3.org/2000/svg","title");
      title.textContent=line.getAttribute("aria-label");line.append(title);
      line.style.cursor="pointer";
      line.addEventListener("click",e=>{
        if(!split) {
          if(chosen){active=rows[chosen.index].querySelector("select");active.focus();renderWaypointPicker();}
          return;
        }
        const point=svg.createSVGPoint();point.x=e.clientX;point.y=e.clientY;
        const xy=point.matrixTransform(svg.getScreenCTM().inverse());
        const dx=q[0]-p[0],dy=q[1]-p[1];
        const t=Math.max(0,Math.min(1,((xy.x-p[0])*dx+(xy.y-p[1])*dy)/Math.max(1e-12,dx*dx+dy*dy)));
        pick(a+(b-a)*t);
      });
      keyboard(line,()=>pick((a+b)/2));
    }
  }
  // Exact interval boundaries are shown even between waypoints.
  function at(s) {
    let i=0;while(i<stations.length-2 && stations[i+1]<s)i++;
    const j=(i+1)%pts.length,t=(s-stations[i])/Math.max(1e-12,stations[i+1]-stations[i]);
    return project(pts[i].map((v,n)=>v+(pts[j][n]-v)*t));
  }
  sections.forEach((r,i)=>{
    const midpoint = r.start < r.end ? (r.start+r.end)/2 : ((r.start+r.end+total)/2)%total;
    const [x,y]=at(midpoint);
    const label=element("text",{x:x+6,y:y-10,"font-size":12,"font-weight":"bold",fill:sectionColor(i),"paint-order":"stroke",stroke:"white","stroke-width":3,"pointer-events":"none"});
    label.textContent=`${i+1} · ${r.speed} m/s`;
    const [bx,by]=at(r.start);
    element("circle",{cx:bx,cy:by,r:5,fill:sectionColor(i),stroke:"white","stroke-width":2,"pointer-events":"none"});
  });
  pts.forEach((p,i)=>{
    const [x,y]=project(p);
    const dot=element("circle",{cx:x,cy:y,r:5,fill:"white",stroke:"#9063b0","stroke-width":2,tabindex:0,role:"button","aria-label":`WP ${i+1}`});
    dot.style.cursor="pointer";
    const title=document.createElementNS("http://www.w3.org/2000/svg","title");
    title.textContent=`WP ${i+1} · ${stations[i].toFixed(2)} m`;dot.append(title);
    const highlight=()=>{dot.setAttribute("r",8);dot.setAttribute("fill","#ffe08a");$("speed-pick-status").textContent=`WP ${i+1} · ${stations[i].toFixed(2)} m${split ? "：クリックでこの点を境界に分割" : "：クリックで選択"}`;};
    const unhighlight=()=>{dot.setAttribute("r",5);dot.setAttribute("fill","white");};
    dot.addEventListener("mouseenter",highlight);dot.addEventListener("focus",highlight);
    dot.addEventListener("mouseleave",unhighlight);dot.addEventListener("blur",unhighlight);
    dot.addEventListener("click",()=>pick(stations[i]));keyboard(dot,()=>pick(stations[i]));
    if(pts.length<50 || i%Math.ceil(pts.length/35)===0){
      const text=element("text",{x:x+6,y:y-6,"font-size":10,fill:"#354959","pointer-events":"none"});text.textContent=String(i+1);
    }
  });
}
