// Pure editing operations. Persisted geometry and speed-section schema stay unchanged.
export function downsampleCustom(points, closed, spacing) {
  if (!Number.isFinite(spacing) || spacing < 0) throw Error("点間隔は0以上の有限値で指定してください");
  const original = points.map(([x,y]) => [x,y]);
  if (spacing === 0) return original;
  const pts = original.filter((p,i) => i === 0 || p.some((v,n) => v !== original[i-1][n]));
  if (closed && pts.length > 1 && pts[0].every((v,n) => v === pts.at(-1)[n])) pts.pop();
  const minimum = closed ? 3 : 2;
  if (pts.length < minimum) throw Error("間引くための有効な編集点が不足しています");
  const stations = [0];
  for (let i=1;i<pts.length;i++) stations.push(stations.at(-1)+Math.hypot(pts[i][0]-pts[i-1][0],pts[i][1]-pts[i-1][1]));
  const total = stations.at(-1) + (closed ? Math.hypot(pts[0][0]-pts.at(-1)[0],pts[0][1]-pts.at(-1)[1]) : 0);
  const count = Math.min(pts.length, Math.max(minimum, Math.ceil(total / spacing)+(closed?0:1)));
  const result = [];
  let previous = -1;
  for (let k=0;k<count;k++) {
    const target = total*k/(closed?count:count-1);
    let i = k === 0 ? 0 : previous+1;
    const last = pts.length-(count-k);
    while (i<last && Math.abs(stations[i+1]-target)<Math.abs(stations[i]-target)) i++;
    if (!closed && k === count-1) i=pts.length-1;
    result.push([...pts[i]]); previous=i;
  }
  return result;
}

export function clearCustom(lane) {
  lane.custom = [];
  lane.custom_speeds = [];
  delete lane.lines.customline;
}

export function copyCustom(lane, source, spacing = 0) {
  if (source === "manual") return [];
  if (!["centerline", "raceline"].includes(source)) throw Error("作成方法を選んでください");
  const points = lane.lines[source]?.points;
  if (!points || points.length < (lane.closed ? 3 : 2))
    throw Error("先にコピー元のラインを生成してください");
  const result = points.map(([x, y]) => [x, y]);
  if (lane.closed && result.length > 3 &&
      result[0].every((v, i) => v === result.at(-1)[i])) result.pop();
  return downsampleCustom(result, lane.closed, spacing);
}

export const sectionColor = (i) => `hsl(${(i * 137.508 + 205) % 360} 62% 40%)`;
export const covers = (row, s) => row.start < row.end
  ? s >= row.start && s < row.end : s >= row.start || s < row.end;

export function partitionSections(rows, total, defaultSpeed) {
  if (!(total > 0) || !(defaultSpeed > 0)) throw Error("有効なラインと速度が必要です");
  const cuts = [...new Set([0, total, ...rows.flatMap(r => [r.start, r.end])])]
    .filter(s => Number.isFinite(s) && s >= 0 && s <= total).sort((a, b) => a - b);
  const result = [];
  for (let i = 1; i < cuts.length; i++) {
    const start = cuts[i - 1], end = cuts[i];
    const matches = rows.filter(r => covers(r, (start + end) / 2));
    result.push({start, end, speed: matches.length ? Math.min(...matches.map(r => r.speed)) : defaultSpeed});
  }
  if (result.length > 40) throw Error("分割後の区間が40個を超えます。詳細指定で区間を整理してください");
  return result;
}

export function splitSection(rows, station) {
  if (rows.length >= 40) throw Error("区間は最大40個です");
  const index = rows.findIndex(r => station > r.start + 1e-6 && station < r.end - 1e-6);
  if (index < 0) return rows.map(r => ({...r}));
  return rows.flatMap((r, i) => i === index
    ? [{...r, end: station}, {...r, start: station}] : [{...r}]);
}

export function mergeSection(rows, index) {
  if (rows.length < 2) return rows.map(r => ({...r}));
  const left = index === 0 ? 0 : index - 1;
  // The lower limit survives a merge; no implicit speed increase.
  return rows.flatMap((r, i) => i === left
    ? [{start: r.start, end: rows[i + 1].end, speed: Math.min(r.speed, rows[i + 1].speed)}]
    : i === left + 1 ? [] : [{...r}]);
}

export function hasOverlap(rows, total) {
  const cuts = [...new Set([0, total, ...rows.flatMap(r => [r.start, r.end])])].sort((a,b) => a-b);
  return cuts.slice(1).some((end,i) => rows.filter(r => covers(r,(cuts[i]+end)/2)).length > 1);
}

export function isPartition(rows, total) {
  return rows.length > 0 && rows[0].start === 0 && rows.at(-1).end === total &&
    rows.every((r, i) => r.start < r.end && (i === 0 || r.start === rows[i - 1].end));
}

// Remove only the selected speed override; do not extend adjacent overrides.
export function deleteSection(rows, index) {
  return rows.filter((_, i) => i !== index).map(r => ({...r}));
}

// Keep section boundaries attached to surviving waypoints / segment fractions.
export function reanchorSections(before, after, closed, rows) {
  const stations=pts=>{const s=[0];for(let i=1;i<pts.length;i++)s.push(s.at(-1)+Math.hypot(pts[i][0]-pts[i-1][0],pts[i][1]-pts[i-1][1]));if(closed&&pts.length)s.push(s.at(-1)+Math.hypot(pts[0][0]-pts.at(-1)[0],pts[0][1]-pts.at(-1)[1]));return s;};
  if(after.length<(closed?3:2)||before.length<(closed?3:2))return [];
  const a=stations(before),b=stations(after), anchors=[[0,0]];
  if(!(a.at(-1)>0&&b.at(-1)>0))return [];
  if(before.length===after.length) {
    for(let i=1;i<a.length-1;i++)anchors.push([a[i],b[i]]);
  } else {
    let last=0;
    for(let i=1;i<before.length;i++) {
      const j=after.findIndex((p,j)=>j>last&&p[0]===before[i][0]&&p[1]===before[i][1]);
      if(j>0 && a[i]<a.at(-1) && b[j]<b.at(-1)){anchors.push([a[i],b[j]]);last=j;}
    }
  }
  anchors.push([a.at(-1),b.at(-1)]);
  const map=s=>{
    s=Math.max(0,Math.min(a.at(-1),s));
    let i=0;while(i<anchors.length-2 && anchors[i+1][0]<s)i++;
    const [x,y]=anchors[i],[xx,yy]=anchors[i+1];
    return y+(yy-y)*(s-x)/Math.max(1e-12,xx-x);
  };
  return rows.map(r=>({...r,start:map(r.start),end:map(r.end)})).filter(r=>Math.abs(r.end-r.start)>1e-6);
}
