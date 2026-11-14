// Geometry-only suggestions in Custom's arc-length coordinates. No lane mutations.
export const sectionKindLabel = kind => ({straight: "ストレート", left: "左カーブ", right: "右カーブ"}[kind] || "");

export function suggestCustomSections(points, closed, {
  threshold = 0.15, window = 2, minLength = 1,
} = {}, existing = [], defaultSpeed = 1.5) {
  if (![threshold, window, minLength, defaultSpeed].every(v => Number.isFinite(v) && v > 0))
    throw Error("曲率しきい値・判定距離・最小区間長・速度は正の有限値で指定してください");
  if (points.length < (closed ? 3 : 2) || points.some(p => p.length < 2 || !p.slice(0, 2).every(Number.isFinite)))
    throw Error("区間候補を作るための有効なCustom編集点が不足しています");
  if (existing.some(r => ![r.start, r.end, r.speed].every(Number.isFinite) || r.speed <= 0))
    throw Error("先に区間の距離と速度を確認してください");
  const pts = points.filter((p, i) => !i || Math.hypot(p[0]-points[i-1][0], p[1]-points[i-1][1]) > 1e-9);
  if (closed && pts.length > 1 && Math.hypot(pts[0][0]-pts.at(-1)[0], pts[0][1]-pts.at(-1)[1]) <= 1e-9) pts.pop();
  if (pts.length < (closed ? 3 : 2)) throw Error("異なるCustom編集点が不足しています");
  const stations = [0];
  for (let i=1; i<pts.length; i++) stations.push(stations.at(-1)+Math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]));
  if (closed) stations.push(stations.at(-1)+Math.hypot(pts[0][0]-pts.at(-1)[0], pts[0][1]-pts.at(-1)[1]));
  const total = stations.at(-1), n = stations.length-1;
  const half = Math.min(window / 2, total / (closed ? 4 : 2));
  function at(s) {
    // Open ends extend along their endpoint tangent, rather than inventing a corner.
    if (closed) s = ((s % total) + total) % total;
    let lo=0, hi=n-1;
    while (lo<hi) { const mid=Math.floor((lo+hi)/2); if(stations[mid+1]<s)lo=mid+1;else hi=mid; }
    const i=lo, a=pts[i], b=pts[(i+1)%pts.length];
    const t=(s-stations[i])/(stations[i+1]-stations[i]);
    return [a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t];
  }
  const labels = Array.from({length:n}, (_,i) => {
    const s=(stations[i]+stations[i+1])/2, a=at(s-half), b=at(s), c=at(s+half);
    const u=[b[0]-a[0],b[1]-a[1]], v=[c[0]-b[0],c[1]-b[1]];
    const span=(Math.hypot(...u)+Math.hypot(...v))/2;
    const k=Math.atan2(u[0]*v[1]-u[1]*v[0], u[0]*v[0]+u[1]*v[1])/Math.max(1e-9,span);
    return Math.abs(k)<threshold ? "straight" : k>0 ? "left" : "right";
  });
  function runs() {
    let start=0;
    if (closed) {
      const cut=labels.findIndex((kind,i) => kind!==labels[(i+n-1)%n]);
      if (cut>=0) start=cut;
    }
    const result=[];
    for(let j=0;j<n;j++) {
      const i=(start+j)%n, kind=labels[i];
      if (result.at(-1)?.kind!==kind) result.push({kind, indices:[], length:0});
      result.at(-1).indices.push(i); result.at(-1).length+=stations[i+1]-stations[i];
    }
    return result;
  }
  // Suppress short islands, including those straddling a closed loop's origin.
  for (;;) {
    const groups=runs();
    if(groups.length<2) break;
    const short=groups.reduce((best,r,i) => r.length<minLength && (best<0 || r.length<groups[best].length) ? i : best,-1);
    if(short<0) break;
    const left=groups[short-1] || (closed ? groups.at(-1) : null);
    const right=groups[short+1] || (closed ? groups[0] : null);
    const neighbor=!left ? right : !right ? left : left.length>=right.length ? left : right;
    for(const i of groups[short].indices) labels[i]=neighbor.kind;
  }
  // Keep the origin as an explicit boundary for compatibility with split editing.
  const result=[];
  for(let i=0;i<n;i++) {
    if(result.at(-1)?.kind===labels[i]) result.at(-1).end=stations[i+1];
    else result.push({start:stations[i],end:stations[i+1],kind:labels[i]});
  }
  if(result.length>40) throw Error("候補が40区間を超えます。判定距離か最小区間長を大きくしてください");
  return result.map(r => {
    // Coarsening retains the lowest effective target in each suggested interval.
    // Include defaultSpeed only where a part has no override; handle wrapped ranges.
    const cuts=[...new Set([r.start,r.end,...existing.flatMap(v=>[v.start,v.end])])]
      .filter(s=>s>=r.start && s<=r.end).sort((a,b)=>a-b);
    const speeds=cuts.slice(1).map((end,i) => {
      const s=(cuts[i]+end)/2;
      const hits=existing.filter(v=>v.start<v.end ? s>=v.start && s<v.end : s>=v.start || s<v.end);
      return hits.length ? Math.min(...hits.map(v=>v.speed)) : defaultSpeed;
    });
    return {...r,speed:Math.min(...speeds)};
  });
}
