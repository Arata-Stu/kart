// Target speeds, not measured vehicle speeds. All visible lines share one scale.
export function lineSpeeds(lane, kind, data) {
  if (kind === "centerline") {
    const speed = lane.centerline_speed_mps ?? 1.5;
    return Number.isFinite(speed) && speed >= 0 ? data.points.map(() => speed) : null;
  }
  if (!Array.isArray(data.profile) || data.profile.length !== data.points.length) return null;
  const values = data.profile.map(row => row?.[5]);
  return values.every(v => Number.isFinite(v) && v >= 0) ? values : null;
}

export function lineSpeedScale(lane, kinds) {
  const speeds = new Map();
  let max = 0;
  for (const kind of kinds) {
    const data = lane.lines[kind];
    if (!data?.points.length) continue;
    const values = lineSpeeds(lane, kind, data);
    if (!values) continue;
    speeds.set(kind, values);
    for (const value of values) max = Math.max(max, value);
  }
  return {speeds, max};
}

export function speedColor(value, max) {
  const fraction = max > 0 ? Math.max(0, Math.min(1, value/max)) : 0;
  return `hsl(${225*(1-fraction)},75%,43%)`;
}

export function paintSpeedLine(ctx, points, closed, speeds, max, project, step=false) {
  ctx.save();
  ctx.setLineDash([]);
  ctx.lineWidth = 4;
  ctx.lineCap = "round";
  for (let i=0; i<points.length-(closed?0:1); i++) {
    const j=(i+1)%points.length;
    const a=project(points[i]), b=project(points[j]);
    if (a[0] === b[0] && a[1] === b[1]) continue;
    const gradient=ctx.createLinearGradient(a[0],a[1],b[0],b[1]);
    for (const t of [0,.25,.5,.75,1])
      gradient.addColorStop(t,speedColor(step?speeds[i]:speeds[i]+(speeds[j]-speeds[i])*t,max));
    ctx.strokeStyle=gradient;
    ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
  }
  ctx.restore();
}

// Return one nearest segment per visible line so overlapping lines stay readable.
export function speedAtCursor(lane,kinds,p,project,tolerance=9) {
  const scale=lineSpeedScale(lane,kinds), result=[];
  for(const [kind,speeds] of scale.speeds) {
    const data=lane.lines[kind], points=data.points;let best=null;
    for(let i=0;i<points.length-(lane.closed?0:1);i++) {
      const j=(i+1)%points.length,a=project(points[i]),b=project(points[j]);
      const dx=b[0]-a[0],dy=b[1]-a[1],length=dx*dx+dy*dy;if(!length)continue;
      const t=Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length));
      const distance=Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy);
      if(distance<=tolerance && (!best || distance<best.distance)) {
        const speed=data.speed_mode==='step'?speeds[i]:speeds[i]+(speeds[j]-speeds[i])*t;
        best={kind,speed,distance,color:speedColor(speed,scale.max)};
      }
    }
    if(best)result.push(best);
  }
  return result.sort((a,b)=>a.distance-b.distance);
}
