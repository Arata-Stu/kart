// Geometry shared by the parking canvas and its editor. Coordinates are map metres.
export function nearestEdge(points, p, tolerance=Infinity) {
  let best=null, distance=tolerance;
  for(let i=0;i<points.length;i++) {
    const a=points[i],b=points[(i+1)%points.length],dx=b[0]-a[0],dy=b[1]-a[1];
    const t=Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy||1)));
    const d=Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy);
    if(d<=distance){best=i;distance=d;}
  }
  return best;
}
export function moveVertex(points, entrance, index, p) {
  const old=points.map(v=>[...v]), next=old.map((v,i)=>i===index?[...p]:v);
  const gate=entrance.map(q=>{
    const edge=nearestEdge(old,q,1e-6);
    if(edge===null)return [...q];
    const a=old[edge],b=old[(edge+1)%old.length],dx=b[0]-a[0],dy=b[1]-a[1];
    const t=((q[0]-a[0])*dx+(q[1]-a[1])*dy)/(dx*dx+dy*dy||1);
    const c=next[edge],d=next[(edge+1)%next.length];
    return [c[0]+t*(d[0]-c[0]),c[1]+t*(d[1]-c[1])];
  });
  return {points:next,entrance:gate};
}
export function vehicleShape(goal, settings={}) {
  const length=settings.vehicle_length??.35,width=settings.vehicle_width??.19,rear=settings.rear_axle_to_rear??.1065;
  const [x,y,yaw]=goal;
  const transform=([a,b])=>[x+a*Math.cos(yaw)-b*Math.sin(yaw),y+a*Math.sin(yaw)+b*Math.cos(yaw)];
  return {corners:[[-rear,-width/2],[length-rear,-width/2],[length-rear,width/2],[-rear,width/2]].map(transform),tip:transform([Math.max(.5,length),0])};
}
export function handles(area, settings={}) {
  if(!area)return [];
  const result=['polygon','maneuver'].flatMap(field=>(area[field]||[]).map((point,index)=>({field,index,point})));
  if(area.goal_pose){result.push({field:'goal',point:area.goal_pose.slice(0,2)});result.push({field:'heading',point:vehicleShape(area.goal_pose,settings).tip});}
  return result;
}
export function hitHandle(area,p,tolerance,settings={}) {
  return handles(area,settings).reverse().find(h=>Math.hypot(h.point[0]-p[0],h.point[1]-p[1])<=tolerance);
}
