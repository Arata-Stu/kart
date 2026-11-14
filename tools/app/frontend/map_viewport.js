// View-only geometry. Coordinates and section distances are never edited here.
export function fitViewport(points, width, height, padding = 45) {
  const xs=points.map(p=>p[0]), ys=points.map(p=>p[1]);
  const minX=Math.min(...xs), maxX=Math.max(...xs), minY=Math.min(...ys), maxY=Math.max(...ys);
  return {center:[(minX+maxX)/2,(minY+maxY)/2], scale:Math.min(
    Math.max(1,width-2*padding)/Math.max(.1,maxX-minX),
    Math.max(1,height-2*padding)/Math.max(.1,maxY-minY))};
}
export const projectViewport = (view,p,width,height) => [width/2+(p[0]-view.center[0])*view.scale,height/2-(p[1]-view.center[1])*view.scale];
export const unprojectViewport = (view,p,width,height) => [view.center[0]+(p[0]-width/2)/view.scale,view.center[1]-(p[1]-height/2)/view.scale];
export function zoomViewport(view, point, factor, width, height) {
  const anchor=unprojectViewport(view,point,width,height);
  const result={center:[...view.center],scale:Math.max(.01,Math.min(3000,view.scale*factor))};
  const after=unprojectViewport(result,point,width,height);
  result.center=result.center.map((v,i)=>v+anchor[i]-after[i]);
  return result;
}
export function panViewport(view,dx,dy) {
  return {...view,center:[view.center[0]-dx/view.scale,view.center[1]+dy/view.scale]};
}
export function pointAtStation(points, stations, station) {
  let i=0;
  while(i<stations.length-2 && stations[i+1]<station)i++;
  const a=points[i], b=points[(i+1)%points.length];
  const t=(station-stations[i])/Math.max(1e-12,stations[i+1]-stations[i]);
  return a.map((v,k)=>v+(b[k]-v)*t);
}
export function nearestPathStation(points, closed, stations, point) {
  let best={distance:Infinity,station:0};
  for(let i=0;i<points.length-(closed?0:1);i++) {
    const a=points[i], b=points[(i+1)%points.length], dx=b[0]-a[0], dy=b[1]-a[1];
    const t=Math.max(0,Math.min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/Math.max(1e-12,dx*dx+dy*dy)));
    const distance=Math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy);
    if(distance<best.distance)best={distance,station:stations[i]+t*(stations[i+1]-stations[i])};
  }
  return best;
}
