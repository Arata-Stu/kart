import assert from 'node:assert/strict';
import { fitViewport, projectViewport, unprojectViewport, zoomViewport, panViewport, pointAtStation, nearestPathStation } from '../frontend/map_viewport.js';
import { waypointStations } from '../frontend/waypoint_sections.js';
const pts=[[0,0],[10,0],[10,10]], stations=waypointStations(pts,false);
const view=fitViewport(pts,800,500);
for(const p of pts) {
  const xy=projectViewport(view,p,800,500);
  assert.ok(xy[0]>=45&&xy[0]<=755&&xy[1]>=45&&xy[1]<=455);
  const restored=unprojectViewport(view,xy,800,500);
  assert.ok(Math.hypot(restored[0]-p[0],restored[1]-p[1])<1e-10);
}
const cursor=[122,188], anchor=unprojectViewport(view,cursor,800,500);
const zoomed=zoomViewport(view,cursor,2,800,500);
assert.equal(zoomed.scale,view.scale*2);
assert.ok(Math.hypot(...projectViewport(zoomed,anchor,800,500).map((v,i)=>v-cursor[i]))<1e-10,'zoom stays anchored under cursor');
const pan=panViewport(zoomed,31,-24);
assert.deepEqual(projectViewport(pan,anchor,800,500).map(v=>Math.round(v)),[153,164]);
assert.deepEqual(pointAtStation(pts,stations,14),[10,4]);
assert.deepEqual(nearestPathStation(pts,false,stations,[11,4]),{distance:1,station:14});
const closed=waypointStations(pts,true), mid=20+Math.sqrt(200)/2;
const hit=nearestPathStation(pts,true,closed,[5,5]);
assert.ok(Math.abs(hit.station-mid)<1e-10);assert.equal(hit.distance,0);
assert.deepEqual(pointAtStation(pts,closed,closed.at(-1)),[0,0]);
assert.equal(zoomViewport(view,cursor,1e20,800,500).scale,3000);
assert.equal(zoomViewport(view,cursor,1e-20,800,500).scale,.01);
assert.ok(Number.isFinite(fitViewport([[1,1],[1,1]],200,100).scale));
assert.deepEqual(view,fitViewport(pts,800,500),'view operations do not mutate the input');
console.log('Map viewport: fitting, cursor zoom anchor, panning, open/closed station picking and bounds passed');
