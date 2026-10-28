import assert from "node:assert/strict";
import { waypointStations, snapWaypointStation } from "../frontend/waypoint_sections.js";
const points = [[0,0],[3,0],[3,4]];
assert.deepEqual(waypointStations(points,false),[0,3,7]);
assert.deepEqual(waypointStations(points,true),[0,3,7,12]);
assert.deepEqual(points,[[0,0],[3,0],[3,4]]);
console.log("waypoint distances: open, closed and loop endpoint OK");

const stations=waypointStations(points,true);
assert.equal(snapWaypointStation(stations,2.8),3);
assert.equal(snapWaypointStation(stations,5.2),7);
assert.equal(snapWaypointStation(stations,11.9),12);
assert.equal(snapWaypointStation(stations,0),0);
