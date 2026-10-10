import assert from "node:assert/strict";
import { waypointStations } from "../frontend/waypoint_sections.js";
const points = [[0,0],[3,0],[3,4]];
assert.deepEqual(waypointStations(points,false),[0,3,7]);
assert.deepEqual(waypointStations(points,true),[0,3,7,12]);
assert.deepEqual(points,[[0,0],[3,0],[3,4]]);
console.log("waypoint distances: open, closed and loop endpoint OK");
