import assert from "node:assert/strict";
import { placementPreview } from "../frontend/placement.js";
import { appendPair } from "../frontend/paired.js";
for (const count of [0, 1, 2, 3]) {
  const lane = { left: [], right: [], custom: [[0, 0]], closed: true };
  for (let i = 0; i < count; i++) appendPair(lane, [i, 0], 1.4);
  const original = structuredClone(lane);
  const p = [4, 3];
  const preview = placementPreview(lane, "pair", p, 2);
  const actual = structuredClone(lane);
  appendPair(actual, p, 2);
  assert.deepEqual(preview.left, actual.left);
  assert.deepEqual(preview.right, actual.right);
  assert.deepEqual(lane, original, "hover must not edit the lane");
  const single = placementPreview(lane, "custom", p, 2);
  assert.deepEqual(single.points, [...lane.custom, p]);
  assert.equal(single.closed, true);
}
const lane = { left: [], right: [], closed: false };
assert.equal(placementPreview(lane, "pair", [0, 0], NaN), null);
assert.equal(placementPreview({left:[[0,0]],right:[]}, "pair", [0, 0], 1), null);
assert.equal(placementPreview(lane, "obstacle", [0, 0], 1), null);
const obstacle = { polygon: [[0, 0], [1, 0]] };
const preview = placementPreview(lane, "obstacle", [1, 1], 1, obstacle);
assert.equal(preview.closed, true);
assert.equal(obstacle.polygon.length, 2);
assert.deepEqual(preview.points, [[0, 0], [1, 0], [1, 1]]);
console.log("placement preview: actual insertion parity, non-mutation and invalid targets OK");
