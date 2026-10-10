import assert from "node:assert/strict";
import {
  centers,
  movePair,
  appendPair,
  resizePairs,
  alignPairs,
} from "../frontend/paired.js";
const lane = { left: [], right: [], closed: false };
appendPair(lane, [0, 0], 2);
appendPair(lane, [0, 5], 2);
assert.deepEqual(lane.left, [
  [-1, 0],
  [-1, 5],
]);
movePair(lane, 1, [2, 5]);
assert.deepEqual(centers(lane), [
  [0, 0],
  [2, 5],
]);
assert.deepEqual(lane.right[1], [3, 5]);
resizePairs(lane, 4, 1);
assert.ok(
  Math.abs(
    Math.hypot(...lane.left[1].map((v, k) => v - lane.right[1][k])) - 4,
  ) < 1e-12,
);
assert.deepEqual(lane.left[0], [-1, 0]);
const uneven = {
  left: [
    [0, 1],
    [2, 1],
    [4, 1],
  ],
  right: [
    [4, -1],
    [0, -1],
  ],
  closed: false,
};
alignPairs(uneven);
assert.deepEqual(centers(uneven), [
  [0, 0],
  [2, 0],
  [4, 0],
]);
const closed = {
  left: [
    [0, 0],
    [4, 0],
    [4, 4],
    [0, 4],
  ],
  right: [
    [3, 3],
    [3, 1],
    [1, 1],
    [1, 3],
  ],
  closed: true,
};
alignPairs(closed);
assert.deepEqual(centers(closed), [
  [0.5, 0.5],
  [3.5, 0.5],
  [3.5, 3.5],
  [0.5, 3.5],
]);
console.log("paired geometry: passed");
const corner = { left: [], right: [], closed: false };
appendPair(corner, [0, 0], 2);
appendPair(corner, [5, 0], 2);
appendPair(corner, [5, 5], 2);
assert.deepEqual(corner.left[1], [4, 1]);
assert.deepEqual(corner.right[1], [6, -1]);
assert.deepEqual(centers(corner), [
  [0, 0],
  [5, 0],
  [5, 5],
]);
assert.throws(() => appendPair(corner, [5, 5], 2), /1cm/);
const square = { left: [], right: [], closed: true };
for (const p of [
  [0, 0],
  [5, 0],
  [5, 5],
  [0, 5],
])
  appendPair(square, p, 2);
resizePairs(square, 2);
assert.deepEqual(square.left, [
  [1, 1],
  [4, 1],
  [4, 4],
  [1, 4],
]);
