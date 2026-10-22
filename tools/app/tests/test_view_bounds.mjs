import assert from "node:assert/strict";
import { viewBounds } from "../frontend/view_bounds.js";

const core = Array.from({length: 1000}, (_, i) => [(i % 40) / 8, Math.floor(i / 40) / 5, 0]);
const cloud = [...core, [-200, 120, 0], [180, -140, 0], [NaN, 0, 0]];
const original = structuredClone(cloud);
assert.deepEqual(viewBounds(null, cloud), [0, 0, 4.875, 4.8]);
assert.deepEqual(viewBounds(null, cloud, true), [-200, -140, 180, 120]);
assert.deepEqual(cloud, original, "fitting must preserve original points");

const doc = {lanes: [
  {left: [[0, 0], [0, 5]], right: [[4, 0], [4, 5]]},
  {left: [[10, 1], [10, 3]], right: [[12, 1], [12, 3]]},
]};
assert.deepEqual(viewBounds(doc, cloud), [0, 0, 12, 5], "all saved lanes take precedence over cloud outliers");
assert.deepEqual(viewBounds(doc, [], true), [0, 0, 12, 5]);
assert.deepEqual(viewBounds({lanes:[{lines:{centerline:{points:[[1, 2], [3, 4]]}}}]}), [1, 2, 3, 4]);
assert.equal(viewBounds(null), null);
assert.deepEqual(viewBounds(null, [[0, 0], [1, 1]]), [0, 0, 1, 1]);
assert.deepEqual(viewBounds(null, [...Array.from({length:100}, (_, i) => [0, i / 20]), [100, 100]]), [0, 0, 0, 4.95]);
assert.deepEqual(viewBounds(null, Array.from({length:100}, () => [2, 3])), [2, 3, 2, 3]);
console.log("view bounds: course priority, multiple lanes, outliers, full extent and degenerate clouds OK");
