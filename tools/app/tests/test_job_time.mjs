import assert from "node:assert/strict";
import { duration } from "../frontend/job_time.js";
assert.equal(duration(0), "0秒");
assert.equal(duration(65.9), "1分 5秒");
assert.equal(duration(3661), "1時間 1分 1秒");
for (const value of [null, undefined, NaN, -1]) assert.equal(duration(value), "時間不明");
console.log("job duration tests passed");
