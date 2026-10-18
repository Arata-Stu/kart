import assert from "node:assert/strict";
import { rasterTransform } from "../frontend/occupancy_geometry.js";
const project = ([x, y]) => [100 + 20 * x, 200 - 20 * y];
for (const yaw of [0, Math.PI / 2, -0.7]) {
  const origin = [3, -4, yaw], r = 0.05, h = 80;
  const [a, b, c, d, e, f] = rasterTransform(origin, r, h, project, 20);
  for (const [u, v] of [[0, h], [0, 0], [40, 0], [0.5, 79.5]]) {
    const actual = [a * u + c * v + e, b * u + d * v + f];
    const x = u * r, y = (h - v) * r;
    const expected = project([3 + Math.cos(yaw) * x - Math.sin(yaw) * y,
      -4 + Math.sin(yaw) * x + Math.cos(yaw) * y]);
    actual.forEach((n, i) => assert.ok(Math.abs(n - expected[i]) < 1e-9));
  }
}
console.log("occupancy pixel origin, row inversion, rotation and cell-center checks passed");
