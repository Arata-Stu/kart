import assert from 'node:assert/strict';
import { hdmapPaths } from '../frontend/mesh_layers.js';
import { PLYLoader } from '../frontend/vendor/three/PLYLoader.js';
const doc = { lanes: [{ closed: true, left: [[1, 2], [3, 4]],
  lines: { centerline: { points: [[5, 6, 99], [7, 8, 100]] } } }] };
const before = JSON.stringify(doc);
const paths = hdmapPaths(doc, 0.25);
assert.deepEqual(paths[0].points, [[1, 2, 0.25], [3, 4, 0.25], [1, 2, 0.25]]);
assert.equal(paths[1].points[0][2], 0.25);
assert.equal(JSON.stringify(doc), before);
const header = format => `ply\nformat ${format} 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n`;
const xyz = [1,2,3,4,5,6,7,8,9];
const ascii = new TextEncoder().encode(header('ascii') + '1 2 3\n4 5 6\n7 8 9\n3 0 1 2\n');
for (const little of [true, false]) {
  const h = new TextEncoder().encode(header(little ? 'binary_little_endian' : 'binary_big_endian'));
  const bytes = new Uint8Array(h.length + 49); bytes.set(h);
  const v = new DataView(bytes.buffer);
  xyz.forEach((n, i) => v.setFloat32(h.length + 4*i, n, little));
  v.setUint8(h.length + 36, 3);
  [0,1,2].forEach((n,i) => v.setInt32(h.length+37+4*i,n,little));
  const g = new PLYLoader().parse(bytes.buffer);
  assert.deepEqual([...g.attributes.position.array], xyz);
  assert.deepEqual([...g.index.array], [0,1,2]); g.dispose();
}
const g = new PLYLoader().parse(ascii.buffer);
assert.deepEqual([...g.attributes.position.array], xyz); g.dispose();
console.log('HDMap planar coordinates and ASCII/binary PLY coordinates preserved');
