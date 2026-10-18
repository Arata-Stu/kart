// HDMap is planar: never interpret a third line field as measured elevation.
export function hdmapPaths(doc, z = 0) {
  if (!Number.isFinite(z)) throw Error("表示高さが不正です");
  const result = [];
  const add = (points, color, closed) => {
    if (!points?.length) return;
    const xyz = points.map(p => [p[0], p[1], z]);
    if (!xyz.every(p => p.every(Number.isFinite))) throw Error("HDMapに不正な座標があります");
    if (closed && xyz.length > 1) xyz.push([...xyz[0]]);
    result.push({ points: xyz, color });
  };
  for (const lane of doc.lanes || []) {
    add(lane.left, 0x42aaff, lane.closed);
    add(lane.right, 0xff9966, lane.closed);
    for (const [kind, color] of [["centerline", 0x32d58b], ["raceline", 0xffd347], ["customline", 0xdc8cff]])
      add(lane.lines?.[kind]?.points, color, lane.closed);
  }
  for (const obstacle of doc.obstacles || []) add(obstacle.polygon, 0xff5555, true);
  return result;
}
