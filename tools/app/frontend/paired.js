// Paired boundary stations; the existing left/right arrays remain authoritative.
export function centers(lane) {
  return lane.left.map((p, i) => p.map((v, k) => (v + lane.right[i][k]) / 2));
}
export function compatible(lane) {
  return lane.left.length === lane.right.length;
}
export function movePair(lane, i, p) {
  const old = centers(lane)[i];
  for (const side of ["left", "right"])
    lane[side][i] = lane[side][i].map((v, k) => v + p[k] - old[k]);
}
// Offset the two adjoining segments at their bisector. Limit sharp-corner miters.
function section(mid, i, width, closed = false) {
  const p = mid[i],
    n = mid.length;
  const previous = i > 0 ? mid[i - 1] : closed && n > 2 ? mid[n - 1] : null;
  const next = i + 1 < n ? mid[i + 1] : closed && n > 2 ? mid[0] : null;
  const unit = (a, b) => {
    const dx = b[0] - a[0],
      dy = b[1] - a[1],
      length = Math.hypot(dx, dy);
    return length > 1e-6 ? [dx / length, dy / length] : null;
  };
  const incoming = previous && unit(previous, p);
  const outgoing = next && unit(p, next);
  let tangent = outgoing || incoming || [1, 0],
    scale = 1;
  if (incoming && outgoing) {
    const x = incoming[0] + outgoing[0],
      y = incoming[1] + outgoing[1];
    const length = Math.hypot(x, y);
    if (length > 1e-6) {
      tangent = [x / length, y / length];
      scale = Math.min(
        2,
        1 / Math.max(0.01, tangent[0] * outgoing[0] + tangent[1] * outgoing[1]),
      );
    }
  }
  const dx = (-tangent[1] * width * scale) / 2,
    dy = (tangent[0] * width * scale) / 2;
  return [
    [p[0] + dx, p[1] + dy],
    [p[0] - dx, p[1] - dy],
  ];
}
export function appendPair(lane, p, width) {
  const mid = centers(lane),
    previous = mid.at(-1);
  if (previous && Math.hypot(p[0] - previous[0], p[1] - previous[1]) < 0.01)
    throw Error("直前の点から1cm以上離して配置してください");
  const oldWidth = mid.length
    ? Math.hypot(
        lane.left.at(-1)[0] - lane.right.at(-1)[0],
        lane.left.at(-1)[1] - lane.right.at(-1)[1],
      )
    : width;
  mid.push([...p]);
  // While drawing, the last station is an open endpoint even for a closed lane.
  if (mid.length > 1) {
    const i = mid.length - 2;
    [lane.left[i], lane.right[i]] = section(mid, i, oldWidth);
  }
  const [left, right] = section(mid, mid.length - 1, width);
  lane.left.push(left);
  lane.right.push(right);
}
export function resizePairs(lane, width, selected = -1) {
  const mid = centers(lane);
  for (let i = 0; i < mid.length; i++) {
    if (selected >= 0 && i !== selected) continue;
    [lane.left[i], lane.right[i]] = section(mid, i, width, lane.closed);
  }
}
export function alignPairs(lane) {
  const count = Math.max(lane.left.length, lane.right.length);
  if (!count) return;
  const minimum = lane.closed ? 3 : 2;
  if (Math.min(lane.left.length, lane.right.length) < minimum)
    throw Error(`左右それぞれ${minimum}点以上必要です`);
  function sample(points) {
    const segments = points
      .slice(0, lane.closed ? undefined : -1)
      .map((p, i) => [p, points[(i + 1) % points.length]]);
    const lengths = segments.map(([a, b]) =>
      Math.hypot(a[0] - b[0], a[1] - b[1]),
    );
    const total = lengths.reduce((a, b) => a + b, 0);
    if (total < 0.1) throw Error("境界が短すぎます");
    return Array.from({ length: count }, (_, i) => {
      let station = (total * i) / (lane.closed ? count : count - 1),
        k = 0;
      while (k < lengths.length - 1 && station > lengths[k])
        station -= lengths[k++];
      const [a, b] = segments[k],
        t = station / Math.max(lengths[k], 1e-12);
      return a.map((v, j) => v + t * (b[j] - v));
    });
  }
  const left = sample(lane.left),
    right = sample(lane.right);
  let best = null,
    cost = Infinity;
  for (const sequence of [right, [...right].reverse()]) {
    for (let shift = 0; shift < (lane.closed ? count : 1); shift++) {
      const candidate = sequence.slice(shift).concat(sequence.slice(0, shift));
      const score = left.reduce(
        (sum, p, i) =>
          sum + (p[0] - candidate[i][0]) ** 2 + (p[1] - candidate[i][1]) ** 2,
        0,
      );
      if (score < cost) {
        best = candidate;
        cost = score;
      }
    }
  }
  lane.left = left;
  lane.right = best;
}
