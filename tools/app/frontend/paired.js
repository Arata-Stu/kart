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
export function appendPair(lane, p, width) {
  const mid = centers(lane),
    prev = mid.at(-1);
  const dx = prev ? p[0] - prev[0] : 1,
    dy = prev ? p[1] - prev[1] : 0;
  const len = Math.hypot(dx, dy) || 1;
  if (mid.length === 1) {
    const firstWidth = Math.hypot(
      lane.left[0][0] - lane.right[0][0],
      lane.left[0][1] - lane.right[0][1],
    );
    lane.left[0] = [
      prev[0] - ((dy / len) * firstWidth) / 2,
      prev[1] + ((dx / len) * firstWidth) / 2,
    ];
    lane.right[0] = [
      prev[0] + ((dy / len) * firstWidth) / 2,
      prev[1] - ((dx / len) * firstWidth) / 2,
    ];
  }
  lane.left.push([
    p[0] - ((dy / len) * width) / 2,
    p[1] + ((dx / len) * width) / 2,
  ]);
  lane.right.push([
    p[0] + ((dy / len) * width) / 2,
    p[1] - ((dx / len) * width) / 2,
  ]);
}
export function resizePairs(lane, width, selected = -1) {
  const mid = centers(lane);
  for (let i = 0; i < mid.length; i++) {
    if (selected >= 0 && i !== selected) continue;
    let dx = lane.left[i][0] - lane.right[i][0],
      dy = lane.left[i][1] - lane.right[i][1];
    const length = Math.hypot(dx, dy);
    if (!length) throw Error("幅0の区間は左右境界を個別に修正してください");
  }
  for (let i = 0; i < mid.length; i++) {
    if (selected >= 0 && i !== selected) continue;
    const dx = lane.left[i][0] - lane.right[i][0],
      dy = lane.left[i][1] - lane.right[i][1];
    const factor = width / (2 * Math.hypot(dx, dy));
    lane.left[i] = [mid[i][0] + dx * factor, mid[i][1] + dy * factor];
    lane.right[i] = [mid[i][0] - dx * factor, mid[i][1] - dy * factor];
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
