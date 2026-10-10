// Preserve a closed path's start; open paths start at the previous endpoint.
export function reversePoints(points, closed) {
  return closed && points.length ? [points[0], ...points.slice(1).reverse()] : [...points].reverse();
}
export function reverseLane(lane) {
  const total = lane.custom.reduce((s, p, i, a) => {
    const q = a[i + 1] || (lane.closed ? a[0] : p);
    return s + Math.hypot(q[0] - p[0], q[1] - p[1]);
  }, 0);
  [lane.left, lane.right] = [reversePoints(lane.right, lane.closed), reversePoints(lane.left, lane.closed)];
  lane.custom = reversePoints(lane.custom, lane.closed);
  lane.custom_speeds = (lane.custom_speeds || []).map(({start, end, speed}) => ({start: total - end, end: total - start, speed}));
  // Headings, speeds (asymmetric accel/decel), and swept footprints must be recomputed.
  lane.lines = {};
}
