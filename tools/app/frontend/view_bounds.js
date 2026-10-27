// View fitting only: never remove points from the cloud or edit map geometry.
const finite = (p) => p && Number.isFinite(p[0]) && Number.isFinite(p[1]);
function bounds(points) {
  if (!points.length) return null;
  const result = [Infinity, Infinity, -Infinity, -Infinity];
  for (const [x, y] of points) {
    result[0] = Math.min(result[0], x);
    result[1] = Math.min(result[1], y);
    result[2] = Math.max(result[2], x);
    result[3] = Math.max(result[3], y);
  }
  return result;
}
const median = (sorted) => {
  const i = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[i] : (sorted[i - 1] + sorted[i]) / 2;
};
export function viewBounds(doc, cloud = [], all = false) {
  const lanes = doc?.lanes || [];
  const parking = (doc?.parking_areas || []).flatMap(a => [...(a.polygon || []), ...(a.entrance || []), ...(a.goal_pose ? [a.goal_pose] : [])]).filter(finite);
  const course = lanes.flatMap((lane) => [...(lane.left || []), ...(lane.right || [])]).filter(finite);
  const lines = lanes.flatMap((lane) => [
    ...(lane.custom || []),
    ...Object.values(lane.lines || {}).flatMap((line) => line?.points || []),
  ]).filter(finite);
  const points = cloud.filter(finite);
  if (all) return bounds([...points, ...course, ...lines, ...parking]);
  if (course.length >= 2) return bounds([...course,...parking]);
  if (lines.length >= 2) return bounds([...lines,...parking]);
  if (points.length < 20) return bounds(points);
  // Median absolute deviation resists sparse distant points. A one-metre
  // minimum window also handles nearly straight or coincident point clouds.
  const axes = [0, 1].map((axis) => {
    const values = points.map((p) => p[axis]).sort((a, b) => a - b);
    const center = median(values);
    const deviations = values.map((v) => Math.abs(v - center)).sort((a, b) => a - b);
    return [center, Math.max(1, 4 * median(deviations))];
  });
  const core = points.filter((p) => axes.every(([center, radius], axis) => Math.abs(p[axis] - center) <= radius));
  return bounds(core.length >= points.length / 2 ? core : points);
}
