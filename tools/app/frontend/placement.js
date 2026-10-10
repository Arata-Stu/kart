import { appendPair, compatible } from "./paired.js";

// Use the same insertion operation as a click, on copies only.
export function placementPreview(lane, target, point, width, obstacle) {
  if (target === "pair") {
    if (!compatible(lane) || !(width >= 0.05 && width <= 100)) return null;
    const next = {
      left: lane.left.map((p) => [...p]),
      right: lane.right.map((p) => [...p]),
      closed: lane.closed,
    };
    appendPair(next, point, width);
    return { ...next, point, width };
  }
  const points = target === "obstacle" ? obstacle?.polygon : lane[target];
  if (!points) return null;
  return {
    points: [...points.map((p) => [...p]), [...point]],
    closed: target === "obstacle" || lane.closed,
    point,
  };
}
