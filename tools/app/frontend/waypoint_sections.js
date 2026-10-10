import { $, state } from "./api.js";
export function waypointStations(points, closed) {
  const stations = [0];
  for (let i = 1; i < points.length; i++)
    stations.push(
      stations.at(-1) +
        Math.hypot(
          points[i][0] - points[i - 1][0],
          points[i][1] - points[i - 1][1],
        ),
    );
  if (closed && points.length)
    stations.push(
      stations.at(-1) +
        Math.hypot(
          points[0][0] - points.at(-1)[0],
          points[0][1] - points.at(-1)[1],
        ),
    );
  return stations;
}
let active = null;
export function resetWaypointPicker() {
  active = null;
}
export function waypointSelect(key, value) {
  const select = document.createElement("select");
  select.dataset.key = key;
  select.setAttribute(
    "aria-label",
    key === "start" ? "開始waypoint" : "終了waypoint",
  );
  const pts = state.lane.custom;
  const stations = waypointStations(pts, state.lane.closed);
  stations.forEach((s, i) =>
    select.add(
      new Option(
        i === pts.length ? "WP 1（周回末尾）" : `WP ${i + 1}`,
        String(s),
      ),
    ),
  );
  // Preserve older distance-based sections exactly, without silently snapping.
  if (!stations.some((s) => s === value))
    select.add(new Option(`既存位置 ${value.toFixed(2)} m`, String(value)));
  select.value = String(value);
  select.addEventListener("focus", () => {
    active = select;
    renderWaypointPicker();
  });
  select.addEventListener("change", renderWaypointPicker);
  return select;
}
export function renderWaypointPicker() {
  const svg = $("speed-waypoints"),
    pts = state.lane.custom;
  svg.replaceChildren();
  if (!active?.isConnected) active = $("speed-rows").querySelector("select");
  $("speed-pick-status").textContent = active
    ? `${[...$("speed-rows").children].indexOf(active.parentElement) + 1}番の区間：${active.dataset.key === "start" ? "開始" : "終了"}WPをクリック（進行順に選択）`
    : "「区間を追加」で開始します";
  if (!pts.length) return;
  const xs = pts.map((p) => p[0]),
    ys = pts.map((p) => p[1]);
  const xmin = Math.min(...xs),
    ymin = Math.min(...ys);
  const sx = Math.max(...xs) - xmin,
    sy = Math.max(...ys) - ymin;
  const scale = Math.min(460 / Math.max(sx, 0.01), 180 / Math.max(sy, 0.01));
  const project = (p) => [
    260 + (p[0] - xmin - sx / 2) * scale,
    120 - (p[1] - ymin - sy / 2) * scale,
  ];
  const stations = waypointStations(pts, state.lane.closed);
  const row = active?.parentElement;
  const start = Number(row?.querySelector('[data-key="start"]').value);
  const end = Number(row?.querySelector('[data-key="end"]').value);
  function element(tag, attrs) {
    const el = document.createElementNS("http://www.w3.org/2000/svg", tag);
    for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
    svg.append(el);
    return el;
  }
  for (let i = 0; i < pts.length - (state.lane.closed ? 0 : 1); i++) {
    const j = (i + 1) % pts.length;
    const cuts = [stations[i], stations[i + 1], start, end]
      .filter((s) => s >= stations[i] && s <= stations[i + 1])
      .sort((a, b) => a - b);
    for (let k = 1; k < cuts.length; k++) {
      const a = cuts[k - 1],
        b = cuts[k],
        mid = (a + b) / 2;
      if (a === b) continue;
      const hit =
        start < end
          ? mid >= start && mid <= end
          : start > end && (mid >= start || mid <= end);
      const interp = (s) =>
        project(
          pts[i].map(
            (v, n) =>
              v +
              ((pts[j][n] - v) * (s - stations[i])) /
                Math.max(1e-12, stations[i + 1] - stations[i]),
          ),
        );
      const p = interp(a),
        q = interp(b);
      element("line", {
        x1: p[0],
        y1: p[1],
        x2: q[0],
        y2: q[1],
        stroke: hit ? "#9063b0" : "#bac6ce",
        "stroke-width": hit ? 6 : 2,
      });
    }
  }
  pts.forEach((p, i) => {
    const [x, y] = project(p);
    const dot = element("circle", {
      cx: x,
      cy: y,
      r: 7,
      fill:
        stations[i] === start
          ? "#339675"
          : stations[i] === end
            ? "#e78f64"
            : "white",
      stroke: "#9063b0",
      tabindex: 0,
      role: "button",
      "aria-label": `WP ${i + 1}`,
    });
    const pick = () => {
      if (!active) return;
      active.value = String(stations[i]);
      if (active.dataset.key === "start")
        active = active.parentElement.querySelector('[data-key="end"]');
      renderWaypointPicker();
    };
    dot.addEventListener("click", pick);
    dot.addEventListener("keydown", (e) => {
      if (e.key === "Enter" || e.key === " ") {
        e.preventDefault();
        pick();
      }
    });
    const text = element("text", {
      x: x + 9,
      y: y - 8,
      "font-size": 11,
      fill: "#354959",
      "pointer-events": "none",
    });
    text.textContent = String(i + 1);
  });
}
