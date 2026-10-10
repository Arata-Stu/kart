import { placementPreview } from "./placement.js";
import { centers, compatible, movePair, appendPair } from "./paired.js";
import { $, state, on, emit, remember, edited, undo } from "./api.js";
const canvas = $("map-canvas"),
  ctx = canvas.getContext("2d");
let width = 1,
  height = 1,
  scale = 25,
  origin = [0, 0],
  drag = null,
  hover = null,
  frame = 0;
let cloudLayer = document.createElement("canvas"),
  cloudDirty = true;
const colors = {
  left: "#328ccb",
  right: "#e78f64",
  centerline: "#339675",
  raceline: "#b28231",
  customline: "#9063b0",
  custom: "#9063b0",
  pair: "#339675",
  obstacle: "#ce4141",
};
const project = (p) => [
  width / 2 + (p[0] - origin[0]) * scale,
  height / 2 - (p[1] - origin[1]) * scale,
];
const world = (x, y) => [
  origin[0] + (x - width / 2) / scale,
  origin[1] - (y - height / 2) / scale,
];
const local = (e) => {
  const r = canvas.getBoundingClientRect();
  return [e.clientX - r.left, e.clientY - r.top];
};
function editPoints() {
  return state.target === "pair"
    ? centers(state.lane)
    : state.target === "obstacle"
      ? state.obstacle?.polygon || []
      : state.lane[state.target];
}
function nearestPoint(x, y) {
  let index = -1,
    best = 12;
  editPoints().forEach((p, i) => {
    const q = project(p),
      distance = Math.hypot(q[0] - x, q[1] - y);
    if (distance < best) {
      best = distance;
      index = i;
    }
  });
  return index;
}
function paintPlacement() {
  if (
    !hover ||
    hover.alt ||
    drag ||
    !state.lane ||
    state.doc?.snapshot_status === "pending" ||
    state.registrationPreview
  )
    return;
  if (
    !(
      state.step === "bounds" ||
      (state.step === "lines" && state.target === "custom")
    )
  )
    return;
  if (state.target === "pair" && !compatible(state.lane)) return;
  const [x, y] = hover.xy;
  const near = nearestPoint(x, y);
  let label = "ドラッグで点を移動";
  ctx.save();
  if (near >= 0) {
    const q = project(editPoints()[near]);
    ctx.beginPath();
    ctx.arc(q[0], q[1], 10, 0, Math.PI * 2);
    ctx.strokeStyle = colors[state.target];
    ctx.lineWidth = 2;
    ctx.stroke();
  } else {
    const preview = placementPreview(
      state.lane,
      state.target,
      world(x, y),
      Number($("pair-width").value),
      state.obstacle,
    );
    if (!preview) {
      ctx.restore();
      return;
    }
    if (state.target === "pair") {
      // Fill only the prospective section, keeping the cloud readable.
      const start = Math.max(0, preview.left.length - 2);
      const polygon = [
        ...preview.left.slice(start),
        ...preview.right.slice(start).reverse(),
      ];
      line(polygon, "#33967566", true, true, 1);
      ctx.fillStyle = "#33967522";
      ctx.fill();
      line(preview.left, colors.left, preview.closed, true, 2);
      line(preview.right, colors.right, preview.closed, true, 2);
      line(
        [preview.left.at(-1), preview.right.at(-1)],
        colors.pair,
        false,
        true,
      );
      for (const p of [preview.left.at(-1), preview.right.at(-1)]) {
        const q = project(p);
        ctx.beginPath();
        ctx.arc(...q, 5, 0, Math.PI * 2);
        ctx.fillStyle = "#ffffff";
        ctx.fill();
        ctx.stroke();
      }
      label = `クリックでlane追加 · 区間幅 ${preview.width.toFixed(2)} m`;
      if (!state.lane.left.length) label += "（向きは2点目で決定）";
    } else {
      line(preview.points.slice(-2), colors[state.target], false, true);
      if (preview.closed && preview.points.length >= 3)
        line(
          [preview.point, preview.points[0]],
          colors[state.target],
          false,
          true,
          1,
        );
      label = "クリックで点を追加";
    }
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, Math.PI * 2);
    ctx.fillStyle = "#fff";
    ctx.fill();
    ctx.strokeStyle = colors[state.target];
    ctx.lineWidth = 2;
    ctx.stroke();
  }
  ctx.font = "12px sans-serif";
  ctx.textAlign = "left";
  const textWidth = ctx.measureText(label).width;
  const tx = Math.max(8, Math.min(x + 14, width - textWidth - 16));
  const ty = Math.max(22, y - 16);
  ctx.fillStyle = "#fffffff0";
  ctx.fillRect(tx - 5, ty - 15, textWidth + 10, 22);
  ctx.fillStyle = "#254455";
  ctx.fillText(label, tx, ty);
  ctx.restore();
}
function invalidate() {
  cloudDirty = true;
  draw();
}
export function fit() {
  const values = state.cloud?.points || [];
  if (!values.length) return;
  let x0 = Infinity,
    x1 = -Infinity,
    y0 = Infinity,
    y1 = -Infinity;
  for (const p of values) {
    x0 = Math.min(x0, p[0]);
    x1 = Math.max(x1, p[0]);
    y0 = Math.min(y0, p[1]);
    y1 = Math.max(y1, p[1]);
  }
  origin = [(x0 + x1) / 2, (y0 + y1) / 2];
  scale = Math.min(
    (width - 100) / Math.max(1, x1 - x0),
    (height - 100) / Math.max(1, y1 - y0),
  );
  invalidate();
}
function line(points, color, closed = false, dashed = false, lineWidth = 2) {
  if (!points?.length) return;
  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.setLineDash(dashed ? [4, 5] : []);
  points.forEach((p, i) => {
    const [x, y] = project(p);
    i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
  });
  if (closed) ctx.closePath();
  ctx.stroke();
  ctx.setLineDash([]);
}
function directionArrows(points, color, closed) {
  if (!points || points.length < 2) return;
  for (
    let i = 0;
    i < points.length - (closed ? 0 : 1);
    i += Math.max(1, Math.floor(points.length / 12))
  ) {
    const a = project(points[i]),
      b = project(points[(i + 1) % points.length]);
    const angle = Math.atan2(b[1] - a[1], b[0] - a[0]);
    if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 1) continue;
    ctx.save();
    ctx.translate(a[0], a[1]);
    ctx.rotate(angle);
    ctx.beginPath();
    ctx.moveTo(5, 0);
    ctx.lineTo(-4, -4);
    ctx.lineTo(-4, 4);
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
    ctx.restore();
  }
}
function paintCloud() {
  cloudLayer.width = canvas.width;
  cloudLayer.height = canvas.height;
  const c = cloudLayer.getContext("2d");
  // Match the actual canvas backing scale, even during a display-DPR change.
  c.setTransform(canvas.width / width, 0, 0, canvas.height / height, 0, 0);
  let visible = 0;
  for (const p of state.cloud?.points || []) {
    if (p[2] < state.zmin || p[2] > state.zmax) continue;
    visible++;
    const [x, y] = project(p);
    if (x < 0 || x > width || y < 0 || y > height) continue;
    const t = Math.max(
      0,
      Math.min(
        1,
        (p[2] - state.zmin) / Math.max(0.01, state.zmax - state.zmin),
      ),
    );
    c.fillStyle = `hsla(${195 - 35 * t},18%,${54 - 14 * t}%,.68)`;
    c.fillRect(x, y, 1.7, 1.7);
  }
  $("point-count").textContent =
    `${visible.toLocaleString()} / ${(state.cloud?.source_count || 0).toLocaleString()}`;
  cloudDirty = false;
}
export function draw() {
  if (frame) return;
  frame = requestAnimationFrame(() => {
    frame = 0;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#f4f6f8";
    ctx.fillRect(0, 0, width, height);
    const spacing = Math.pow(
      10,
      Math.floor(Math.log10(80 / Math.max(scale, 0.0001))),
    );
    ctx.strokeStyle = "#e5eaee";
    ctx.lineWidth = 1;
    const lo = world(0, height),
      hi = world(width, 0);
    ctx.beginPath();
    for (
      let x = Math.floor(lo[0] / spacing) * spacing;
      x < hi[0];
      x += spacing
    ) {
      const px = project([x, 0])[0];
      ctx.moveTo(px, 0);
      ctx.lineTo(px, height);
    }
    for (
      let y = Math.floor(lo[1] / spacing) * spacing;
      y < hi[1];
      y += spacing
    ) {
      const py = project([0, y])[1];
      ctx.moveTo(0, py);
      ctx.lineTo(width, py);
    }
    ctx.stroke();
    if (cloudDirty) paintCloud();
    ctx.drawImage(cloudLayer, 0, 0, width, height);
    if (state.showPath)
      line(state.cloud?.trajectory, "#a8b6c1", false, true, 1.3);
    const d = state.lane;
    if (d) {
      for (const other of state.doc.lanes) {
        if (other.id === d.id) continue;
        line(other.left, "#a6adb3", other.closed, true, 1);
        line(other.right, "#a6adb3", other.closed, true, 1);
      }
      for (const o of state.doc.obstacles || []) {
        line(
          o.polygon,
          colors.obstacle,
          true,
          false,
          o.id === state.obstacleId ? 3 : 2,
        );
        if (o.polygon.length >= 3) {
          ctx.fillStyle = "#ce414133";
          ctx.fill();
        }
        if (o.polygon.length) {
          const p = project(o.polygon[0]);
          ctx.fillStyle = colors.obstacle;
          ctx.fillText(o.id, p[0] + 8, p[1] - 8);
        }
      }
      line(d.left, colors.left, d.closed);
      line(d.right, colors.right, d.closed);
      for (const [kind, data] of Object.entries(d.lines)) {
        line(data.points, colors[kind], d.closed, false, 2.5);
        directionArrows(data.points, colors[kind], d.closed);
      }
      if (!Object.keys(d.lines).length)
        directionArrows(d.left, colors.left, d.closed);
      if (state.step === "lines") {
        if (state.target === "custom")
          line(d.custom, colors.custom, d.closed, true);
        let station = 0,
          lastLabel = -Infinity;
        ctx.textAlign = "left";
        ctx.font = "11px sans-serif";
        for (const [i, p] of d.custom.entries()) {
          const [x, y] = project(p);
          ctx.fillStyle = colors.custom;
          if (i === 0 || station - lastLabel > Math.max(1, 85 / scale)) {
            ctx.fillText(i === 0 ? "○ 0 m" : station.toFixed(1), x + 8, y - 7);
            lastLabel = station;
          }
          if (i < d.custom.length - 1)
            station += Math.hypot(
              d.custom[i + 1][0] - p[0],
              d.custom[i + 1][1] - p[1],
            );
        }
        const custom = d.lines.customline;
        if (custom?.speed_sections?.length) {
          for (let i = 0; i < custom.points.length - (d.closed ? 0 : 1); i++) {
            const j = (i + 1) % custom.points.length;
            const v = Math.min(custom.profile[i][5], custom.profile[j][5]);
            const ratio = v / custom.settings.max_speed;
            line(
              [custom.points[i], custom.points[j]],
              `hsl(${25 + 200 * ratio},75%,43%)`,
              false,
              false,
              3,
            );
          }
          ctx.fillStyle = "#647c8c";
          ctx.fillText("Custom速度: 橙 低速 → 青 高速", 14, 22);
        }
      }
      if (state.target === "pair" && state.step === "bounds" && compatible(d)) {
        line(centers(d), colors.pair, d.closed, true, 1);
        for (let i = 0; i < d.left.length; i++)
          line([d.left[i], d.right[i]], "#33967555", false, false, 1);
      }
      if (state.registrationPreview) {
        for (const o of state.registrationPreview.obstacles || [])
          line(o.polygon, "#119caa", true, true, 3);
        for (const preview of state.registrationPreview.lanes) {
          for (const key of ["left", "right"])
            line(preview[key], "#119caa", preview.closed, true, 3);
          for (const data of Object.values(preview.lines))
            line(data.points, "#119caa", preview.closed, true, 2);
        }
      }
      const target =
        state.step === "bounds"
          ? state.target
          : state.step === "lines" && state.target === "custom"
            ? "custom"
            : null;
      if (target)
        for (const [i, p] of editPoints().entries()) {
          const [x, y] = project(p);
          ctx.beginPath();
          ctx.arc(x, y, i === state.selected ? 6 : 4, 0, Math.PI * 2);
          ctx.fillStyle = i === state.selected ? colors[target] : "#fff";
          ctx.fill();
          ctx.strokeStyle = colors[target];
          ctx.lineWidth = 2;
          ctx.stroke();
        }
    }
    paintPlacement();
    const bar = spacing * scale;
    ctx.strokeStyle = "#647c8c";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(width - 25 - bar, height - 28);
    ctx.lineTo(width - 25, height - 28);
    ctx.stroke();
    ctx.fillStyle = "#647c8c";
    ctx.font = "10px sans-serif";
    ctx.textAlign = "right";
    ctx.fillText(`${spacing.toPrecision(1)} m`, width - 25, height - 36);
  });
}
new ResizeObserver(() => {
  const r = canvas.getBoundingClientRect(),
    dpr = devicePixelRatio || 1;
  width = Math.max(1, r.width);
  height = Math.max(1, r.height);
  canvas.width = width * dpr;
  canvas.height = height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  invalidate();
}).observe(canvas);
canvas.addEventListener("contextmenu", (e) => e.preventDefault());
canvas.addEventListener(
  "wheel",
  (e) => {
    e.preventDefault();
    const [x, y] = local(e),
      before = world(x, y);
    scale = Math.max(
      0.01,
      Math.min(3000, scale * Math.exp(-e.deltaY * 0.0015)),
    );
    const after = world(x, y);
    origin[0] += before[0] - after[0];
    origin[1] += before[1] - after[1];
    invalidate();
  },
  { passive: false },
);
canvas.addEventListener("pointerdown", (e) => {
  if (e.button !== 0 && e.button !== 1 && e.button !== 2) return;
  const [x, y] = local(e);
  canvas.focus();
  canvas.setPointerCapture(e.pointerId);
  const edit =
    state.doc &&
    (state.step === "bounds" ||
      (state.step === "lines" && state.target === "custom"));
  if (!edit || e.button !== 0 || e.altKey) {
    drag = { mode: "pan", x, y, origin: [...origin] };
    return;
  }
  if (state.target === "pair" && !compatible(state.lane)) return;
  if (state.target === "obstacle" && !state.obstacle) return;
  const index = nearestPoint(x, y);
  remember();
  state.selected = index;
  drag = { mode: "edit", x, y, index, moved: false };
  draw();
  emit("status");
});
canvas.addEventListener("pointermove", (e) => {
  const [x, y] = local(e),
    p = world(x, y);
  $("coordinates").textContent =
    `X ${p[0].toFixed(2)} · Y ${p[1].toFixed(2)} m`;
  hover = { xy: [x, y], alt: e.altKey };
  if (!drag) {
    draw();
    return;
  }
  if (drag.mode === "pan") {
    origin = [
      drag.origin[0] - (x - drag.x) / scale,
      drag.origin[1] + (y - drag.y) / scale,
    ];
    invalidate();
  } else if (drag.index >= 0 && Math.hypot(x - drag.x, y - drag.y) > 2) {
    if (state.target === "pair") movePair(state.lane, drag.index, p);
    else editPoints()[drag.index] = p;
    drag.moved = true;
    edited();
  }
});
canvas.addEventListener("pointerup", (e) => {
  if (drag?.mode === "edit") {
    if (
      drag.index < 0 &&
      Math.hypot(...local(e).map((v, i) => v - [drag.x, drag.y][i])) < 5
    ) {
      if (state.target === "pair") {
        const width = Number($("pair-width").value);
        if (!(width >= 0.05 && width <= 100)) {
          drag = null;
          state.undo.pop();
          return;
        }
        appendPair(state.lane, world(...local(e)), width);
        state.selected = state.lane.left.length - 1;
      } else {
        editPoints().push(world(...local(e)));
        state.selected = editPoints().length - 1;
      }
      edited();
    } else if (!drag.moved) {
      state.undo.pop();
    }
  }
  drag = null;
  draw();
});
canvas.addEventListener("pointerleave", () => {
  hover = null;
  draw();
});
$("pair-width").addEventListener("input", draw);
canvas.addEventListener("pointercancel", () => {
  hover = null;
  drag = null;
  draw();
});
canvas.addEventListener("keydown", (e) => {
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") {
    e.preventDefault();
    undo(e.shiftKey);
  }
  if (
    (e.key === "Delete" || e.key === "Backspace") &&
    state.doc &&
    state.selected >= 0 &&
    state.step !== "cloud"
  ) {
    e.preventDefault();
    remember();
    if (state.target === "pair") {
      state.lane.left.splice(state.selected, 1);
      state.lane.right.splice(state.selected, 1);
    } else editPoints().splice(state.selected, 1);
    state.selected = -1;
    edited();
  }
});
on("document", () => {
  if (state.target === "pair" && state.lane && !compatible(state.lane))
    state.target = "left";
  draw();
});
on("view", invalidate);
on("loaded", () => {
  hover = null;
  fit();
});
