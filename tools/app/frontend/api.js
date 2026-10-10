export const $ = (id) => document.getElementById(id);
export const state = {
  id: null,
  doc: null,
  laneId: null,
  get lane() {
    return this.doc?.lanes.find((lane) => lane.id === this.laneId);
  },
  obstacleId: null,
  get obstacle() {
    return this.doc?.obstacles?.find((o) => o.id === this.obstacleId);
  },
  cloud: null,
  dirty: false,
  step: "cloud",
  target: "left",
  selected: -1,
  zmin: -Infinity,
  zmax: Infinity,
  showPath: true,
  undo: [],
  redo: [],
  config: null,
  maps: [],
  records: [],
  jobs: [],
  settings: {
    spacing: 0.10,
    vehicle_width: 0.19,
    vehicle_length: 0.47,
    rear_axle_to_rear: 0.1065,
    margin: 0.05,
    max_speed: 3,
    lateral_accel: 2.5,
    accel: 1.5,
    decel: 2.5,
    curvature_limit: 1,
    optimizer: "local",
  },
};
export const events = new EventTarget();
export const emit = (name, detail) =>
  events.dispatchEvent(new CustomEvent(name, { detail }));
export const on = (name, fn) =>
  events.addEventListener(name, (event) => {
    Promise.resolve()
      .then(() => fn(event))
      .catch((error) => toast(error.message, true));
  });
export async function api(path, body) {
  const response = await fetch(
    "/api/" + path,
    body === undefined
      ? {}
      : {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(body),
        },
  );
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || "処理に失敗しました");
  return result;
}
let toastTimer;
export function toast(message, error = false) {
  clearTimeout(toastTimer);
  $("toast").textContent = message;
  $("toast").className = error ? "error" : "";
  $("toast").hidden = false;
  toastTimer = setTimeout(
    () => ($("toast").hidden = true),
    error ? 12000 : 7000,
  );
}
export function action(id, fn) {
  $(id).addEventListener("click", async () => {
    const button = $(id);
    if (button.disabled) return;
    button.disabled = true;
    try {
      await fn();
    } catch (e) {
      toast(e.message, true);
    } finally {
      button.disabled = false;
      emit("status");
    }
  });
}
export function item(title, detail, fn, selected = false) {
  const button = document.createElement("button");
  button.className = "record-item" + (selected ? " selected" : "");
  const strong = document.createElement("strong");
  strong.textContent = title;
  const small = document.createElement("small");
  small.textContent = detail;
  button.append(strong, small);
  button.addEventListener("click", () =>
    Promise.resolve(fn()).catch((e) => toast(e.message, true)),
  );
  return button;
}
export const fmt = (v) => Number(v).toLocaleString("ja-JP");
function editSnapshot() {
  return structuredClone({
    lanes: state.doc.lanes,
    laneId: state.laneId,
    obstacles: state.doc.obstacles || [],
    obstacleId: state.obstacleId,
  });
}
export function remember() {
  state.undo.push(editSnapshot());
  if (state.undo.length > 60) state.undo.shift();
  state.redo = [];
}
export function edited() {
  state.dirty = true;
  if (state.target === "obstacle") {
    for (const lane of state.doc.lanes) lane.lines = {};
  } else if (state.target === "custom") {
    delete state.lane.lines.customline;
    if (state.lane.custom.length < (state.lane.closed ? 3 : 2))
      state.lane.custom_speeds = [];
  } else state.lane.lines = {};
  emit("document");
}
export function undo(redo = false) {
  if (!state.doc) return;
  const from = redo ? state.redo : state.undo,
    to = redo ? state.undo : state.redo;
  if (!from.length) return;
  to.push(editSnapshot());
  const snapshot = from.pop();
  state.doc.lanes = snapshot.lanes;
  state.laneId = snapshot.laneId;
  state.doc.obstacles = snapshot.obstacles;
  state.obstacleId = snapshot.obstacleId;
  state.selected = -1;
  state.dirty = true;
  emit("document");
}
