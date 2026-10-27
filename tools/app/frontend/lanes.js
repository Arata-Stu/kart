import { $, state, action, on, emit, remember, toast } from "./api.js";

function update() {
  const speed = $("centerline-speed");
  speed.disabled = !state.lane || state.doc?.snapshot_status === "pending";
  if (document.activeElement !== speed) speed.value = state.lane?.centerline_speed_mps ?? 1.5;
  const select = $("lane-select");
  const ids = state.doc?.lanes.map((lane) => lane.id) || [];
  if (
    JSON.stringify([...select.options].map((o) => o.value)) !==
    JSON.stringify(ids)
  ) {
    select.replaceChildren();
    for (const id of ids) {
      const option = document.createElement("option");
      option.value = id;
      option.textContent = id;
      select.append(option);
    }
  }
  select.value = state.laneId || "";
  select.disabled = !state.doc || state.doc.snapshot_status === "pending";
  $("lane-add").disabled =
    !state.doc || state.doc.snapshot_status === "pending" || ids.length >= 64;
  $("lane-remove").disabled =
    !state.doc || state.doc.snapshot_status === "pending" || ids.length <= 1;
}
function changeCenterSpeed(reportInvalid) {
  if (!state.lane || state.doc.snapshot_status === "pending") return;
  const input = $("centerline-speed"), value = Number(input.value);
  if (!input.value.trim() || !Number.isFinite(value) || value < 0 || value > 20) {
    if (!reportInvalid) return;
    input.value = state.lane.centerline_speed_mps ?? 1.5;
    toast("Center速度は0〜20 m/sで指定してください", true);
    return;
  }
  if (value === (state.lane.centerline_speed_mps ?? 1.5)) return;
  remember();
  state.lane.centerline_speed_mps = value;
  state.dirty = true;
  emit("document");
}
$("centerline-speed").addEventListener("input", () => changeCenterSpeed(false));
$("centerline-speed").addEventListener("change", () => changeCenterSpeed(true));
$("lane-select").addEventListener("change", () => {
  state.laneId = $("lane-select").value;
  state.selected = -1;
  Object.assign(state.settings, state.lane.lines.raceline?.settings || {});
  $("raceline-method").value = state.settings.optimizer;
  state.settings.centerline_method = state.lane.lines.centerline?.settings?.centerline_method || "paired";
  state.settings.region_resolution = state.lane.lines.centerline?.settings?.region_resolution ?? 0.02;
  $("centerline-method").value = state.settings.centerline_method;
  emit("document");
});
action("lane-add", () => {
  $("lane-form").reset();
  let n = 1;
  while (
    state.doc.lanes.some((l) => l.id === `lane_${String(n).padStart(3, "0")}`)
  )
    n++;
  $("lane-form").elements.lane_id.value = `lane_${String(n).padStart(3, "0")}`;
  $("lane-dialog").showModal();
});
$("lane-form").addEventListener("submit", (event) => {
  event.preventDefault();
  const id = event.target.elements.lane_id.value;
  if (
    state.doc.lanes.some((lane) => lane.id.toLowerCase() === id.toLowerCase())
  )
    return toast("同じlane IDがあります", true);
  if (state.doc.lanes.length >= 64) return toast("laneは64個までです", true);
  const lane = event.target.elements.duplicate.checked
    ? structuredClone(state.lane)
    : { closed: true, left: [], right: [], custom: [], custom_speeds: [], centerline_speed_mps: 1.5 };
  lane.id = id;
  lane.lines = {};
  remember();
  state.doc.lanes.push(lane);
  state.laneId = id;
  state.selected = -1;
  state.dirty = true;
  $("lane-dialog").close();
  emit("document");
});
let deleting = null;
action("lane-remove", () => {
  deleting = state.laneId;
  $("lane-remove-description").textContent = `「${deleting}」を削除します。`;
  $("lane-remove-dialog").showModal();
});
action("lane-remove-confirm", () => {
  if (
    !state.doc ||
    state.doc.snapshot_status === "pending" ||
    state.doc.lanes.length <= 1 ||
    deleting !== state.laneId
  )
    throw Error("対象laneを確認してください");
  remember();
  state.doc.lanes = state.doc.lanes.filter((lane) => lane.id !== deleting);
  state.laneId = state.doc.lanes[0].id;
  state.selected = -1;
  state.dirty = true;
  $("lane-remove-dialog").close();
  emit("document");
});
on("document", update);
on("status", update);

let resetting = null;
action("hdmap-reset", () => {
  resetting = state.id;
  $("hdmap-reset-dialog").showModal();
});
action("hdmap-reset-confirm", () => {
  if (
    resetting !== state.id ||
    !state.lane ||
    state.doc.snapshot_status === "pending"
  )
    throw Error("対象地図を確認してください");
  remember();
  state.doc.lanes = [
    {
      id: state.laneId,
      closed: true,
      left: [],
      right: [],
      custom: [],
      custom_speeds: [],
      centerline_speed_mps: 1.5,
      lines: {},
    },
  ];
  state.doc.obstacles = [];
  state.doc.parking_areas = [];
  state.doc.parking_drivable = [];
  emit("parking-reset-editor");
  state.obstacleId = null;
  state.selected = -1;
  state.target = "pair";
  state.dirty = true;
  $("hdmap-reset-dialog").close();
  $("step-bounds").click();
  emit("document");
  toast("HDMapをリセットしました。保存で確定、Undoで元に戻せます");
});
