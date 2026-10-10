import { $, state, action, on, emit, remember, toast } from "./api.js";

function update() {
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
$("lane-select").addEventListener("change", () => {
  state.laneId = $("lane-select").value;
  state.selected = -1;
  Object.assign(state.settings, state.lane.lines.raceline?.settings || {});
  $("raceline-method").value = state.settings.optimizer;
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
    : { closed: true, left: [], right: [], custom: [], custom_speeds: [] };
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
      lines: {},
    },
  ];
  state.doc.obstacles = [];
  state.obstacleId = null;
  state.selected = -1;
  state.target = "pair";
  state.dirty = true;
  $("hdmap-reset-dialog").close();
  $("step-bounds").click();
  emit("document");
  toast("HDMapをリセットしました。保存で確定、Undoで元に戻せます");
});
