import { $, state, action, remember, emit, on, toast } from "./api.js";
import { copyCustom } from "./custom_model.js";
import { showLine } from "./line_visibility.js";

function render() {
  const source = $("custom-source").value;
  const lane = state.lane;
  const busy = state.jobs.some(j => j.status === "running" &&
    (j.title.includes(state.id) || j.title.includes("生成")));
  const unavailable = !lane || state.doc.snapshot_status === "pending" || busy;
  $("custom-source").disabled = unavailable;
  $("custom-start").disabled = unavailable || (source !== "manual" && !lane.lines[source]);
  $("custom-start").textContent = source === "manual" ? "空の点列で作成" : "コピーして編集";
  $("custom-source-help").textContent = lane?.custom.length
    ? `現在 ${lane.custom.length} 点。作成すると既存Customと区間速度を置き換えます（Undo可）。`
    : "手動で描くか、生成済みのCenter／Raceをコピーして始めます。";
}
$("custom-source").addEventListener("change", render);
action("custom-start", () => {
  if (!state.lane || state.doc.snapshot_status === "pending") return;
  const points = copyCustom(state.lane, $("custom-source").value);
  remember();
  state.lane.custom = points;
  state.lane.custom_speeds = [];
  delete state.lane.lines.customline;
  state.target = "custom";
  showLine("customline");
  state.selected = -1;
  state.diagnostic = null;
  state.dirty = true;
  emit("document");
  toast(points.length ? "コピーしました。点をドラッグして調整できます" : "地図上をクリックして点を追加してください");
});
for (const name of ["document", "status", "ready"]) on(name, render);
