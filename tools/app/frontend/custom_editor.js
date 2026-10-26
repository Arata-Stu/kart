import { $, state, action, remember, emit, on, toast } from "./api.js";
import { copyCustom, downsampleCustom, clearCustom } from "./custom_model.js";
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
  $("custom-spacing").disabled = unavailable;
  $("custom-downsample").disabled = unavailable || !lane.custom.length;
  $("custom-delete").disabled = unavailable || !(lane.custom.length || lane.lines.customline || lane.custom_speeds.length);
  try {
    const points = source === "manual" ? lane?.custom : lane?.lines[source]?.points;
    $("custom-point-preview").textContent = points?.length
      ? `${source === "manual" ? "現在の点" : "コピー"}：${points.length} → ${downsampleCustom(points, lane.closed, Number($("custom-spacing").value)).length} 点`
      : "0 mで間引きなし。点間隔は目安です。";
  } catch (error) { $("custom-point-preview").textContent = error.message; }
  $("custom-source-help").textContent = lane?.custom.length
    ? `現在 ${lane.custom.length} 点。作成すると既存Customと区間速度を置き換えます（Undo可）。`
    : "手動で描くか、生成済みのCenter／Raceをコピーして始めます。";
}
$("custom-source").addEventListener("change", render);
$("custom-spacing").addEventListener("input", render);
action("custom-start", () => {
  if (!state.lane || state.doc.snapshot_status === "pending") return;
  const points = copyCustom(state.lane, $("custom-source").value, Number($("custom-spacing").value));
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
action("custom-downsample", () => {
  const lane = state.lane;
  const points = downsampleCustom(lane.custom, lane.closed, Number($("custom-spacing").value));
  if (points.length === lane.custom.length) { toast("点数は変わりません。点間隔を大きくしてください"); return; }
  remember();
  const before = lane.custom.length;
  lane.custom = points;
  lane.custom_speeds = [];
  delete lane.lines.customline;
  state.target = "custom"; showLine("customline");
  state.selected = -1; state.diagnostic = null; state.dirty = true;
  emit("document");
  toast(`${before} → ${points.length} 点に間引きました。区間速度を設定し直して再生成してください`);
});
action("custom-delete", () => {
  remember();
  clearCustom(state.lane);
  state.selected = -1; state.diagnostic = null; state.dirty = true;
  emit("document");
  toast("Customの編集点・区間速度・生成結果を削除しました（保存前はUndoで戻せます）");
});
for (const name of ["document", "status", "ready"]) on(name, render);
