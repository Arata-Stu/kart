import {
  waypointStations,
  waypointSelect,
  renderWaypointPicker,
  resetWaypointPicker,
  setSectionSplit,
  speedRows,
  selectSpeedRow,
} from "./waypoint_sections.js";
import { $, state, api, action, toast, emit, remember, on } from "./api.js";
import { partitionSections, splitSection, mergeSection, deleteSection, isPartition } from "./custom_model.js";
import { save, loadMap } from "./maps.js";

function length() {
  const pts = state.lane.custom;
  return pts.reduce((total, p, i) => {
    const q = pts[i + 1] || (state.lane.closed ? pts[0] : p);
    return total + Math.hypot(q[0] - p[0], q[1] - p[1]);
  }, 0);
}
function addRow(
  value = {
    start: 0,
    end: waypointStations(state.lane.custom, state.lane.closed).at(-1),
    speed: 1,
  },
  redraw = true,
) {
  if ($("speed-rows").children.length >= 40) return;
  const row = document.createElement("div");
  row.className = "speed-row";
  for (const key of ["start", "end", "speed"]) {
    if (key !== "speed") {
      const select = waypointSelect(key, value[key]);
      select.disabled = $("speed-mode").value === "split";
      row.append(select);
      continue;
    }
    const input = document.createElement("input");
    Object.assign(input, {
      type: "number",
      value: value[key],
      min: key === "speed" ? 0.1 : 0,
      max: key === "speed" ? 20 : length(),
      step: "any",
      required: true,
    });
    input.dataset.key = key;
    input.setAttribute(
      "aria-label",
      { start: "開始距離", end: "終了距離", speed: "区間速度 m/s" }[key],
    );
    input.addEventListener("input", renderWaypointPicker);
    row.append(input);
  }
  const remove = document.createElement("button");
  remove.type = "button";
  remove.textContent = $("speed-mode").value === "split" ? "結合" : "×";
  remove.setAttribute("aria-label", $("speed-mode").value === "split" ? "隣の区間と結合（低い速度を保持）" : "区間を削除");
  remove.onclick = () => {
    if ($("speed-mode").value === "split") {
      rebuildRows(mergeSection(speedRows(), [...$("speed-rows").children].indexOf(row)));
    } else { row.remove(); renderWaypointPicker(); }
  };
  const buttons=document.createElement("div");
  buttons.className="speed-row-actions";
  if ($("speed-mode").value === "split") buttons.append(remove);
  const erase=document.createElement("button");erase.type="button";erase.textContent="削除";
  erase.setAttribute("aria-label","この区間の速度指定を削除");
  erase.title="この区間の速度上限を解除します。隣の区間は変更しません";
  erase.onclick=()=>{
    const remaining=deleteSection(speedRows(),[...$("speed-rows").children].indexOf(row));
    $("speed-mode").value="ranges";configureMode(remaining,false);
  };
  buttons.append(erase);row.append(buttons);
  $("speed-rows").append(row);
  row.addEventListener("focusin", event => {
    if (event.target.dataset.key === "speed") selectSpeedRow(row);
  });
  if (redraw) renderWaypointPicker();
}
function rebuildRows(rows) {
  resetWaypointPicker();
  $("speed-rows").replaceChildren();
  for (const row of rows) addRow(row, false);
  renderWaypointPicker();
}
function configureMode(rows, partition = true) {
  const splitting = $("speed-mode").value === "split";
  $("speed-add").hidden = splitting;
  $("speed-mode-help").textContent = splitting
    ? "WPをクリックして区切ります。線上クリックも最寄りWPに合わせます。全体を重複のない区間に分け、行ごとに速度を設定します。"
    : "開始・終了を個別指定します。既存区間はすべて色分け表示。閉路の始点跨ぎや重複も指定できます。";
  setSectionSplit(splitting ? station => {
    try { rebuildRows(splitSection(speedRows(), station)); }
    catch (error) { toast(error.message, true); }
  } : null);
  rebuildRows(splitting && partition
    ? partitionSections(rows, length(), state.settings.max_speed) : rows);
}
$("speed-mode").addEventListener("change", () => {
  const rows = speedRows();
  try { configureMode(rows); }
  catch (error) {
    $("speed-mode").value = "ranges";
    configureMode(rows, false);
    toast(error.message, true);
  }
});
action("custom-speeds-open", () => {
  if (state.lane.custom.length < (state.lane.closed ? 3 : 2))
    throw Error("先にCustomlineの点を描いてください");
  $("speed-length").textContent = `全長 ${length().toFixed(2)} m / ${state.lane.custom.length} 編集点`;
  $("custom-speed-mode").value=state.lane.custom_speed_mode || "profile";
  const rows = state.lane.custom_speeds || [];
  // Existing ranges retain their exact distances and overlap until explicitly converted.
  const partition = isPartition(rows, length());
  $("speed-mode").value = !rows.length || partition ? "split" : "ranges";
  configureMode(rows, !partition);
  $("speed-dialog").showModal();
  renderWaypointPicker();
});
action("speed-add", () => addRow());
action("speed-clear", () => {
  $("speed-mode").value="ranges";configureMode([],false);
});
$("speed-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const limits = speedRows();
  if (
    limits.some(
      (row) =>
        row.start === row.end || (!state.lane.closed && row.start > row.end),
    )
  )
    return toast(
      "始終点を確認してください。始点を跨ぐ指定は閉路のみです",
      true,
    );
  remember();
  state.lane.custom_speeds = limits;
  state.lane.custom_speed_mode = $("custom-speed-mode").value;
  delete state.lane.lines.customline;
  state.dirty = true;
  $("speed-dialog").close();
  emit("document");
  toast("区間速度を適用しました。Customlineを再生成してください");
});
let request = null;
function clearPreview() {
  state.registrationPreview = null;
  request = null;
  $("registration-preview-bar").hidden = true;
  emit("view");
}
action("registration-open", async () => {
  await save();
  clearPreview();
  const select = $("registration-source");
  select.replaceChildren();
  for (const map of state.maps.filter((map) => map.id !== state.id)) {
    const option = document.createElement("option");
    option.value = map.id;
    option.textContent = map.title;
    select.append(option);
  }
  if (!select.options.length)
    throw Error("位置合わせ元になる別の地図が必要です");
  $("registration-dialog").showModal();
});
$("registration-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const button = event.submitter;
  button.disabled = true;
  try {
    const data = Object.fromEntries(new FormData(event.target));
    const body = { id: state.id, revision: state.doc.revision, ...data };
    const result = await api("registration-preview", body);
    request = { ...body, token: result.token };
    state.registrationPreview = result.document;
    $("registration-dialog").close();
    $("registration-preview-bar").hidden = false;
    emit("view");
  } catch (e) {
    toast(e.message, true);
  } finally {
    button.disabled = false;
  }
});
action("registration-adjust", () => $("registration-dialog").showModal());
action("registration-cancel", clearPreview);
action("registration-apply", async () => {
  if (!request || state.dirty || request.id !== state.id)
    throw Error("編集内容が変わっています。再プレビューしてください");
  const id = state.id;
  await api("registration-apply", request);
  clearPreview();
  await loadMap(id, true);
  toast("HDMapとラインを位置合わせしました。置換前の編集状態も保存しました");
});
on("document", () => {
  if (!state.registrationPreview || state.dirty) clearPreview();
});
