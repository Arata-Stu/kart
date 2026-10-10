import { compatible, resizePairs, alignPairs } from "./paired.js";
import {
  $,
  state,
  on,
  emit,
  api,
  action,
  item,
  toast,
  remember,
  edited,
  undo,
  fmt,
} from "./api.js";
import { fit, draw } from "./canvas.js";
export async function refreshMaps() {
  state.maps = await api("maps");
  const list = $("map-list");
  list.replaceChildren();
  if (!state.maps.length) {
    const p = document.createElement("p");
    p.className = "empty";
    p.textContent =
      "まだ地図がありません。bagから作成するか、点群を読み込んでください。";
    list.append(p);
  }
  for (const m of state.maps)
    list.append(
      item(
        m.title,
        m.snapshot_status === "pending"
          ? "VSLAM作成済み · 点群未取得"
          : `${fmt(m.point_count)} 点 · ${m.lane_count} lanes`,
        () => loadMap(m.id),
        m.id === state.id,
      ),
    );
  emit("catalog");
}
export async function loadMap(id, preserve = false) {
  const previousStep = state.step,
    previousRange = [state.zmin, state.zmax];
  if (state.dirty && !confirm("未保存の編集を破棄して地図を開きますか？"))
    return;
  const [doc, cloud] = await Promise.all([
    api("map?id=" + encodeURIComponent(id)),
    api("cloud?id=" + encodeURIComponent(id)),
  ]);
  Object.assign(state, {
    id,
    doc,
    cloud,
    registrationPreview: null,
    dirty: false,
    undo: [],
    redo: [],
    selected: -1,
  });
  state.laneId = doc.lanes.some((lane) => lane.id === state.laneId)
    ? state.laneId
    : doc.lanes[0].id;
  Object.assign(state.settings, state.lane.lines.raceline?.settings || {});
  $("raceline-method").value = state.settings.optimizer;
  let min = Infinity,
    max = -Infinity;
  for (const p of cloud.points) {
    min = Math.min(min, p[2]);
    max = Math.max(max, p[2]);
  }
  if (!cloud.points.length) {
    min = 0;
    max = 1;
  }
  state.zmin = Math.floor(min * 100) / 100;
  state.zmax = Math.ceil(max * 100) / 100;
  for (const id of ["z-min", "z-max"]) {
    $(id).min = state.zmin;
    $(id).max = Math.max(state.zmin + 0.01, state.zmax);
  }
  $("z-min").value = state.zmin;
  $("z-max").value = state.zmax;
  const tf = cloud.provenance.applied_transform;
  $("tf-note").textContent =
    cloud.provenance.warning ||
    (tf
      ? `終了時TFを適用: ${tf.parent} ← ${tf.child}\n採用時刻 ${tf.stamp_ns} ns`
      : "map座標の点群です。TFの二重変換は行いません。");
  if (preserve) {
    [state.zmin, state.zmax] = previousRange;
    $("z-min").value = state.zmin;
    $("z-max").value = state.zmax;
    step(previousStep);
    emit("view");
  } else {
    step("cloud");
    emit("loaded");
  }
  emit("document");
  await refreshMaps();
}
export async function save() {
  if (!state.doc || !state.dirty) return;
  state.doc = await api("save", { id: state.id, document: state.doc });
  state.dirty = false;
  emit("document");
  await refreshMaps();
}
function step(value) {
  state.step = value;
  state.selected = -1;
  if (value === "bounds" && state.target === "custom") state.target = "left";
  for (const s of ["cloud", "bounds", "lines"]) {
    $("panel-" + s).hidden = s !== value;
    $("step-" + s).classList.toggle("active", s === value);
  }
  emit("document");
}
function status() {
  const pending = state.doc?.snapshot_status === "pending";
  const d = pending ? null : state.lane;
  $("capture-open").hidden = !pending;
  for (const id of ["step-bounds", "step-lines"])
    $(id).disabled = !state.doc || pending;
  const busy = state.jobs.some(
    (j) =>
      j.status === "running" &&
      (j.title.includes(state.id) || j.title.includes("生成")),
  );
  $("map-title").textContent = state.doc?.title || "地図を選択";
  $("save-state").textContent = d
    ? state.dirty
      ? "● 未保存"
      : `保存済み · r${state.doc.revision}`
    : "";
  $("empty-canvas").hidden = !!state.doc;
  $("save").disabled = !d || !state.dirty || busy;
  $("lane-reverse").disabled = !d || busy;
  $("simulation-open").disabled = !d || busy || !Object.keys(d.lines).length;
  $("delete-map").disabled = !state.doc || busy;
  $("hdmap-reset").disabled = !d || busy;
  $("copy-map").disabled = !state.doc || busy;
  $("vgl-open").disabled =
    !state.doc ||
    busy ||
    !state.config?.environment.ros ||
    !state.maps.find((m) => m.id === state.id)?.has_vslam;
  $("capture-open").disabled =
    !pending || busy || !state.config?.environment.ros;
  $("undo").disabled = !state.undo.length;
  $("redo").disabled = !state.redo.length;
  $("closed").checked = d?.closed ?? true;
  $("boundary-count").textContent =
    `左 ${d?.left.length || 0} / 右 ${d?.right.length || 0}`;
  for (const k of ["centerline", "raceline", "customline"]) {
    $(k + "-status").textContent = d?.lines[k]
      ? `${d.lines[k].points.length} 点`
      : "未生成";
    $("generate-" + k).disabled = !d || busy;
  }
  $("edit-pair").classList.toggle("active", state.target === "pair");
  $("edit-left").classList.toggle("active", state.target === "left");
  $("edit-right").classList.toggle("active", state.target === "right");
  $("edit-custom").classList.toggle("active", state.target === "custom");
  $("z-min-value").textContent = Number.isFinite(state.zmin)
    ? state.zmin.toFixed(2) + " m"
    : "—";
  $("z-max-value").textContent = Number.isFinite(state.zmax)
    ? state.zmax.toFixed(2) + " m"
    : "—";
  $("canvas-hint").textContent =
    state.step === "cloud" ||
    (state.step === "lines" && state.target !== "custom")
      ? "ホイール: 拡大 · 右ドラッグ: 移動"
      : `編集: ${state.target === "obstacle" ? "禁止領域" : state.target === "pair" ? "左右セット" : state.target === "left" ? "左境界" : state.target === "right" ? "右境界" : "Customline"} · クリック追加 / 点ドラッグ / Delete削除`;
  for (const id of [
    "registration-open",
    "custom-speeds-open",
    "raceline-method",
    "delete-point",
    "clear-line",
    "edit-custom",
    "edit-pair",
    "pair-resize",
    "pair-align",
    "pair-width",
    "edit-left",
    "edit-right",
    "closed",
    "export",
    "z-min",
    "z-max",
    "z-reset",
  ])
    $(id).disabled = !d;
}
for (const s of ["cloud", "bounds", "lines"])
  action("step-" + s, () => step(s));
action("bounds-next", () => step("lines"));
for (const t of ["left", "right", "custom", "pair"])
  action("edit-" + t, () => {
    if (t === "pair" && !compatible(state.lane)) {
      if (
        !confirm(
          "左右を同じ点数へ再配置してセット編集します。境界形状が変わる場合があります。Undoで戻せます。続けますか？",
        )
      )
        return;
      const candidate = structuredClone(state.lane);
      alignPairs(candidate);
      remember();
      state.lane.left = candidate.left;
      state.lane.right = candidate.right;
      state.lane.lines = {};
      state.dirty = true;
    }
    state.target = t;
    state.selected = -1;
    emit("document");
  });
action("undo", () => undo());
action("redo", () => undo(true));
action("fit", fit);
action("save", save);
action("refresh-maps", refreshMaps);
action("delete-point", () => {
  if (state.selected < 0) return;
  remember();
  if (state.target === "obstacle")
    state.obstacle?.polygon.splice(state.selected, 1);
  else
    for (const side of state.target === "pair"
      ? ["left", "right"]
      : [state.target])
      state.lane[side].splice(state.selected, 1);
  state.selected = -1;
  edited();
});
action("clear-line", () => {
  if (!confirm("選択中の境界をクリアしますか？")) return;
  remember();
  if (state.target === "obstacle") {
    if (state.obstacle) state.obstacle.polygon = [];
  } else
    for (const side of state.target === "pair"
      ? ["left", "right"]
      : [state.target])
      state.lane[side] = [];
  state.selected = -1;
  edited();
});
$("closed").addEventListener("change", () => {
  remember();
  state.lane.closed = $("closed").checked;
  state.lane.lines = {};
  state.dirty = true;
  emit("document");
});
for (const id of ["z-min", "z-max"])
  $(id).addEventListener("input", () => {
    let lo = Number($("z-min").value),
      hi = Number($("z-max").value);
    if (lo > hi) {
      if (id === "z-min") hi = lo;
      else lo = hi;
    }
    Object.assign(state, { zmin: lo, zmax: hi });
    $("z-min").value = lo;
    $("z-max").value = hi;
    status();
    emit("view");
  });
action("z-reset", () => {
  state.zmin = Number($("z-min").min);
  state.zmax = Number($("z-max").max);
  $("z-min").value = state.zmin;
  $("z-max").value = state.zmax;
  status();
  emit("view");
});
$("raceline-method").addEventListener("change", () => {
  state.settings.optimizer = $("raceline-method").value;
});
$("show-path").addEventListener("change", () => {
  state.showPath = $("show-path").checked;
  draw();
});
for (const kind of ["centerline", "raceline", "customline"])
  action("generate-" + kind, async () => {
    const laneId = state.laneId,
      mapId = state.id,
      settings = structuredClone(state.settings);
    await save();
    if (state.id !== mapId)
      throw Error("地図が切り替わりました。生成し直してください");
    const job = await api("generate", {
      id: state.id,
      kind,
      revision: state.doc.revision,
      settings,
      lane_id: laneId,
    });
    emit("job-started", job);
    toast(kind + "の生成を開始しました");
  });
action("export", async () => {
  await save();
  const r = await api("export", { id: state.id });
  toast("書き出しました: " + r.path);
});
on("status", status);
on("document", status);
on("job-finished", async (e) => {
  const job = e.detail;
  await refreshMaps();
  if (job.status === "succeeded" && job.result?.map) {
    if (
      !state.dirty &&
      (state.id === job.result.map ||
        state.id === job.result.copied_from ||
        !state.id)
    )
      await loadMap(
        job.result.map,
        state.id === job.result.map && state.doc?.snapshot_status !== "pending",
      );
    else
      toast(
        "地図処理が完了しました。未保存の編集を保護しています。一覧から地図を開き直してください。",
      );
  }
});
window.addEventListener("beforeunload", (e) => {
  if (state.dirty) {
    e.preventDefault();
    e.returnValue = "";
  }
});

let pendingDelete = null;
action("delete-map", () => {
  if (!state.doc) return;
  pendingDelete = { id: state.id, revision: state.doc.revision };
  $("delete-map-description").textContent =
    `「${state.doc.title}」をごみ箱へ移動します。` +
    (state.dirty
      ? "未保存の編集は破棄されます。"
      : "復元は「ごみ箱・復元」から行えます。");
  $("delete-map-dialog").showModal();
});
action("delete-map-confirm", async () => {
  if (!pendingDelete || state.id !== pendingDelete.id)
    throw Error("対象地図が変わっています。開き直してください");
  const { id, revision } = pendingDelete;
  await api("delete-map", { id, revision });
  $("delete-map-dialog").close();
  pendingDelete = null;
  Object.assign(state, {
    id: null,
    doc: null,
    cloud: null,
    dirty: false,
    undo: [],
    redo: [],
    selected: -1,
    registrationPreview: null,
  });
  emit("document");
  emit("view");
  await refreshMaps();
  toast("地図をごみ箱へ移動しました。「ごみ箱・復元」から戻せます");
});
async function refreshTrash() {
  const entries = await api("trash");
  const list = $("trash-list");
  list.replaceChildren();
  if (!entries.length) list.textContent = "ごみ箱は空です";
  for (const entry of entries)
    list.append(
      item(entry.title, `r${entry.revision} · クリックして復元`, async () => {
        await api("restore-map", { token: entry.token });
        await refreshTrash();
        await refreshMaps();
        toast("地図を復元しました");
      }),
    );
}
action("trash-open", async () => {
  await refreshTrash();
  $("trash-dialog").showModal();
});

action("pair-resize", () => {
  if (state.target !== "pair") throw Error("セット編集を選択してください");
  const width = Number($("pair-width").value);
  if (!Number.isFinite(width) || width < 0.05 || width > 100)
    throw Error("幅は0.05〜100 mです");
  const candidate = structuredClone(state.lane);
  resizePairs(candidate, width, state.selected);
  remember();
  state.lane.left = candidate.left;
  state.lane.right = candidate.right;
  edited();
});

action("pair-reshape", () => {
  if (state.target !== "pair" || !compatible(state.lane))
    throw Error("左右対応したセット編集を選択してください");
  const width = Number($("pair-width").value);
  if (!Number.isFinite(width) || width < 0.05 || width > 100)
    throw Error("幅は0.05〜100 mです");
  const candidate = structuredClone(state.lane);
  resizePairs(candidate, width, -1);
  remember();
  state.lane.left = candidate.left;
  state.lane.right = candidate.right;
  edited();
});

action("pair-align", () => {
  if (
    !confirm(
      "左右の点を等間隔に再配置し、方向と始点を合わせます。形状が変わる場合があります。続けますか？",
    )
  )
    return;
  const candidate = structuredClone(state.lane);
  alignPairs(candidate);
  remember();
  state.lane.left = candidate.left;
  state.lane.right = candidate.right;
  state.target = "pair";
  state.selected = -1;
  edited();
});
