import { $, state, api, action, toast, emit, on } from "./api.js";
import { refreshMaps, loadMap } from "./maps.js";
const open = (id) => $(id).showModal();
for (const button of document.querySelectorAll("[data-close]"))
  button.addEventListener("click", () => $(button.dataset.close).close());
function field(parent, labelText, name, value, type = "text", extra = {}) {
  const label = document.createElement("label");
  label.textContent = labelText;
  const input = document.createElement("input");
  Object.assign(input, { name, value, type, ...extra });
  label.append(input);
  parent.append(label);
}
function form(id, submit) {
  $(id).addEventListener("submit", async (event) => {
    event.preventDefault();
    const button = $(id).querySelector("button.primary");
    button.disabled = true;
    try {
      await submit(Object.fromEntries(new FormData($(id))));
    } catch (e) {
      toast(e.message, true);
    } finally {
      button.disabled = false;
    }
  });
}
function fillConnection() {
  for (const [k, v] of Object.entries(state.config.connection))
    $("connection-form").elements[k].value = v;
  const preset = $("connection-preset");
  preset.replaceChildren(new Option("カスタム", ""));
  for (const row of state.config.jetson_hosts.presets)
    preset.add(new Option(row.label, row.host));
  preset.value = state.config.connection.host;
}
action("connection-open", () => {
  fillConnection();
  open("connection-dialog");
});
action("connection-reset", async () => {
  state.config.connection = await api("connection/reset", {});
  fillConnection();
  emit("connection");
  toast("接続設定を初期値に戻して保存しました");
});
$("connection-preset").addEventListener("change", () => {
  if ($("connection-preset").value)
    $("connection-form").elements.host.value = $("connection-preset").value;
});
$("connection-form").elements.host.addEventListener("input", () => {
  const host = $("connection-form").elements.host.value;
  $("connection-preset").value = state.config.jetson_hosts.presets.some(
    (p) => p.host === host,
  )
    ? host
    : "";
});
form("connection-form", async (data) => {
  state.config.connection = await api("connection", data);
  $("connection-dialog").close();
  emit("connection");
  toast("接続設定を保存しました");
});
action("import-open", () => open("import-dialog"));
form("import-form", async (data) => {
  await api("import", data);
  $("import-dialog").close();
  await refreshMaps();
  await loadMap(data.name);
  toast("点群を読み込みました");
});
export async function openBuild(selected = "") {
  state.records = await api("records");
  const select = $("build-form").elements.bag;
  select.replaceChildren();
  for (const r of state.records) {
    const o = document.createElement("option");
    o.value = r.id;
    o.textContent = r.id;
    select.append(o);
  }
  if (selected) select.value = selected;
  $("build-environment").textContent = state.config.environment.message;
  $("build-submit").disabled =
    !state.config.environment.ros || !state.records.length;
  $("build-detail-fields").replaceChildren();
  const defaults = state.config.environment.defaults;
  for (const [key, label] of Object.entries({
    left_image: "左画像",
    right_image: "右画像",
    left_info: "左CameraInfo",
    right_info: "右CameraInfo",
  }))
    field($("build-detail-fields"), label, key, defaults[key]);
  field($("build-detail-fields"), "基準frame", "base_frame", "camera_link");
  field(
    $("build-detail-fields"),
    "左光学frame",
    "left_frame",
    "camera_infra1_optical_frame",
  );
  field(
    $("build-detail-fields"),
    "右光学frame",
    "right_frame",
    "camera_infra2_optical_frame",
  );
  field(
    $("build-detail-fields"),
    "隔離ROS domain",
    "ros_domain_id",
    defaults.ros_domain_id,
    "number",
    { min: 1, max: 232 },
  );
  open("build-dialog");
}
action("build-open", () => openBuild());
form("build-form", async (data) => {
  const workflow = {};
  for (const key of ["left_image", "right_image", "left_info", "right_info"])
    workflow[key] = data[key];
  workflow.ros_domain_id = Number(data.ros_domain_id);
  workflow.dense_reconstruction = data.dense_reconstruction === "true";
  workflow.fs_model_res = data.fs_model_res;
  workflow.overrides = {
    tracking_mode: 0,
    base_frame: data.base_frame,
    camera_optical_frames: [data.left_frame, data.right_frame],
  };
  const job = await api("build", { name: data.name, bag: data.bag, workflow });
  $("build-dialog").close();
  emit("job-started", job);
});
action("line-settings-open", () => {
  $("line-setting-fields").replaceChildren();
  const labels = {
    curvature_limit: "最適化の曲率上限 (1/m)",
    spacing: "点間隔 (m)",
    vehicle_width: "車幅 (m)",
    vehicle_length: "車体全長 (m)",
    rear_axle_to_rear: "後輪軸から後端 (m)",
    margin: "片側の余裕 (m)",
    max_speed: "速度上限 (m/s)",
    lateral_accel: "横加速度上限 (m/s²)",
    accel: "加速上限 (m/s²)",
    decel: "減速上限 (m/s²)",
  };
  for (const [k, label] of Object.entries(labels))
    field($("line-setting-fields"), label, k, state.settings[k], "number", {
      min: k === "margin" ? 0 : 0.01,
      step: k === "rear_axle_to_rear" ? 0.0001 : 0.01,
      required: true,
    });
  open("line-settings-dialog");
});
form("line-settings-form", async (data) => {
  for (const [k, v] of Object.entries(data)) state.settings[k] = Number(v);
  $("line-settings-dialog").close();
  toast("次回の生成に設定を適用します");
});

let denseStages = null;
let denseSubmitting = false;
function renderDenseStages() {
  if (!denseStages || $("dense-dialog").dataset.map !== state.id) return;
  const running = state.jobs.find(j => j.status === "running" &&
    (j.title === "深度推定: " + state.id || j.title === "占有地図・メッシュ生成: " + state.id));
  for (const step of ["depth", "occupancy"]) {
    const done = step === "depth" ? denseStages.depth_ready : denseStages.dense_ready;
    const stale = step === "occupancy" && denseStages.dense_stale;
    const active = running?.title.startsWith(step === "depth" ? "深度推定:" : "占有地図・メッシュ生成:");
    const label = active ? (running.cancel_requested ? "停止要求中" : "実行中") : stale ? "旧深度で生成済み" : done ? "✓ 生成済み" : "未生成";
    $("dense-" + step + "-state").textContent = label;
    $("dense-" + step + "-card").dataset.status = active ? "running" : stale ? "stale" : done ? "done" : "pending";
    const button = $("dense-" + step);
    button.textContent = active ? "処理中…" : step === "depth"
      ? done ? "深度を再生成…" : "深度を生成"
      : done ? "占有地図・メッシュを再生成…" : "占有地図・メッシュを生成";
    button.classList.toggle("primary", !done && denseStages["can_" + step]);
    button.disabled = !!running || denseSubmitting || !denseStages["can_" + step] || !state.config?.environment.ros;
  }
  $("dense-resolution").disabled = !!running || denseSubmitting;
  $("mesh-color-start").disabled = !denseStages.dense_ready ||
    state.jobs.some(j => j.status === "running" && j.title.endsWith(": " + state.id));
}
async function openDense() {
  const selectedMap = state.id;
  const [data, stages] = await Promise.all([
    api("dense?id=" + encodeURIComponent(selectedMap)),
    api("dense-steps?id=" + encodeURIComponent(selectedMap)),
  ]);
  if (selectedMap !== state.id) return;
  $("dense-dialog").dataset.map = selectedMap;
  denseStages = stages;
  $("dense-step-status").textContent = stages.reason ||
    `深度: ${stages.depth_ready ? "生成済み" : "未生成"} / 占有地図・メッシュ: ${stages.dense_stale ? "旧深度の成果物（再生成可能）" : stages.dense_ready ? "生成済み" : "未生成"}`;
  $("dense-resolution").value = stages.resolution;
  renderDenseStages();
  if (!stages.reason && stages.depth_ready && !stages.dense_ready)
    $("dense-step-status").textContent = "深度推定は完了しています。次は「2. 占有地図・メッシュ」を実行してください。";
  $("dense-info").textContent = data.summary
    ? `${data.summary.vertices.toLocaleString()} 頂点 / ${data.summary.faces.toLocaleString()} 面 · 地面合わせ後の座標系`
    : "占有地図・メッシュは未生成です。以下から工程を順番に実行できます。";
  $("dense-files").replaceChildren();
  $("dense-preview").hidden = true;
  for (const file of data.files) {
    const row = document.createElement("p"), link = document.createElement("a");
    link.href = file.url;
    link.download = file.name;
    link.textContent = `${file.name} (${(file.bytes / 1048576).toFixed(1)} MiB)`;
    row.append(link);
    $("dense-files").append(row);
    if (file.name === "occupancy_map.png") {
      $("dense-preview").src = file.url;
      $("dense-preview").hidden = false;
    }
  }
  open("dense-dialog");
}
action("dense-open", openDense);
on("status", renderDenseStages);
on("job-finished", async e => {
  if ($("dense-dialog").open && e.detail.title.endsWith(": " + state.id)) await openDense();
});

for (const step of ["depth", "occupancy"])
  action("dense-" + step, async () => {
    if ($("dense-dialog").dataset.map !== state.id)
      throw Error("地図が切り替わりました。ダイアログを開き直してください");
    if (state.dirty) throw Error("HDMapの編集を保存してから実行してください");
    if (denseSubmitting) return;
    denseSubmitting = true;
    renderDenseStages();
    try {
    const id = state.id;
    const latest = await api("dense-steps?id=" + encodeURIComponent(id));
    if (state.id !== id) throw Error("地図が切り替わりました");
    const ready = step === "depth" ? latest.depth_ready : latest.dense_ready;
    if (ready && !confirm(step === "depth"
      ? `深度推定は既に完了しています。${$("dense-resolution").value}で再生成しますか？\n時間のかかる処理をもう一度実行します。占有地図・メッシュを作るだけならキャンセルして工程2へ進んでください。`
      : "占有地図・メッシュは既に生成されています。保存済みの深度からもう一度生成しますか？\n成功するまで現在の成果物は保持されます。")) return;
    const job = await api("dense-step", {
      id, revision: state.doc.revision, step,
      resolution: $("dense-resolution").value,
    });
    $("dense-dialog").close();
    emit("job-started", job);
    toast("追加生成を開始しました。完了後にこの画面を開くと次の工程へ進めます");
    } finally {
      denseSubmitting = false;
      renderDenseStages();
    }
  });

action("mesh-color-start", async () => {
  if ($("dense-dialog").dataset.map !== state.id || state.dirty)
    throw Error("地図を保存してダイアログを開き直してください");
  if (!confirm("元bagのRGB画像でmeshを着色します。既存の着色がある場合は再生成しますか？"))
    return;
  const job = await api("mesh-color", { id: state.id, revision: state.doc.revision,
    bag: $("mesh-color-bag").value.trim() });
  $("dense-dialog").close();
  emit("job-started", job);
  toast("RGB着色を開始しました。完了後に3D重ね表示を開いてください");
});
