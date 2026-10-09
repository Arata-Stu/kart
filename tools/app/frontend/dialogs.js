import { $, state, api, action, toast, emit } from "./api.js";
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
action("connection-open", () => {
  for (const [k, v] of Object.entries(state.config.connection))
    $("connection-form").elements[k].value = v;
  const preset = $("connection-preset");
  preset.replaceChildren(new Option("カスタム", ""));
  for (const row of state.config.jetson_hosts.presets)
    preset.add(new Option(row.label, row.host));
  preset.value = state.config.connection.host;
  open("connection-dialog");
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
