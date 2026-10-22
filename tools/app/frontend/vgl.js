import { $, state, api, action, emit, on } from "./api.js";
let target = null;
let catalog = null;
let submitting = false;

function option(value, label) {
  const item = document.createElement("option");
  item.value = value;
  item.textContent = label;
  return item;
}
function checkSelection() {
  const path = $("vgl-model").value.trim();
  const width = Number($("vgl-width").value);
  const height = Number($("vgl-height").value);
  const model = catalog?.model_options.find((m) => m.path === path);
  let error = catalog?.error || "";
  if (!path) error ||= "使用するモデルを選択してください。";
  if (![width, height].every((v) => Number.isInteger(v) && v >= 16 && v <= 8192))
    error ||= "画像幅・高さは16〜8192の整数で指定してください。";
  if (model?.error) error ||= model.error;
  if (model?.width && (width !== model.width || height !== model.height))
    error ||= "選択したモデルと画像幅・高さが一致しません。";
  $("vgl-selection").textContent = error ||
    `モデル: ${path}\n画像: ${width} × ${height}` +
    (model?.width ? "（manifest・ONNXハッシュ照合済み）" : "（解像度未確認・生成開始時に検査）");
  $("vgl-start").disabled = submitting || !!error || !$("vgl-name").value.trim();
  return !$("vgl-start").disabled;
}
function updateModel() {
  const model = catalog.model_options.find((m) => m.path === $("vgl-model").value.trim());
  if (model?.width) {
    $("vgl-width").value = model.width;
    $("vgl-height").value = model.height;
  }
  for (const id of ["vgl-width", "vgl-height"]) $(id).readOnly = !!model?.width;
  checkSelection();
}
function selectModel() {
  const custom = $("vgl-model-choice").value === "custom";
  $("vgl-model").readOnly = !custom;
  $("vgl-model").value = custom ? "" : $("vgl-model-choice").value;
  $("vgl-width").value = "";
  $("vgl-height").value = "";
  updateModel();
}
$("vgl-model-choice").addEventListener("change", selectModel);
$("vgl-model").addEventListener("input", updateModel);
for (const id of ["vgl-name", "vgl-width", "vgl-height"])
  $(id).addEventListener("input", checkSelection);
on("status", () => { if (catalog) checkSelection(); });

action("vgl-open", async () => {
  const id = state.id;
  const data = await api("vgl?id=" + encodeURIComponent(id));
  if (state.id !== id) return;
  target = id;
  catalog = data;
  $("vgl-name").value = data.name;
  const models = data.model_options.filter((m) => !m.error).sort((a, b) =>
    (a.width || Infinity) - (b.width || Infinity) ||
    (a.height || Infinity) - (b.height || Infinity) || a.path.localeCompare(b.path));
  $("vgl-model-choice").replaceChildren(option("", "モデルを選択してください"));
  for (const model of models) {
    const size = model.width ? `${model.width} × ${model.height}` : "解像度不明・手動指定";
    $("vgl-model-choice").append(option(model.path, `${size} · ${model.path}`));
  }
  $("vgl-model-choice").append(option("custom", "その他・パスを手動指定"));
  const initial = models.find((m) => m.width === data.defaults.width && m.height === data.defaults.height);
  $("vgl-model-choice").value = initial?.path || "";
  $("vgl-info").textContent = data.error || `元地図: ${target} · 生成済みbundle: ${data.bundles.length}件`;
  selectModel();
  $("vgl-dialog").showModal();
});
action("vgl-start", async () => {
  if (!target || target !== state.id) throw Error("対象地図を選び直してください");
  // action() disables its button before calling this handler; recalculate validity.
  if (!checkSelection()) throw Error("モデルパスと画像サイズを確認してください");
  submitting = true;
  checkSelection();
  try {
    const path = $("vgl-model").value.trim();
    const model = catalog.model_options.find((m) => m.path === path);
    const size = model?.width ? {} : {width: Number($("vgl-width").value), height: Number($("vgl-height").value)};
    const job = await api("vgl", {id: target, name: $("vgl-name").value, model_dir: path, ...size});
    $("vgl-dialog").close();
    emit("job-started", job);
  } finally {
    submitting = false;
    checkSelection();
  }
});
