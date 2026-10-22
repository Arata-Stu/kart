import { $, state, api, action, emit, on } from "./api.js";
let pending = null;
let bundleOptions = [];
let submitting = false;
function option(value, label = value) {
  const item = document.createElement("option");
  item.value = value;
  item.textContent = label;
  return item;
}
function selection() {
  return bundleOptions.find((b) => b.path === $("capture-vgl").value);
}
function checkSelection() {
  const bundle = selection();
  const path = $("capture-model").value;
  const valid = !!bundle && !bundle.error && bundle.models.includes(path) && !!$("capture-bag").value;
  const shape = bundle?.input_shape;
  $("capture-info").textContent = !bundle
    ? "先にこの地図の「VGL地図生成」を実行してください。"
    : bundle.error || (valid
      ? `画像: ${shape[3]} × ${shape[2]} · 対応モデルを初期選択（${bundle.models.length}候補）\nモデル: ${path}\nONNX・地図の整合性照合済み。GPU互換性は実行時に検査します。`
      : "地図生成に使ったbagと対応モデルを選択してください。");
  $("capture-start").disabled = submitting || !valid;
  return valid;
}
function selectBundle() {
  const bundle = selection();
  const previous = $("capture-model").value;
  $("capture-model").replaceChildren();
  for (const path of bundle?.models || []) $("capture-model").append(option(path));
  if (!bundle?.models.length) $("capture-model").append(option("", "対応モデルなし"));
  $("capture-model").value = bundle?.models.includes(previous) ? previous : bundle?.default_model || "";
  checkSelection();
}
$("capture-vgl").addEventListener("change", selectBundle);
$("capture-model").addEventListener("change", checkSelection);
$("capture-bag").addEventListener("change", checkSelection);
on("status", () => { if (pending) checkSelection(); });
action("capture-open", async () => {
  if (state.doc?.snapshot_status !== "pending") return;
  const target = {id: state.id, revision: state.doc.revision};
  const [records, vgl] = await Promise.all([
    api("records"), api("vgl?id=" + encodeURIComponent(target.id)),
  ]);
  if (state.id !== target.id || state.doc.revision !== target.revision) return;
  pending = target;
  bundleOptions = vgl.bundle_options;
  $("capture-vgl").replaceChildren();
  for (const bundle of bundleOptions) $("capture-vgl").append(option(bundle.path));
  $("capture-bag").replaceChildren();
  for (const record of records) $("capture-bag").append(option(record.id));
  selectBundle();
  $("capture-dialog").showModal();
});
action("capture-start", async () => {
  if (!pending || pending.id !== state.id || pending.revision !== state.doc.revision)
    throw Error("対象地図を開き直してください");
  if (!checkSelection()) throw Error("対応するVGL地図・モデルを確認してください");
  submitting = true;
  checkSelection();
  try {
    const job = await api("capture", { ...pending, bag: $("capture-bag").value,
      vgl_bundle: $("capture-vgl").value, model_dir: $("capture-model").value });
    $("capture-dialog").close();
    emit("job-started", job);
  } finally {
    submitting = false;
    checkSelection();
  }
});
