import { showLoss } from "./loss_curve.js";
import { installBagPicker } from "./bag_picker.js";
import { showFeatures } from "./features.js";
import { trashButton, installTrash } from "./learning_trash.js";
import { showEvaluation, showEngine } from "./evaluation.js";
import { $, state, api, action, on, emit, item, toast } from "./api.js";
import { nextVersion, datasetOption, datasetName, artifactOption } from "./learning_names.js";
const refreshBagPicker = installBagPicker();
let catalog = null;
let bags = [];
let step = "dataset";
const kinds = {
  dataset: "datasets",
  train: "runs",
  export: "runs",
  push: "models",
  evaluate: "evaluations",
  features: "features",
};
const labels = {
  datasets: "データセット",
  runs: "学習済み実験",
  models: "ONNXモデル",
  evaluations: "オフライン評価",
  features: "特徴量の確認結果",
};
const fields = [
  "python",
  "inference_python",
  "evs_raw_python",
  "encoder_repo",
  "weights",
  "evs_weights",
  "evs_json",
  "device",
  "model_root",
];
function suggestNames() {
  if (!catalog) return;
  const input = $("e2e-dataset-name");
  const suggestion = datasetName(bags.find(row => row.id === $("e2e-bag").value));
  if (!input.value || input.value === input.dataset.suggestion) {
    input.value = suggestion;
    input.dataset.suggestion = suggestion;
  }
  const base = ($("e2e-train-modality").value === "evs" ? "evs20" : $("e2e-architecture").value === "dinov3_vitt16" ? "dinov3-tiny" : "dinov3") + ($("e2e-mode").value === "steer_only" ? "-steer" : "-control");
  for (const [id, prefix, kind] of [
    ["e2e-distill-name", "dinov3-tiny-encoder", "encoders"],
    ["e2e-run-name", base + ($("e2e-finetune").checked ? "-finetune" : ""), "runs"],
    ["e2e-features-name", ($("e2e-features-run").value || base) + "-features", "features"],
    ["e2e-model-name", $("e2e-export-run").value || base, "models"],
    [
      "e2e-eval-name",
      ($("e2e-eval-model").value || base) + "-eval",
      "evaluations",
    ],
  ]) {
    const input = $(id);
    if (!input.value || input.value === input.dataset.suggestion) {
      input.value = nextVersion(
        prefix,
        catalog[kind].map((row) => row.id),
      );
      input.dataset.suggestion = input.value;
    }
  }
}
function options(id, rows, title) {
  const select = $(id),
    previous = new Set([...select.selectedOptions].map((o) => o.value));
  select.replaceChildren();
  if (!select.multiple) select.add(new Option(title, ""));
  const groups = new Map();
  for (const row of rows) {
    let parent = select;
    if (row.group) {
      if (!groups.has(row.group)) {
        const group = document.createElement("optgroup");
        group.label = row.group;
        select.append(group);
        groups.set(row.group, group);
      }
      parent = groups.get(row.group);
    }
    const option = new Option(row.label || row.id, row.id);
    option.selected = previous.has(row.id);
    option.title = row.id;
    parent.append(option);
  }
}
function render() {
  if (!catalog) return;
  for (const kind of ["datasets", "runs", "models", "evaluations"])
    $("e2e-count-" + kind).textContent = catalog[kind].length;
  $("e2e-device").textContent = "学習: " + catalog.settings.device;
  $("e2e-storage").textContent = catalog.root;
  $("e2e-model-root").textContent = catalog.settings.model_root;
  const connection = state.config?.connection;
  $("e2e-remote").textContent = connection?.host
    ? `${connection.user}@${connection.host}:${connection.port}`
    : "Jetson接続未設定";
  const kind = kinds[step];
  $("e2e-library-title").textContent = labels[kind];
  const list = $("e2e-library");
  list.replaceChildren();
  if (!catalog[kind].length) {
    const p = document.createElement("p");
    p.className = "empty";
    p.textContent =
      kind === "datasets"
        ? "最初のデータセットを作成してください。"
        : kind === "runs"
          ? "学習が完了すると、実験と最終epochの指標が表示されます。"
          : kind === "features" ? "可視化が完了すると、ここから比較画像を開けます。" : "ONNXの変換・検証が完了すると表示されます。";
    list.append(p);
  }
  for (const row of catalog[kind]) {
    let detail =
      kind === "datasets"
        ? `${row.id.includes("/") ? row.id.split("/").slice(0, -1).join(" ") + " · " : ""}${row.samples.toLocaleString()} samples · ${row.bag.split("/").pop()}`
        : kind === "runs"
          ? `${row.finetune ? "全体fine-tuning" : "head学習"} · ${row.mode} · epoch ${row.metrics?.epoch ?? "—"} · val MSE ${row.metrics?.validation_mse?.toPrecision(4) ?? "—"}`
          : `${row.mode} · ORT最大誤差 ${Number(row.ort_max_abs_error).toExponential(2)}`;
    if (kind === "evaluations")
      detail = `${row.samples} frames · ${row.model} · steer MAE ${row.metrics.steering_command.mae.toFixed(4)}`;
    if (kind === "features") detail = `${row.frame_count} frames · ${row.run} · ${row.dataset}`;
    const entry = document.createElement("div");
    entry.className = "artifact-row";
    entry.append(
      item((row.modality === "evs" ? "EVS · " : "") + (row.name || row.id), (kind !== "datasets" && row.id.includes("/") ? row.id.split("/").slice(0,-1).join(" ") + " · " : "") + detail, () => {
        if (kind === "features") return showFeatures(row.id);
        if (kind === "evaluations") return showEvaluation(row.id);
        if (step === "dataset") {
          $("e2e-train-modality").value = row.modality || "rgb";
          $("e2e-train-modality").dispatchEvent(new Event("change"));
          setStep("train");
          $("e2e-train-data").value = row.id;
        } else if (kind === "runs") {
          if (step === "train") return showLoss(row.id);
          setStep("export");
          $("e2e-export-run").value = row.id;
          suggestNames();
        } else $("e2e-push-model").value = row.id;
      }),
      trashButton(kind, row, refreshLearning),
    );
    list.append(entry);
  }
}
function setStep(next) {
  step = next;
  for (const name of Object.keys(kinds)) {
    $("e2e-pane-" + name).hidden = name !== step;
    $("e2e-tab-" + name).classList.toggle("active", name === step);
  }
  render();
}
export async function refreshLearning() {
  const [data, records] = await Promise.all([api("e2e"), api("records")]);
  catalog = data;
  bags = records;
  refreshBagPicker(records);
  for (const id of ["e2e-train-data", "e2e-validation-data"])
    options(
      id,
      data.datasets.filter(row => (row.modality || "rgb") === $("e2e-train-modality").value).map(datasetOption),
      "",
    );
  options("e2e-eval-data", data.datasets.filter(row => row.modality === "evs").map(datasetOption), "EVSデータセットを選択");
  options("e2e-export-run", data.runs, "実験を選択");
  options("e2e-tiny-encoder", data.encoders.map(artifactOption), "蒸留済みTiny encoderを選択");
  options("e2e-distill-teacher", data.runs.filter(row => row.modality !== "evs" && (!row.architecture || row.architecture === 'dinov3_vits16')), "学習環境の公式ViT-S重み");
  options("e2e-features-run", data.runs.filter(row => row.modality !== "evs"), "学習済み実験を選択");
  options("e2e-features-data", data.datasets.filter(row => row.modality !== "evs").map(datasetOption), "RGBデータセットを選択");
  options("e2e-push-model", data.models.map(artifactOption), "モデルを選択");
  options("e2e-eval-model", data.models.map(artifactOption), "モデルを選択");
  options("e2e-engine-history", data.engines, "Notebook engineの結果を選択");
  updateEvalInput();
  suggestNames();
  render();
}
$("e2e-bag").addEventListener("change", suggestNames);
function updateEvalInput() {
  const evs = catalog?.models.find(row => row.id === $("e2e-eval-model").value)?.modality === "evs";
  $("e2e-eval-data-label").hidden = !evs;
  $("e2e-eval-bag-label").hidden = evs;
}
$("e2e-eval-model").addEventListener("change", () => { updateEvalInput(); suggestNames(); });
$("e2e-mode").addEventListener("change", suggestNames);
$("e2e-export-run").addEventListener("change", suggestNames);
function selected(id) {
  return [...$(id).selectedOptions].map((o) => o.value);
}
async function start(operation, body) {
  const job = await api("e2e/" + operation, body);
  emit("job-started", job);
  $("jobs-drawer").hidden = false;
  toast(job.title + "を開始しました");
}
for (const name of Object.keys(kinds))
  action("e2e-tab-" + name, () => setStep(name));
action("e2e-refresh", refreshLearning);
action("e2e-create-dataset", () =>
  start("dataset", {
    name: $("e2e-dataset-name").value,
    bag: $("e2e-bag").value,
    modality: $("e2e-modality").value,
    raw_manifest: $("e2e-raw-manifest").value,
    sample_stride: Number($("e2e-sample-stride").value),
    max_reorder_us: Number($("e2e-max-reorder").value),
    clock: $("e2e-clock").value,
    max_skew_ms: Number($("e2e-skew").value),
    image_topic: $("e2e-image-topic").value,
    command_topic: $("e2e-command-topic").value,
    mode_topic: $("e2e-mode-topic").value,
  }),
);
action("e2e-start-train", () =>
  start("train", {
    name: $("e2e-run-name").value,
    train: selected("e2e-train-data"),
    validation: selected("e2e-validation-data"),
    mode: $("e2e-mode").value,
    fixed_throttle: Number($("e2e-fixed-throttle").value),
    max_throttle: Number($("e2e-max-throttle").value),
    epochs: Number($("e2e-epochs").value),
    batch_size: Number($("e2e-batch").value),
    learning_rate: Number($("e2e-lr").value),
    modality: $("e2e-train-modality").value,
    architecture: $("e2e-train-modality").value === "evs" ? "dinov3_vits16" : $("e2e-architecture").value,
    encoder: $("e2e-tiny-encoder").value,
    finetune: $("e2e-finetune").checked,
    feature_cache: $("e2e-feature-cache").checked,
    num_workers: Number($("e2e-workers").value),
  }),
);
action("e2e-start-export", () =>
  start("export", {
    name: $("e2e-model-name").value,
    run: $("e2e-export-run").value,
  }),
);
action("e2e-start-evaluate", () =>
  start("evaluate", {
    name: $("e2e-eval-name").value,
    model: $("e2e-eval-model").value,
    dataset: $("e2e-eval-data").value,
    bag: $("e2e-eval-bag").value,
    image_topic: $("e2e-image-topic").value,
    command_topic: $("e2e-command-topic").value,
    mode_topic: $("e2e-mode-topic").value,
    clock: $("e2e-clock").value,
    max_skew_ms: Number($("e2e-skew").value),
  }),
);
action("e2e-start-push", async () => {
  if (!state.config.connection.host || !state.config.connection.user) {
    $("connection-open").click();
    return;
  }
  await start("push", {
    id: $("e2e-push-model").value,
    connection: state.config.connection,
    model_root: catalog.settings.model_root,
  });
});
action("e2e-connection", () => $("connection-open").click());
action("e2e-settings-open", async () => {
  if (!catalog) await refreshLearning();
  for (const key of fields)
    $("e2e-setting-" + key).value = catalog.settings[key];
  $("e2e-settings-dialog").showModal();
});
action("e2e-settings-close", () => $("e2e-settings-dialog").close());
action("e2e-settings-save", async () => {
  const values = Object.fromEntries(
    fields.map((k) => [k, $("e2e-setting-" + k).value]),
  );
  await api("e2e/settings", values);
  $("e2e-settings-dialog").close();
  await refreshLearning();
  toast("学習環境を保存しました");
});
on("job-finished", async ({detail:job}) => {
  if (catalog) await refreshLearning();
  if (job.status === 'succeeded' && job.result?.kind === 'encoders') $("e2e-tiny-encoder").value = job.result.id;
  if (job.status === 'succeeded' && ['runs','encoders'].includes(job.result?.kind) &&
      step === 'train' && !$("learning-page").hidden && !document.querySelector('dialog[open]'))
    await showLoss(job.result.id, job.result.kind);
});
on("connection", render);

action("e2e-build-engine", () =>
  start("build-engine", {
    model: $("e2e-eval-model").value,
    name: nextVersion(
      ($("e2e-eval-model").value || "model").slice(0, 42) + "-notebook-trt",
      catalog.engines.map((r) => r.id),
    ),
  }),
);
$("e2e-engine-history").addEventListener("change", () =>
  showEngine($("e2e-engine-history").value).catch((e) =>
    toast(e.message, true),
  ),
);

installTrash(refreshLearning);

$("e2e-features-run").addEventListener("change", suggestNames);
action("e2e-start-features", () => start("features", {
  name: $("e2e-features-name").value, run: $("e2e-features-run").value,
  dataset: $("e2e-features-data").value, max_frames: Number($("e2e-features-max").value),
}));

$("e2e-finetune").addEventListener("change", () => {
  const finetune = $("e2e-finetune").checked;
  $("e2e-feature-cache").disabled = finetune;
  $("e2e-cache-help").textContent = finetune
    ? '全体fine-tuningではbackboneが変わるため、特徴量キャッシュを使用しません。'
    : '初回に特徴量を保存し、同じ入力・重み・前処理なら再利用します。';
  const lr = $("e2e-lr");
  if (Number(lr.value) === (finetune ? 1e-4 : 1e-5))
    lr.value = finetune ? '0.00001' : '0.0001';
  $("e2e-training-scope").textContent = finetune
    ? 'backboneを凍結せず、headと一緒に全体を更新。既定学習率 1e-5。学習率は手動変更できます。'
    : 'backboneを凍結してheadのみ学習。既定学習率 1e-4。';
  suggestNames();
});

$("e2e-architecture").addEventListener("change", () => {
  $("e2e-tiny-options").hidden = $("e2e-architecture").value !== 'dinov3_vitt16';
  suggestNames();
});
action('e2e-start-distill', () => start('distill', {
  name: $('e2e-distill-name').value, architecture:'dinov3_vitt16',
  train:selected('e2e-train-data'), validation:selected('e2e-validation-data'),
  teacher_run:$('e2e-distill-teacher').value,
  epochs:Number($('e2e-distill-epochs').value), learning_rate:Number($('e2e-distill-lr').value),
  batch_size:Number($('e2e-batch').value), num_workers:Number($('e2e-workers').value),
  feature_cache:$('e2e-feature-cache').checked,
}));
action('e2e-distill-curve', () => {
  if (!$('e2e-tiny-encoder').value) throw new Error('Tiny encoderを選択してください');
  return showLoss($('e2e-tiny-encoder').value,'encoders');
});

$("e2e-modality").addEventListener("change", () => {
  $("e2e-evs-options").hidden = $("e2e-modality").value !== "evs";
  if ($("e2e-modality").value === "evs") $("e2e-clock").value = "header";
});
$("e2e-train-modality").addEventListener("change", () => {
  const evs = $("e2e-train-modality").value === "evs";
  $("e2e-rgb-model").hidden = evs;
  $("e2e-tiny-options").hidden = evs || $("e2e-architecture").value !== "dinov3_vitt16";
  for (const id of ["e2e-train-data", "e2e-validation-data"])
    options(id, catalog.datasets.filter(row => (row.modality || "rgb") === (evs ? "evs" : "rgb")).map(datasetOption), "");
  suggestNames();
});
