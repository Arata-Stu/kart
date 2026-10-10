import { $, state, api, action, on, emit, item, toast } from "./api.js";
import { nextVersion } from "./learning_names.js";
let catalog = null;
let step = "dataset";
const kinds = {
  dataset: "datasets",
  train: "runs",
  export: "runs",
  push: "models",
};
const labels = {
  datasets: "データセット",
  runs: "学習済み実験",
  models: "ONNXモデル",
};
const fields = ["python", "encoder_repo", "weights", "device", "model_root"];
function suggestNames() {
  if (!catalog) return;
  const base =
    $("e2e-mode").value === "steer_only" ? "dinov3-steer" : "dinov3-control";
  for (const [id, prefix, kind] of [
    ["e2e-run-name", base, "runs"],
    ["e2e-model-name", $("e2e-export-run").value || base, "models"],
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
  for (const row of rows) {
    const option = new Option(row.label || row.id, row.id);
    option.selected = previous.has(row.id);
    select.add(option);
  }
}
function render() {
  if (!catalog) return;
  for (const kind of ["datasets", "runs", "models"])
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
          : "ONNXの変換・検証が完了すると表示されます。";
    list.append(p);
  }
  for (const row of catalog[kind]) {
    let detail =
      kind === "datasets"
        ? `${row.samples.toLocaleString()} samples · ${row.bag.split("/").pop()}`
        : kind === "runs"
          ? `${row.mode} · epoch ${row.metrics?.epoch ?? "—"} · val MSE ${row.metrics?.validation_mse?.toPrecision(4) ?? "—"}`
          : `${row.mode} · ORT最大誤差 ${Number(row.ort_max_abs_error).toExponential(2)}`;
    list.append(
      item(row.id, detail, () => {
        if (step === "dataset") {
          setStep("train");
          $("e2e-train-data").value = row.id;
        } else if (kind === "runs") {
          setStep("export");
          $("e2e-export-run").value = row.id;
          suggestNames();
        } else $("e2e-push-model").value = row.id;
      }),
    );
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
  options(
    "e2e-bag",
    records.map((r) => ({ id: r.id, label: r.name + " · " + r.id })),
    "bagを選択",
  );
  for (const id of ["e2e-train-data", "e2e-validation-data"])
    options(
      id,
      data.datasets.map((d) => ({ ...d, label: `${d.id} (${d.samples})` })),
      "",
    );
  options("e2e-export-run", data.runs, "実験を選択");
  options("e2e-push-model", data.models, "モデルを選択");
  suggestNames();
  render();
}
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
    finetune: $("e2e-finetune").checked,
  }),
);
action("e2e-start-export", () =>
  start("export", {
    name: $("e2e-model-name").value,
    run: $("e2e-export-run").value,
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
on("job-finished", () => {
  if (catalog) return refreshLearning();
});
on("connection", render);
