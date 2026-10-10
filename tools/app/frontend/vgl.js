import { $, state, api, action, emit } from "./api.js";
let target = null;
action("vgl-open", async () => {
  target = state.id;
  const data = await api("vgl?id=" + encodeURIComponent(target));
  $("vgl-name").value = data.name;
  $("vgl-models").replaceChildren();
  for (const model of data.models) {
    const option = document.createElement("option");
    option.value = model;
    $("vgl-models").append(option);
  }
  $("vgl-model").value = data.models[0] || "";
  $("vgl-width").value = data.defaults.width;
  $("vgl-height").value = data.defaults.height;
  $("vgl-info").textContent = data.error || `元地図: ${target} · 生成済みbundle: ${data.bundles.length}件`;
  $("vgl-start").disabled = !!data.error;
  $("vgl-dialog").showModal();
});
action("vgl-start", async () => {
  if (!target || target !== state.id) throw Error("対象地図を選び直してください");
  const job = await api("vgl", {id: target, name: $("vgl-name").value,
    model_dir: $("vgl-model").value, width: Number($("vgl-width").value), height: Number($("vgl-height").value)});
  $("vgl-dialog").close();
  emit("job-started", job);
});
