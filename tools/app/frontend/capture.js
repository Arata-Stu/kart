import { $, state, api, action, emit } from "./api.js";
let pending = null;
action("capture-open", async () => {
  if (state.doc?.snapshot_status !== "pending") return;
  pending = { id: state.id, revision: state.doc.revision };
  const [records, vgl] = await Promise.all([
    api("records"), api("vgl?id=" + encodeURIComponent(state.id)),
  ]);
  for (const [id, values] of [["capture-vgl", vgl.bundles], ["capture-model", vgl.models]]) {
    $(id).replaceChildren();
    for (const value of values) {
      const option = document.createElement("option");
      option.value = option.textContent = value;
      $(id).append(option);
    }
  }
  $("capture-info").textContent = !vgl.bundles.length
    ? "先にこの地図の「VGL地図生成」を実行してください。"
    : !vgl.models.length ? "実行GPU用のVGLモデルを準備してください。" : "";
  $("capture-bag").replaceChildren();
  for (const record of records) {
    const option = document.createElement("option");
    option.value = record.id;
    option.textContent = record.id;
    $("capture-bag").append(option);
  }
  $("capture-start").disabled = !records.length || !vgl.bundles.length || !vgl.models.length;
  $("capture-dialog").showModal();
});
action("capture-start", async () => {
  if (!pending || pending.id !== state.id)
    throw Error("対象地図を開き直してください");
  const job = await api("capture", { ...pending, bag: $("capture-bag").value,
    vgl_bundle: $("capture-vgl").value, model_dir: $("capture-model").value });
  $("capture-dialog").close();
  emit("job-started", job);
});
