import { $, state, api, action, emit } from "./api.js";
let pending = null;
action("capture-open", async () => {
  if (state.doc?.snapshot_status !== "pending") return;
  pending = { id: state.id, revision: state.doc.revision };
  const records = await api("records");
  $("capture-bag").replaceChildren();
  for (const record of records) {
    const option = document.createElement("option");
    option.value = record.id;
    option.textContent = record.id;
    $("capture-bag").append(option);
  }
  $("capture-start").disabled = !records.length;
  $("capture-dialog").showModal();
});
action("capture-start", async () => {
  if (!pending || pending.id !== state.id)
    throw Error("対象地図を開き直してください");
  const job = await api("capture", { ...pending, bag: $("capture-bag").value });
  $("capture-dialog").close();
  emit("job-started", job);
});
