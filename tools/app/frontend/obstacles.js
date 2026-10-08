import { $, state, action, on, emit, remember, edited } from "./api.js";
function update() {
  const rows = state.doc?.obstacles || [];
  if (!rows.some((o) => o.id === state.obstacleId))
    state.obstacleId = rows[0]?.id || null;
  const select = $("obstacle-select");
  select.replaceChildren(
    ...rows.map((o) => {
      const e = document.createElement("option");
      e.value = e.textContent = o.id;
      return e;
    }),
  );
  select.value = state.obstacleId || "";
  const disabled = !state.doc || state.doc.snapshot_status === "pending";
  $("obstacle-add").disabled = disabled || rows.length >= 100;
  for (const id of ["obstacle-select", "obstacle-remove", "edit-obstacle"])
    $(id).disabled = disabled || !rows.length;
  $("edit-obstacle").classList.toggle("active", state.target === "obstacle");
}
$("obstacle-select").addEventListener("change", () => {
  state.obstacleId = $("obstacle-select").value;
  state.selected = -1;
  emit("document");
});
action("edit-obstacle", () => {
  state.target = "obstacle";
  state.selected = -1;
  emit("document");
});
action("obstacle-add", () => {
  const rows = (state.doc.obstacles ||= []);
  let n = 1;
  while (rows.some((o) => o.id === `obstacle_${n}`)) n++;
  remember();
  state.obstacleId = `obstacle_${n}`;
  rows.push({ id: state.obstacleId, polygon: [] });
  state.target = "obstacle";
  state.selected = -1;
  edited();
});
action("obstacle-remove", () => {
  if (!state.obstacle || !confirm(`「${state.obstacleId}」を削除しますか？`))
    return;
  remember();
  state.doc.obstacles = state.doc.obstacles.filter(
    (o) => o.id !== state.obstacleId,
  );
  state.selected = -1;
  state.target = "obstacle";
  edited();
});
on("document", update);
on("status", update);
