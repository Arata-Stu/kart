import { $, state, emit, on } from "./api.js";

export function lineVisible(kind) { return state.visibleLines[kind] !== false; }
export function showLine(kind) {
  state.visibleLines[kind] = true;
  render();
}
function render() {
  for (const kind of ["centerline", "raceline", "customline"])
    $("show-" + kind).checked = lineVisible(kind);
}
for (const kind of ["centerline", "raceline", "customline"]) {
  $("show-" + kind).addEventListener("change", () => {
    state.visibleLines[kind] = $("show-" + kind).checked;
    if (kind === "customline" && !lineVisible(kind)) state.selected = -1;
    emit("view");
    emit("status");
  });
}
for (const name of ["ready", "document"]) on(name, render);
