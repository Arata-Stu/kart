import { $, state, on, emit, api } from "./api.js";
import { rasterTransform } from "./occupancy_geometry.js";

let raster = null, owner = null, request = 0, version = null;
on("document", async () => {
  const nextVersion = state.doc?.mapping?.dense_output || state.id;
  if (owner === state.id && version === nextVersion) return;
  version = nextVersion;
  owner = state.id;
  const ticket = ++request, id = owner;
  raster = null;
  $("show-occupancy").checked = false;
  $("show-occupancy").disabled = true;
  $("occupancy-opacity").disabled = true;
  $("occupancy-note").textContent = id ? "占有地図を確認中" : "";
  emit("view");
  if (!id) return;
  try {
    const info = await api("occupancy-overlay?id=" + encodeURIComponent(id));
    if (ticket !== request || state.id !== id) return;
    if (!info.available) {
      $("occupancy-note").textContent = info.reason;
      return;
    }
    const image = new Image();
    image.src = info.url;
    await image.decode();
    if (ticket !== request || state.id !== id) return;
    raster = { ...info, image };
    $("show-occupancy").disabled = false;
    $("occupancy-opacity").disabled = false;
    $("occupancy-note").textContent = "元map座標 · 黒:占有／灰:不明";
    emit("view");
  } catch (error) {
    if (ticket === request && state.id === id)
      $("occupancy-note").textContent = "占有地図を表示できません: " + error.message;
  }
});
for (const name of ["show-occupancy", "occupancy-opacity"])
  $(name).addEventListener("input", () => emit("view"));

export function paintOccupancy(ctx, project, scale) {
  if (!raster || owner !== state.id || !$("show-occupancy").checked) return;
  const { image, origin, resolution } = raster;
  ctx.save();
  ctx.globalAlpha = Number($("occupancy-opacity").value);
  ctx.imageSmoothingEnabled = false;
  // PNG rows go down; map cells go up from the bottom-left YAML origin.
  ctx.transform(...rasterTransform(origin, resolution, image.height, project, scale));
  ctx.drawImage(image, 0, 0);
  ctx.restore();
}
