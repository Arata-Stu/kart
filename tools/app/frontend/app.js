import { $, state, api, action, emit, toast } from "./api.js";
import { refreshMaps } from "./maps.js";
import { refreshRecords } from "./transfers.js";
import { refreshLearning } from "./learning.js";
import { poll } from "./jobs.js";
import "./dialogs.js";
import "./line_tools.js";
import "./lanes.js";
import "./obstacles.js";
import "./copies.js";
import "./capture.js";
function page(name) {
  for (const p of ["map", "transfer", "learning"]) {
    $(p + "-page").hidden = p !== name;
    $("nav-" + p).classList.toggle("active", p === name);
  }
  $("jobs-drawer").hidden = true;
  if (name === "learning")
    refreshLearning().catch((e) => toast(e.message, true));
  if (name === "transfer")
    refreshRecords().catch((e) => toast(e.message, true));
}
action("nav-map", () => page("map"));
action("nav-transfer", () => page("transfer"));
action("nav-learning", () => page("learning"));
try {
  state.config = await api("config");
  emit("ready");
  await Promise.all([refreshMaps(), refreshRecords(), poll()]);
  emit("document");
} catch (e) {
  toast(e.message, true);
}
