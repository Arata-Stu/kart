import { $, state, api, action, emit, toast } from "./api.js";
import { refreshMaps } from "./maps.js";
import { refreshRecords } from "./transfers.js";
import { refreshLearning } from "./learning.js";
import { refreshCoverage } from "./coverage.js";
import { refreshAudit } from "./audit.js";
import { poll } from "./jobs.js";
import "./dialogs.js";
import "./line_tools.js";
import "./custom_editor.js";
import "./simulation.js";
import "./tuning.js";
import "./esc_trial.js";
import "./bringup.js";
import "./tuning_parameters.js";
import "./tuning_speed.js";
import "./lanes.js";
import "./obstacles.js";
import "./parking.js";
import "./copies.js";
import "./capture.js";
import "./vgl.js";
import "./mesh_view.js";
function page(name) {
  for (const p of ["map", "transfer", "learning", "audit", "coverage", "esc", "bringup"]) {
    $(p + "-page").hidden = p !== name;
    $("nav-" + p).classList.toggle("active", p === name);
  }
  $("jobs-drawer").hidden = true;
  if (name === "learning")
    refreshLearning().catch((e) => toast(e.message, true));
  if (name === "coverage") refreshCoverage().catch((e) => toast(e.message, true));
  if (name === "audit")
    refreshAudit().catch((e) => toast(e.message, true));
  if (name === "transfer")
    refreshRecords().catch((e) => toast(e.message, true));
}
action("nav-map", () => page("map"));
action("nav-transfer", () => page("transfer"));
action("nav-learning", () => page("learning"));
action("nav-coverage", () => page("coverage"));
action("nav-audit", () => page("audit"));
action("nav-esc", () => page("esc"));
action("nav-bringup", () => page("bringup"));
try {
  state.config = await api("config");
  emit("ready");
  await Promise.all([refreshMaps(), refreshRecords(), poll()]);
  emit("document");
} catch (e) {
  toast(e.message, true);
}
