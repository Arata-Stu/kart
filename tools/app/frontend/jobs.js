import { $, state, api, action, item, toast, on, emit } from "./api.js";
let selected = null,
  busy = false;
const known = new Map();
const labels = {
  running: "実行中",
  succeeded: "完了",
  failed: "失敗",
  cancelled: "中止",
  interrupted: "中断・結果未確認",
};
async function showLog(id) {
  selected = id;
  const result = await api("log?id=" + id);
  $("job-log").textContent = result.text;
}
export async function poll() {
  if (busy) return;
  busy = true;
  try {
    state.jobs = await api("jobs");
    for (const job of state.jobs) {
      if (known.get(job.id) === "running" && job.status !== "running") {
        emit("job-finished", job);
        toast(`${job.title}: ${job.message}`, job.status === "failed");
      }
      known.set(job.id, job.status);
    }
    const running = state.jobs.filter((j) => j.status === "running");
    $("job-dot").classList.toggle("running", !!running.length);
    $("job-summary").textContent = running.length
      ? `${running.length}件の処理中 · ${running[0].title}`
      : state.jobs.length
        ? `${state.jobs[0].title} · ${labels[state.jobs[0].status]}`
        : "処理は実行されていません";
    if (!$("jobs-drawer").hidden) {
      const list = $("job-list");
      list.replaceChildren();
      for (const job of state.jobs) {
        const row = document.createElement("div");
        row.className = "job-row";
        const button = item(job.title, labels[job.status], () =>
          showLog(job.id),
        );
        button.className = "job-select";
        row.append(button);
        if (job.status === "running") {
          const stop = document.createElement("button");
          stop.textContent = "中止";
          stop.addEventListener("click", () =>
            api("stop", { id: job.id }).catch((e) => toast(e.message, true)),
          );
          row.append(stop);
        }
        list.append(row);
      }
      if (selected) await showLog(selected);
    }
    emit("status");
  } catch (e) {
    toast(e.message, true);
  } finally {
    busy = false;
  }
}
action("jobs-toggle", async () => {
  $("jobs-drawer").hidden = !$("jobs-drawer").hidden;
  await poll();
});
action("jobs-close", () => {
  $("jobs-drawer").hidden = true;
});
on("job-started", (e) => {
  known.set(e.detail.id, "running");
  selected = e.detail.id;
  poll();
});
setInterval(poll, 1500);
