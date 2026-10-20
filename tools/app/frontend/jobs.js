import { $, state, api, action, item, toast, on, emit } from "./api.js";
import { duration } from "./job_time.js";
import { compactLog } from "./job_log.js";
let selected = null,
  busy = false;
const known = new Map();
const stopping = new Set();
const timeLabel = (job) => `${job.status === "running" ? "経過" : "所要"} ${duration(job.elapsed_s)}`;
const isStopping = (job) => job.cancel_requested || stopping.has(job.id);
const statusLabel = (job) => job.status === "running" && isStopping(job) ? "停止要求中" : labels[job.status];
function stopButtons() {
  const active = state.jobs.find((job) => job.status === "running");
  for (const [element, job] of [
    [$("job-stop-active"), active],
    [$("job-stop-selected"), state.jobs.find((job) => job.id === selected)],
  ]) {
    element.hidden = !job || job.status !== "running";
    element.disabled = !!job && isStopping(job);
    element.textContent = job && isStopping(job) ? "停止要求中…" : "処理を停止";
    element.title = job?.title || "";
    element.dataset.job = job?.id || "";
  }
  const job = state.jobs.find((j) => j.id === selected);
  $("job-log-status").textContent = job ? `${statusLabel(job)} · ${timeLabel(job)}` : "";
}
async function stopJob(id) {
  if (!id || stopping.has(id)) return;
  stopping.add(id);
  stopButtons();
  try {
    const result = await api("stop", { id });
    const job = state.jobs.find((j) => j.id === id);
    if (job) Object.assign(job, result);
    toast(result.status === "running" ? "停止を要求しました。終了処理を待っています" : "この処理は終了しています");
  } catch (error) {
    toast(error.message, true);
  } finally {
    stopping.delete(id);
    stopButtons();
    await poll();
  }
}
for (const id of ["job-stop-active", "job-stop-selected"])
  action(id, () => stopJob($(id).dataset.job));
const labels = {
  running: "実行中",
  succeeded: "完了",
  failed: "失敗",
  cancelled: "中止",
  interrupted: "中断・結果未確認",
};
async function showLog(id) {
  const switching = selected !== id;
  selected = id;
  stopButtons();
  const result = await api("log?id=" + id);
  if (selected !== id) return;
  const view = compactLog(result.text);
  const history = $("job-log");
  const follow = switching || history.scrollHeight-history.scrollTop-history.clientHeight<32;
  history.textContent = view.text;
  $("job-progress").textContent = view.progress;
  $("job-progress").hidden = !view.progress;
  if (follow) history.scrollTop = history.scrollHeight;
  $("job-log-copy").disabled = !view.text && !view.progress;
}
action("job-log-copy", async () => {
  const text = [$("job-log").textContent, $("job-progress").textContent].filter(Boolean).join("\n");
  if (!selected || !text) throw new Error("コピーするログがありません");
  try {
    await navigator.clipboard.writeText(text);
  } catch {
    // HTTP access via a notebook IP may not provide the Clipboard API.
    const input = document.createElement("textarea");
    input.value = text;
    input.readOnly = true;
    input.style.position = "fixed";
    input.style.opacity = "0";
    const focus = document.activeElement;
    document.body.append(input);
    try {
      input.select();
      if (!document.execCommand("copy"))
        throw new Error(
          "コピーできませんでした。ブラウザのコピー権限を確認してください",
        );
    } finally {
      input.remove();
      focus?.focus({ preventScroll: true });
    }
  }
  toast("表示中のログをコピーしました");
});
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
    stopButtons();
    $("job-dot").classList.toggle("running", !!running.length);
    $("job-summary").textContent = running.length
      ? `${running.length}件の処理中 · ${running[0].title} · ${statusLabel(running[0])} · ${timeLabel(running[0])}`
      : state.jobs.length
        ? `${state.jobs[0].title} · ${statusLabel(state.jobs[0])} · ${timeLabel(state.jobs[0])}`
        : "処理は実行されていません";
    if (!$("jobs-drawer").hidden) {
      const list = $("job-list");
      list.replaceChildren();
      for (const job of state.jobs) {
        const row = document.createElement("div");
        row.className = "job-row";
        const button = item(
          job.title,
          `${statusLabel(job)} · ${timeLabel(job)}${job.status === "running" && !isStopping(job) && !job.message?.startsWith("実行: ") ? " · " + job.message : ""}`,
          () => showLog(job.id),
        );
        button.className = "job-select";
        row.append(button);
        if (job.status === "running") {
          const stop = document.createElement("button");
          stop.textContent = isStopping(job) ? "停止要求中…" : "停止";
          stop.disabled = isStopping(job);
          stop.addEventListener("click", () => stopJob(job.id));
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
