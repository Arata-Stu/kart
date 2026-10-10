import { $, api } from "./api.js";
let requested = null;
export async function showEvaluation(id) {
  requested = id;
  const report = await api("e2e/evaluation?id=" + encodeURIComponent(id));
  if (requested !== id) return;
  const root = $("e2e-eval-result");
  root.replaceChildren();
  const title = document.createElement("h3");
  title.textContent = `${id} · ${report.samples} frames`;
  root.append(title);
  const note = document.createElement("p");
  note.className = "help";
  note.textContent = `${report.mode} / ${report.provider || "CPUExecutionProvider"} 推論中央値 ${report.inference_ms_median.toFixed(1)} ms（前処理を除く）· 教師=灰 / 指令予測=紫。steer-onlyのスロットルはmetadataの固定値です。`;
  root.append(note);
  if (report.cuda_fallback_reason) {
    const warning = document.createElement("p");
    warning.className = "help";
    warning.textContent = "CUDA未使用: " + report.cuda_fallback_reason;
    root.append(warning);
  }
  if (report.inference_ms) {
    const stats = document.createElement("p");
    const t = report.inference_ms;
    stats.textContent = `ONNX ${(report.onnx_size_bytes / 1048576).toFixed(1)} MiB · ${report.provider || "CPUExecutionProvider"} 推論 平均 ${t.mean.toFixed(2)} / 中央値 ${t.median.toFixed(2)} / P95 ${t.p95.toFixed(2)} ms · warmup ${report.warmup_runs}回を除外`;
    root.append(stats);
  }
  for (const [key, label, low, high] of [
    ["steering", "ステア", -1, 1],
    ["throttle", "スロットル", 0, 1],
  ]) {
    const m = report.metrics[key + "_command"];
    const raw = report.metrics[key + "_raw"];
    const caption = document.createElement("p");
    caption.textContent = `${label}：指令MAE ${m.mae.toFixed(4)} / RMSE ${m.rmse.toFixed(4)} / P95 ${m.p95_abs.toFixed(4)} · 生出力MAE ${raw.mae.toFixed(4)}`;
    root.append(caption);
    const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    svg.setAttribute("viewBox", "0 0 500 140");
    svg.setAttribute("aria-label", label + "の教師と予測、横軸は秒");
    svg.style.width = "100%";
    svg.style.background = "#f4f6f8";
    const rows = report.preview;
    const times = rows.map((r) => r.time_s);
    const t0 = Math.min(...times),
      t1 = Math.max(...times);
    for (const [field, color] of [
      [key + "_label", "#7a8a95"],
      [key + "_command", "#9063b0"],
    ]) {
      const line = document.createElementNS(svg.namespaceURI, "polyline");
      line.setAttribute(
        "points",
        rows
          .map(
            (r) =>
              `${30 + (460 * (r.time_s - t0)) / Math.max(t1 - t0, 0.001)},${10 + (100 * (high - r[field])) / (high - low)}`,
          )
          .join(" "),
      );
      line.setAttribute("fill", "none");
      line.setAttribute("stroke", color);
      line.setAttribute("stroke-width", "1.5");
      svg.append(line);
    }
    for (const [x, y, text] of [
      [2, 15, high],
      [2, 110, low],
      [30, 132, `${t0.toFixed(1)} s`],
      [445, 132, `${t1.toFixed(1)} s`],
    ]) {
      const label = document.createElementNS(svg.namespaceURI, "text");
      label.setAttribute("x", x);
      label.setAttribute("y", y);
      label.setAttribute("font-size", 11);
      label.textContent = text;
      svg.append(label);
    }
    root.append(svg);
  }
}

export async function showEngine(id) {
  const root = $("e2e-engine-result");
  if (!id) {
    root.textContent = "";
    return;
  }
  const report = await api("e2e/engine?id=" + encodeURIComponent(id));
  if ($("e2e-engine-history").value !== id) return;
  root.textContent = `ONNX ${(report.onnx_size_bytes / 1048576).toFixed(1)} MiB / engine ${(report.engine_size_bytes / 1048576).toFixed(1)} MiB\nBuild＋計測 ${report.total_build_and_benchmark_s.toFixed(1)} s\nGPU: ${report.gpu}\n合成入力の計測（ms、rosbag精度評価とは別）\n${JSON.stringify(report.timing_ms, null, 2)}`;
}
