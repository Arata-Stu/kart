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
  note.textContent = `${report.mode} / CPU推論中央値 ${report.inference_ms_median.toFixed(1)} ms（前処理を除く）· 教師=灰 / 指令予測=紫。steer-onlyのスロットルはmetadataの固定値です。`;
  root.append(note);
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
