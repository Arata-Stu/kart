import { $, state, api, action, toast, emit, remember, on } from "./api.js";
import { save, loadMap } from "./maps.js";

function length() {
  const pts = state.lane.custom;
  return pts.reduce((total, p, i) => {
    const q = pts[i + 1] || (state.lane.closed ? pts[0] : p);
    return total + Math.hypot(q[0] - p[0], q[1] - p[1]);
  }, 0);
}
function addRow(value = { start: 0, end: Math.min(1, length()), speed: 1 }) {
  if ($("speed-rows").children.length >= 40) return;
  const row = document.createElement("div");
  row.className = "speed-row";
  for (const key of ["start", "end", "speed"]) {
    const input = document.createElement("input");
    Object.assign(input, {
      type: "number",
      value: value[key],
      min: key === "speed" ? 0.1 : 0,
      max: key === "speed" ? 20 : length(),
      step: "any",
      required: true,
    });
    input.dataset.key = key;
    input.setAttribute(
      "aria-label",
      { start: "開始距離", end: "終了距離", speed: "速度上限" }[key],
    );
    row.append(input);
  }
  const remove = document.createElement("button");
  remove.type = "button";
  remove.textContent = "×";
  remove.setAttribute("aria-label", "区間を削除");
  remove.onclick = () => row.remove();
  row.append(remove);
  $("speed-rows").append(row);
}
action("custom-speeds-open", () => {
  if (state.lane.custom.length < (state.lane.closed ? 3 : 2))
    throw Error("先にCustomlineの点を描いてください");
  $("speed-length").textContent =
    `全長 ${length().toFixed(2)} m / 紫の○が始点、数字は始点からの距離 (m)`;
  $("speed-rows").replaceChildren();
  for (const row of state.lane.custom_speeds || []) addRow(row);
  $("speed-dialog").showModal();
});
action("speed-add", () => addRow());
$("speed-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const limits = [...$("speed-rows").children].map((row) =>
    Object.fromEntries(
      [...row.querySelectorAll("input")].map((el) => [
        el.dataset.key,
        Number(el.value),
      ]),
    ),
  );
  if (
    limits.some(
      (row) =>
        row.start === row.end || (!state.lane.closed && row.start > row.end),
    )
  )
    return toast(
      "始終点を確認してください。始点を跨ぐ指定は閉路のみです",
      true,
    );
  remember();
  state.lane.custom_speeds = limits;
  delete state.lane.lines.customline;
  state.dirty = true;
  $("speed-dialog").close();
  emit("document");
  toast("区間速度を適用しました。Customlineを再生成してください");
});
let request = null;
function clearPreview() {
  state.registrationPreview = null;
  request = null;
  $("registration-preview-bar").hidden = true;
  emit("view");
}
action("registration-open", async () => {
  await save();
  clearPreview();
  const select = $("registration-source");
  select.replaceChildren();
  for (const map of state.maps.filter((map) => map.id !== state.id)) {
    const option = document.createElement("option");
    option.value = map.id;
    option.textContent = map.title;
    select.append(option);
  }
  if (!select.options.length)
    throw Error("位置合わせ元になる別の地図が必要です");
  $("registration-dialog").showModal();
});
$("registration-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const button = event.submitter;
  button.disabled = true;
  try {
    const data = Object.fromEntries(new FormData(event.target));
    const body = { id: state.id, revision: state.doc.revision, ...data };
    const result = await api("registration-preview", body);
    request = { ...body, token: result.token };
    state.registrationPreview = result.document;
    $("registration-dialog").close();
    $("registration-preview-bar").hidden = false;
    emit("view");
  } catch (e) {
    toast(e.message, true);
  } finally {
    button.disabled = false;
  }
});
action("registration-adjust", () => $("registration-dialog").showModal());
action("registration-cancel", clearPreview);
action("registration-apply", async () => {
  if (!request || state.dirty || request.id !== state.id)
    throw Error("編集内容が変わっています。再プレビューしてください");
  const id = state.id;
  await api("registration-apply", request);
  clearPreview();
  await loadMap(id, true);
  toast("HDMapとラインを位置合わせしました。置換前の編集状態も保存しました");
});
on("document", () => {
  if (!state.registrationPreview || state.dirty) clearPreview();
});
