import { $, state, api, action, emit, toast } from "./api.js";
import { save } from "./maps.js";

let source = null;
action("copy-map", () => {
  if (!state.doc) return;
  source = state.id;
  const match = source.match(/^(.*)_v(\d+)$/);
  const base = match ? match[1] : source;
  let version = match ? Number(match[2]) + 1 : 2;
  let candidate;
  do {
    const suffix = "_v" + version++;
    candidate = base.slice(0, 64 - suffix.length) + suffix;
  } while (state.maps.some((map) => map.id === candidate));
  $("copy-map-name").value = candidate;
  $("copy-map-source").textContent = `複製元: ${state.doc.title}`;
  $("copy-map-dialog").showModal();
});
action("copy-map-confirm", async () => {
  if (!source || state.id !== source) throw Error("元地図を開き直してください");
  const name = $("copy-map-name").value.trim();
  if (!/^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$/.test(name))
    throw Error(
      "地図名は英数字で始まる英数字・_・-（64文字以内）にしてください",
    );
  if (state.maps.some((map) => map.id === name))
    throw Error("同名の地図があります");
  await save();
  if (state.id !== source) throw Error("元地図が切り替わりました");
  const job = await api("copy-map", {
    id: source,
    name,
    revision: state.doc.revision,
  });
  $("copy-map-dialog").close();
  emit("job-started", job);
  toast("複製を開始しました。完了後に新しい地図を開きます");
});
