import { $, state, api, action, item, toast, on, emit } from "./api.js";
import { openBuild } from "./dialogs.js";
let relative = "",
  selected = null;
export async function refreshRecords() {
  state.records = await api("records");
  const list = $("local-records");
  list.replaceChildren();
  if (!state.records.length) {
    const p = document.createElement("p");
    p.className = "empty";
    p.textContent = "受信したbagがここに表示されます。";
    list.append(p);
  }
  for (const r of state.records)
    list.append(
      item(r.name, r.id + " · 地図作成を開く", () => openBuild(r.id)),
    );
  $("local-path").textContent = state.config.records;
}
function connection() {
  const p = state.config.connection;
  $("connection-summary").textContent = p.host
    ? `${p.user}@${p.host}:${p.port} · ${p.record_root}`
    : "接続設定を入力してください。SSH鍵認証を使用します。";
}
async function browse(path = "") {
  if (!state.config.connection.host || !state.config.connection.user) {
    $("connection-open").click();
    return;
  }
  const r = await api("browse", {
    connection: state.config.connection,
    relative: path,
  });
  relative = r.relative;
  selected = null;
  $("pull").disabled = true;
  $("remote-path").textContent = "record / " + relative;
  const list = $("remote-list");
  list.replaceChildren();
  if (!r.entries.length) {
    const p = document.createElement("p");
    p.className = "empty";
    p.textContent = "サブフォルダがありません。";
    list.append(p);
  }
  for (const entry of r.entries) {
    const button = item(
      (entry.bag ? "▣ " : "▱ ") + entry.name,
      entry.bag ? "rosbag · クリックで選択" : "フォルダを開く",
      async () => {
        if (!entry.bag) return browse(entry.relative);
        selected = entry;
        for (const row of list.children) row.classList.remove("selected");
        button.classList.add("selected");
        $("pull-name").value = entry.name
          .replace(/[^A-Za-z0-9_-]/g, "_")
          .replace(/^[^A-Za-z0-9]+/, "")
          .slice(0, 64);
        $("pull").disabled = false;
      },
    );
    list.append(button);
  }
}
action("browse-remote", () => browse(relative));
action("remote-up", () => browse(relative.split("/").slice(0, -1).join("/")));
action("refresh-records", refreshRecords);
action("pull", async () => {
  if (!selected) throw new Error("bagを選択してください");
  const job = await api("pull", {
    connection: state.config.connection,
    relative: selected.relative,
    name: $("pull-name").value,
  });
  emit("job-started", job);
  toast("bagの受信を開始しました");
});
action("push", async () => {
  const id = $("push-map").value;
  if (!id) throw new Error("送信する地図を選択してください");
  if (!state.config.connection.host || !state.config.connection.user) {
    $("connection-open").click();
    return;
  }
  if (state.id === id && state.dirty)
    throw new Error("地図画面で編集を保存してから送信してください");
  const job = await api("push", { connection: state.config.connection, id });
  emit("job-started", job);
  toast("新規フォルダへの地図送信を開始しました");
});
on("catalog", () => {
  const select = $("push-map"),
    previous = select.value;
  select.replaceChildren();
  const empty = document.createElement("option");
  empty.value = "";
  empty.textContent = "地図を選択";
  select.append(empty);
  for (const m of state.maps) {
    const o = document.createElement("option");
    o.value = m.id;
    o.textContent =
      m.title +
      (!m.lines.includes("centerline")
        ? "（Centerline未生成）"
        : m.has_vslam
          ? "（VSLAM＋HDMap）"
          : "（HDMap・点群のみ）");
    o.disabled = !m.lines.includes("centerline");
    select.append(o);
  }
  select.value = previous || state.id || "";
});
on("connection", () => {
  connection();
  relative = "";
  selected = null;
  $("remote-list").replaceChildren();
  $("remote-path").textContent = "record /";
  $("pull").disabled = true;
});
on("job-finished", () => refreshRecords().catch((e) => toast(e.message, true)));
on("ready", connection);
