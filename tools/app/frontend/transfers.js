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
const folders = new Map();
const expanded = new Set();
let connectionVersion = 0;
function handle(button, fn) {
  button.addEventListener("click", async () => {
    button.disabled = true;
    try {
      await fn();
    } catch (error) {
      toast(error.message, true);
    } finally {
      button.disabled = false;
    }
  });
}
function renderRemote() {
  const path = selected?.relative || relative;
  const crumbs = $("remote-path");
  crumbs.replaceChildren();
  crumbs.title = state.config.connection.record_root + (path ? "/" + path : "");
  const parts = path ? path.split("/") : [];
  for (let i = 0; i <= parts.length; i++) {
    if (i) crumbs.append(" / ");
    const button = document.createElement("button");
    button.textContent = i ? parts[i - 1] : "record";
    if (i === parts.length) button.setAttribute("aria-current", "location");
    const target = parts.slice(0, i).join("/");
    if (selected && i === parts.length) button.disabled = true;
    else handle(button, () => browse(target));
    crumbs.append(button);
  }
  $("remote-up").disabled = !path;
  $("pull").disabled = !selected;
  const list = $("remote-list");
  list.replaceChildren();
  function branch(entry, parent) {
    const wrapper = document.createElement("li");
    const button = document.createElement("button");
    const isOpen = expanded.has(entry.relative);
    button.className = "remote-tree-row";
    button.textContent = (entry.bag ? "▣ " : isOpen ? "▾ " : "▸ ") + entry.name;
    button.title = state.config.connection.record_root + "/" + entry.relative;
    button.classList.toggle("selected", selected?.relative === entry.relative);
    button.classList.toggle(
      "current-folder",
      !selected && relative === entry.relative,
    );
    if (entry.bag)
      button.setAttribute(
        "aria-pressed",
        String(selected?.relative === entry.relative),
      );
    else button.setAttribute("aria-expanded", String(isOpen));
    handle(button, async () => {
      if (entry.bag) {
        selected = entry;
        relative = entry.relative.split("/").slice(0, -1).join("/");
        $("pull-name").value = entry.name
          .replace(/[^A-Za-z0-9_-]/g, "_")
          .replace(/^[^A-Za-z0-9]+/, "")
          .slice(0, 64);
      } else if (isOpen) {
        expanded.delete(entry.relative);
        if (
          !entry.relative ||
          selected?.relative.startsWith(entry.relative + "/")
        )
          selected = null;
        relative = entry.relative;
      } else {
        await browse(entry.relative);
        return;
      }
      renderRemote();
    });
    wrapper.append(button);
    parent.append(wrapper);
    if (!entry.bag && isOpen) {
      const children = document.createElement("ul");
      wrapper.append(children);
      const entries = folders.get(entry.relative) || [];
      for (const child of entries) branch(child, children);
      if (!entries.length) {
        const empty = document.createElement("li");
        empty.className = "remote-tree-empty";
        empty.textContent = "サブフォルダなし";
        children.append(empty);
      }
    }
  }
  const tree = document.createElement("ul");
  tree.className = "remote-tree";
  tree.setAttribute("aria-label", "Jetsonのrecordフォルダ");
  branch({ name: "record", relative: "", bag: false }, tree);
  list.append(tree);
}
async function browse(path = "") {
  if (!state.config.connection.host || !state.config.connection.user) {
    $("connection-open").click();
    return;
  }
  const version = connectionVersion;
  const r = await api("browse", {
    connection: state.config.connection,
    relative: path,
  });
  if (version !== connectionVersion) return;
  folders.set(r.relative, r.entries);
  expanded.add("");
  const parts = r.relative.split("/").filter(Boolean);
  for (let i = 1; i <= parts.length; i++)
    expanded.add(parts.slice(0, i).join("/"));
  relative = r.relative;
  selected = null;
  renderRemote();
}
action("browse-remote", () => browse(folders.has("") ? relative : ""));
action("remote-up", () =>
  browse((selected?.relative || relative).split("/").slice(0, -1).join("/")),
);
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
  connectionVersion++;
  folders.clear();
  expanded.clear();
  relative = "";
  selected = null;
  $("remote-list").replaceChildren();
  $("remote-path").textContent = "record /";
  $("remote-up").disabled = true;
  $("pull").disabled = true;
});
on("job-finished", () => refreshRecords().catch((e) => toast(e.message, true)));
on("ready", connection);
