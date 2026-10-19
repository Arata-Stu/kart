// Version numbers are suggestions; the server still rejects overwriting artifacts.
export function nextVersion(base, ids) {
  base = base.replace(/-v\d+$/, "");
  let version = 1;
  for (const id of ids) {
    if (!id.startsWith(base + "-v")) continue;
    const suffix = id.slice(base.length + 2);
    if (/^\d+$/.test(suffix)) version = Math.max(version, Number(suffix) + 1);
  }
  return `${base}-v${version}`;
}

// Keep the path as identity: repeated names on different dates are distinct datasets.
export function datasetOption(row) {
  const parts = row.id.split("/");
  const dated = parts.length === 3;
  return {
    ...row,
    group: dated ? parts[0] : "従来のデータセット",
    label: `${dated ? parts[1] + " · " : ""}${row.name || parts.at(-1)} (${row.samples.toLocaleString()})`,
  };
}

// Match the server's artifact-name rules while keeping the bag's run name.
export function datasetName(record) {
  if (!record) return "";
  const raw = record.name || record.id.split("/").filter(Boolean).at(-1) || "dataset";
  return raw.normalize("NFKC").replace(/\.mcap$/i, "")
    .replace(/[^A-Za-z0-9_-]+/g, "_").replace(/^[^A-Za-z0-9]+/, "")
    .slice(0, 64) || "dataset";
}
