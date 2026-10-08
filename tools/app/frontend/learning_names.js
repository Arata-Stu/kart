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
