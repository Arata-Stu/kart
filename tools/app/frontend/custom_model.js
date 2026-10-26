// Pure editing operations. Persisted geometry and speed-section schema stay unchanged.
export function copyCustom(lane, source) {
  if (source === "manual") return [];
  if (!["centerline", "raceline"].includes(source)) throw Error("作成方法を選んでください");
  const points = lane.lines[source]?.points;
  if (!points || points.length < (lane.closed ? 3 : 2))
    throw Error("先にコピー元のラインを生成してください");
  const result = points.map(([x, y]) => [x, y]);
  if (lane.closed && result.length > 3 &&
      result[0].every((v, i) => v === result.at(-1)[i])) result.pop();
  return result;
}

export const sectionColor = (i) => `hsl(${(i * 137.508 + 205) % 360} 62% 40%)`;
export const covers = (row, s) => row.start < row.end
  ? s >= row.start && s < row.end : s >= row.start || s < row.end;

export function partitionSections(rows, total, defaultSpeed) {
  if (!(total > 0) || !(defaultSpeed > 0)) throw Error("有効なラインと速度が必要です");
  const cuts = [...new Set([0, total, ...rows.flatMap(r => [r.start, r.end])])]
    .filter(s => Number.isFinite(s) && s >= 0 && s <= total).sort((a, b) => a - b);
  const result = [];
  for (let i = 1; i < cuts.length; i++) {
    const start = cuts[i - 1], end = cuts[i];
    const matches = rows.filter(r => covers(r, (start + end) / 2));
    result.push({start, end, speed: matches.length ? Math.min(...matches.map(r => r.speed)) : defaultSpeed});
  }
  if (result.length > 40) throw Error("分割後の区間が40個を超えます。詳細指定で区間を整理してください");
  return result;
}

export function splitSection(rows, station) {
  if (rows.length >= 40) throw Error("区間は最大40個です");
  const index = rows.findIndex(r => station > r.start + 1e-6 && station < r.end - 1e-6);
  if (index < 0) return rows.map(r => ({...r}));
  return rows.flatMap((r, i) => i === index
    ? [{...r, end: station}, {...r, start: station}] : [{...r}]);
}

export function mergeSection(rows, index) {
  if (rows.length < 2) return rows.map(r => ({...r}));
  const left = index === 0 ? 0 : index - 1;
  // The lower limit survives a merge; no implicit speed increase.
  return rows.flatMap((r, i) => i === left
    ? [{start: r.start, end: rows[i + 1].end, speed: Math.min(r.speed, rows[i + 1].speed)}]
    : i === left + 1 ? [] : [{...r}]);
}

export function hasOverlap(rows, total) {
  const cuts = [...new Set([0, total, ...rows.flatMap(r => [r.start, r.end])])].sort((a,b) => a-b);
  return cuts.slice(1).some((end,i) => rows.filter(r => covers(r,(cuts[i]+end)/2)).length > 1);
}

export function isPartition(rows, total) {
  return rows.length > 0 && rows[0].start === 0 && rows.at(-1).end === total &&
    rows.every((r, i) => r.start < r.end && (i === 0 || r.start === rows[i - 1].end));
}
