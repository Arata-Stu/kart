export function duration(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return "時間不明";
  const total = Math.floor(seconds), h = Math.floor(total / 3600);
  const m = Math.floor(total / 60) % 60, s = total % 60;
  return h ? `${h}時間 ${m}分 ${s}秒` : m ? `${m}分 ${s}秒` : `${s}秒`;
}
