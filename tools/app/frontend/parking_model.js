export function parkingWarnings(doc) {
  const ids = new Set((doc?.parking_areas || []).map(a => a.id));
  const missing = ['P1', 'P2', 'P3'].filter(id => !ids.has(id));
  return missing.length ? `駐車areaを定義してください。未定義: ${missing.join('、')}。` : '';
}
export function parsePoints(text) {
  const rows = text.trim() ? text.trim().split(/\n/).filter(s => s.trim()).map(s => s.trim().split(/[\s,]+/).filter(Boolean).map(Number)) : [];
  if (rows.some(p => p.length !== 2 || p.some(v => !Number.isFinite(v)))) throw Error('座標は1行に x y（m）を入力してください');
  return rows;
}
export const pointText = rows => (rows || []).map(p => p.join(' ')).join('\n');
