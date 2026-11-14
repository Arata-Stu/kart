import { $, state, api, action, on, emit, toast } from './api.js';
import { save, refreshMaps } from './maps.js';
import { fit } from './canvas.js';
import { restoreLineSettings } from './line_settings.js';
import { tuning, prepareTuningPreview } from './tuning.js';

let mapId = null, versions = [], busy = false, serial = 0;
function options(id) {
  const select = $(id), selected = select.value;
  const rows = [['', 'HDMap全体の版を選択'], ...versions.map(v => [v.id,
    `${v.id === 'previous' ? '↶ ' : ''}${v.name} · ${v.lane_count} lanes${!v.compatible ? ' · 元地図不一致' : v.generated ? '' : ' · 要生成'}`])];
  if (JSON.stringify([...select.options].map(o => [o.value, o.textContent])) === JSON.stringify(rows)) return;
  select.replaceChildren();
  for (const [value, text] of rows) {
    const option = document.createElement('option');
    option.value = value; option.textContent = text; select.append(option);
  }
  select.value = versions.some(v => v.id === selected) ? selected : '';
}
function render() {
  options('hdmap-version-select'); options('tuning-hdmap-version');
  const running = state.jobs.some(j => j.status === 'running' && (j.title.includes(state.id) || j.title.includes('生成')));
  const unavailable = busy || tuning.busy || !state.doc || state.doc.snapshot_status === 'pending' || running;
  for (const id of ['hdmap-version-name', 'hdmap-version-select', 'tuning-hdmap-version', 'hdmap-version-refresh']) $(id).disabled = unavailable;
  $('hdmap-version-create').disabled = unavailable || versions.filter(v => v.id !== 'previous').length >= 32;
  for (const [button, select, preview] of [['hdmap-version-load', 'hdmap-version-select', false],
    ['hdmap-version-preview', 'hdmap-version-select', true], ['tuning-hdmap-preview', 'tuning-hdmap-version', true]]) {
    const version = versions.find(v => v.id === $(select).value);
    $(button).disabled = unavailable || state.dirty || !version?.compatible || (preview && !version.generated);
  }
  $('hdmap-version-state').textContent = `${versions.filter(v => v.id !== 'previous').length}/32 保存版 · 作業用の編集元: ${state.doc?.hdmap_version?.name || '元のHDMap'}`;
}
async function refresh() {
  const id = state.id, request = ++serial;
  if (id !== mapId) versions = [];
  mapId = id;
  if (!id) { render(); return; }
  const result = await api('hdmap/versions?id=' + encodeURIComponent(id));
  if (request !== serial || state.id !== id) return;
  versions = result; render();
}
action('hdmap-version-refresh', refresh);
action('hdmap-version-create', async () => {
  const id = state.id, lane = state.laneId, name = $('hdmap-version-name').value.trim();
  if (!name) throw Error('HDMapの版名を入力してください');
  busy = true; render();
  try {
    await save();
    if (state.id !== id || state.laneId !== lane || state.dirty) throw Error('地図が変わりました。版の保存をやり直してください');
    const version = await api('hdmap/version-create', {id, name, revision:state.doc.revision,
      lane_id:lane, line_type:$('tuning-line').value || 'centerline'});
    if (state.id !== id) return;
    await refresh();
    $('hdmap-version-select').value = version.id; $('tuning-hdmap-version').value = version.id;
    toast(`${version.name}をHDMap版として保存しました。点群・自己位置推定用地図は共用します`);
  } finally { busy = false; render(); }
});
async function restore(select, preview) {
  if (state.dirty) throw Error('先に作業用HDMapの編集を保存してください');
  const version = versions.find(v => v.id === $(select).value);
  if (!version?.compatible) throw Error('同じ自己位置推定用地図のHDMap版を選んでください');
  const id = state.id, working = state.doc, before = JSON.stringify(working);
  busy = true; render();
  try {
    const result = await api('hdmap/version-restore', {id, version_id:version.id,
      revision:working.revision, lane_id:state.laneId, line_type:$('tuning-line').value || 'centerline'});
    if (state.id !== id || state.doc !== working || JSON.stringify(state.doc) !== before)
      throw Error('読み込み中に編集または地図選択が変わりました。編集中の内容を保護しています。地図を再読み込みしてください');
    state.doc = result.document; state.laneId = result.lane_id;
    state.target = null; state.selected = -1; state.diagnostic = null;
    state.undo = []; state.redo = [];
    restoreLineSettings(state.lane);
    $('tuning-line').value = result.line_type;
    $('tuning-line').dispatchEvent(new Event('change'));
    emit('document'); fit();
    await refreshMaps(); await refresh();
    if (state.id !== id) return;
    $('hdmap-version-select').value = version.id;
    $('tuning-hdmap-version').value = version.id;
    render();
    if (preview) {
      await prepareTuningPreview();
      document.querySelector('.tuning-panel').open = true;
      toast(`${version.name}をプレビューしました。「適用」で実車のHDMapを切り替えます`);
    } else toast(`${version.name}を作業用へ読み込みました。実車は未変更です。戻す場合は「直前の作業用HDMap」を選択してください`);
  } finally { busy = false; render(); }
}
action('hdmap-version-load', () => restore('hdmap-version-select', false));
action('hdmap-version-preview', () => restore('hdmap-version-select', true));
action('tuning-hdmap-preview', () => restore('tuning-hdmap-version', true));
for (const id of ['hdmap-version-select', 'tuning-hdmap-version']) $(id).addEventListener('change', render);
on('loaded', refresh);
on('document', () => { if (state.id !== mapId) return refresh(); render(); });
for (const event of ['status', 'ready', 'tuning']) on(event, render);
