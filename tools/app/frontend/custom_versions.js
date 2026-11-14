import { $, state, action, remember, on, emit, toast } from './api.js';
import { save } from './maps.js';
import { showLine } from './line_visibility.js';
import { tuning, prepareTuningPreview } from './tuning.js';
import { snapshotCustom, loadCustomVersion, nextVersionId } from './custom_versions_model.js';

function options(id, versions, placeholder) {
  const select = $(id), selected = select.value;
  const rows = [['', placeholder], ...versions.map(v => [v.id, `${v.name}${v.line ? ' · 生成済み' : ' · 要生成'}`])];
  if (JSON.stringify([...select.options].map(o => [o.value, o.textContent])) === JSON.stringify(rows)) return;
  select.replaceChildren();
  for (const [value, label] of rows) {
    const option = document.createElement('option');
    option.value = value; option.textContent = label; select.append(option);
  }
  select.value = versions.some(v => v.id === selected) ? selected : '';
}
function render() {
  const lane = state.lane, versions = lane?.custom_versions || [];
  const busy = state.jobs.some(j => j.status === 'running' && (j.title.includes(state.id) || j.title.includes('生成')));
  const unavailable = !lane || state.doc?.snapshot_status === 'pending' || busy;
  options('custom-version-select', versions, '保存した版を選択');
  options('tuning-custom-version', versions, 'Customの戦略を選択');
  for (const id of ['custom-version-select', 'custom-version-name', 'custom-version-load', 'custom-version-update', 'custom-version-delete', 'custom-version-new']) $(id).disabled = unavailable;
  const selected = versions.find(v => v.id === $('custom-version-select').value);
  for (const id of ['custom-version-load', 'custom-version-update', 'custom-version-delete']) $(id).disabled ||= !selected;
  $('custom-version-new').disabled ||= versions.length >= 32 || !lane?.custom.length;
  $('custom-version-status').textContent = `${versions.length}/32 版 · 編集中: ${versions.find(v => v.id === lane?.custom_version)?.name || '作業用Custom'}`;
  $('tuning-custom-version').disabled = unavailable || tuning.busy;
  const candidate = versions.find(v => v.id === $('tuning-custom-version').value);
  $('tuning-custom-preview').disabled = unavailable || tuning.busy || state.dirty || !candidate?.line;
}
function changed() {
  state.selected = -1; state.diagnostic = null; state.dirty = true;
  emit('document');
}
for (const id of ['custom-version-select', 'tuning-custom-version']) $(id).addEventListener('change', render);
action('custom-version-new', () => {
  const lane = state.lane, name = $('custom-version-name').value.trim();
  if (!lane?.custom.length) throw Error('先にCustomの点を作成してください');
  if ((lane.custom_versions?.length || 0) >= 32) throw Error('Customの版は32個までです');
  if (lane.custom_versions?.some(v => v.name.toLocaleLowerCase() === name.toLocaleLowerCase())) throw Error('同じ版名があります。別名にするか「選択版を更新」を使ってください');
  const version = snapshotCustom(lane, nextVersionId(lane), name);
  remember(); (lane.custom_versions ||= []).push(version); lane.custom_version = version.id;
  changed(); render(); $('custom-version-select').value = version.id; render();
  toast('版を追加しました。地図の保存で確定します');
});
action('custom-version-update', () => {
  const lane = state.lane, index = lane.custom_versions.findIndex(v => v.id === $('custom-version-select').value);
  if (index < 0) throw Error('更新する版を選択してください');
  const old = lane.custom_versions[index], version = snapshotCustom(lane, old.id, old.name);
  remember(); lane.custom_versions[index] = version; lane.custom_version = old.id; changed();
  toast('選択版を現在の編集内容で更新しました。地図の保存で確定します');
});
action('custom-version-load', () => {
  const id = $('custom-version-select').value;
  if (!state.lane.custom_versions?.some(v => v.id === id)) throw Error('読み込む版を選択してください');
  remember(); const version = loadCustomVersion(state.lane, id);
  state.target = null; showLine('customline'); changed();
  if (version.line?.settings) Object.assign(state.settings, version.line.settings);
  toast(`${version.name}を読み込みました（Undo可）。実車は「プレビュー→適用」で切り替えます`);
});
action('custom-version-delete', () => {
  const lane = state.lane, id = $('custom-version-select').value;
  remember(); lane.custom_versions = lane.custom_versions.filter(v => v.id !== id);
  if (lane.custom_version === id) lane.custom_version = '';
  changed(); toast('版を削除しました。作業中のCustomは保持します（Undo可）');
});
action('tuning-custom-preview', async () => {
  if (state.dirty) throw Error('先に地図の編集を保存してください');
  const id = $('tuning-custom-version').value;
  const mapId = state.id, laneId = state.laneId;
  const version = state.lane?.custom_versions?.find(v => v.id === id);
  if (!version?.line) throw Error('選んだ版を読み込み、Customを生成・保存してください');
  remember(); loadCustomVersion(state.lane, id); state.target = null; showLine('customline'); changed();
  await save();
  if (state.id !== mapId || state.laneId !== laneId || state.dirty) throw Error('地図またはlaneが変わりました。戦略を選び直してください');
  $('tuning-line').value = 'customline'; $('tuning-line').dispatchEvent(new Event('change'));
  await prepareTuningPreview();
  toast(`${version.name}をプレビューしました。「適用」で実車へ反映します`);
});
for (const event of ['document', 'status', 'ready', 'tuning']) on(event, render);
