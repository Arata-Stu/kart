import { $, api } from './api.js';

let target = null;
const actions = ['auto', 'manual', 'stop', 'status'];

function enable(enabled) {
  for (const action of actions) $('mode-' + action).disabled = !enabled;
}

export function setModeTarget(selected) {
  target = selected && structuredClone(selected);
  enable(Boolean(target));
  $('mode-feedback').textContent = target ? '現在のモードは未取得です。' : 'JetsonとDockerを選択してください。';
}

async function send(action) {
  const selected = target;
  if (!selected || $('mode-' + action).disabled) return;
  enable(false);
  $('mode-feedback').textContent = '車両の応答待ち…';
  try {
    const result = await api('bringup/mode_' + action, selected);
    if (selected === target) {
      $('mode-feedback').textContent = new Date().toLocaleTimeString() + ' 取得時点 ' + result.mode + '（' + result.source + '）';
    }
  } catch (error) {
    if (selected === target) $('mode-feedback').textContent = error.message;
  } finally {
    if (selected === target) enable(true);
  }
}

for (const action of actions) $('mode-' + action).onclick = () => send(action);
