import { $, api } from './api.js';

import { plotBluetooth, setPlotTarget } from './link_plots.js';

let target = null;
let linkTimer;
let lastReport = null;
const timeLabel = ms => new Date(ms).toLocaleTimeString('ja-JP', {hour12:false, fractionalSecondDigits:3,
  hour:'2-digit', minute:'2-digit', second:'2-digit'});

async function pollLink(selected) {
  const started = performance.now();
  try {
    const data = await api('bringup/bluetooth_status', selected);
    if (target !== selected) return;
    plotBluetooth(data);
    const detail = [data.state];
    if (data.address) detail.push(data.address);
    detail.push('相対RSSI ' + (Number.isFinite(data.rssi) ? data.rssi + ' dB' : '取得不可'));
    if (lastReport && lastReport.address !== data.address) lastReport = null;
    if (Number.isFinite(data.last_report_ms)) {
      lastReport = {address:data.address, time:data.last_report_ms};
      detail.push('最終HID読取 ' + timeLabel(data.last_report_ms) +
        '（計測終了の' + Math.round(data.report_age_ms) + ' ms前）');
    } else if (data.reports === 0) {
      detail.push('直近の観測500 msで入力なし');
      if (lastReport) detail.push('前回までの最終観測 ' + timeLabel(lastReport.time));
    }
    if (Number.isFinite(data.max_gap_ms)) detail.push('観測内の最大間隔 ' + data.max_gap_ms.toFixed(1) + ' ms');
    if (Number.isFinite(data.measured_ms)) detail.push('Jetson計測 ' + timeLabel(data.measured_ms));
    detail.push('取得所要 ' + Math.round(performance.now() - started) + ' ms');
    $('dualsense-link').textContent = detail.join(' ｜ ');
  } catch (error) {
    if (target === selected) {
      plotBluetooth(null);
      $('dualsense-link').textContent = 'DualSense状態未確認：' + error.message;
    }
  } finally {
    if (target === selected) linkTimer = setTimeout(() => pollLink(selected), 1000);
  }
}
const actions = ['auto', 'manual', 'stop', 'status'];

function enable(enabled) {
  for (const action of actions) $('mode-' + action).disabled = !enabled;
}

export function setModeTarget(selected) {
  target = selected && structuredClone(selected);
  setPlotTarget(target);
  clearTimeout(linkTimer);
  lastReport = null;
  $('dualsense-link').textContent = target ? 'DualSense計測中…' : 'DualSense未取得';
  if (target) pollLink(target);
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
