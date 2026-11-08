import { $, api } from './api.js';

const series = [
  ['rssi', 'Bluetooth 相対RSSI', 'dB', '#9862bc'],
  ['hid', 'Bluetooth HID 最大読取間隔', 'ms', '#087b76'],
  ['joy', 'ROS Joy 最大受信間隔', 'ms', '#356bc2'],
  ['board', '基板 status 最大受信間隔', 'ms', '#b56022'],
];
const history = Object.fromEntries(series.map(([key]) => [key, []]));
let target, timer, generation = 0, latest = 0;

function draw() {
  if (!$('link-diagnostics').open) return;
  $('link-plots').innerHTML = series.map(([key, label, unit, color]) => {
    const points = history[key].filter(p => p.t >= latest - 120000);
    const values = points.map(p => p.v).filter(Number.isFinite);
    const lo = Math.min(0, ...values), hi = Math.max(lo + 1, 0, ...values);
    let path = '', dots = '', previous = null;
    for (const {t, v} of points) {
      if (!Number.isFinite(v)) { previous = null; continue; }
      const x = 45 + (t - latest + 120000) / 120000 * 540;
      const y = 53 - (v - lo) / (hi - lo) * 45;
      path += (previous !== null && t - previous < 10000 ? 'L' : 'M') + x + ' ' + y;
      dots += '<circle cx="'+x+'" cy="'+y+'" r="2"><title>'+v.toFixed(1)+' '+unit+'</title></circle>';
      previous = t;
    }
    const last = points.at(-1)?.v;
    return '<div><strong>'+label+'</strong> · '+(Number.isFinite(last)?last.toFixed(1)+' '+unit:'未取得')+
      '<svg role="img" aria-label="'+label+'" viewBox="0 0 600 72" style="display:block;width:100%;max-width:850px">'+
      '<path d="M45 8V53H585" fill="none" stroke="#999"/><g font-size="10" fill="currentColor">'+
      '<text x="1" y="13">'+hi.toFixed(1)+'</text><text x="1" y="55">'+lo.toFixed(1)+'</text>'+
      '<text x="45" y="69">−120秒</text><text x="585" y="69" text-anchor="end">最新計測</text></g>'+
      '<path fill="none" stroke="'+color+'" d="'+path+'"/><g fill="'+color+'">'+dots+'</g></svg></div>';
  }).join('');
}

function record(t, values) {
  if (!Number.isFinite(t)) return;
  latest = Math.max(latest, t);
  for (const [key, v] of Object.entries(values)) {
    const seq = history[key];
    seq.push({t,v});
    while (seq.length > 180 || seq[0]?.t < latest - 120000) seq.shift();
  }
  draw();
}

function missing(keys) {
  for (const key of keys) {
    const seq = history[key];
    if (Number.isFinite(seq.at(-1)?.v)) seq.push({t:latest, v:null});
  }
  draw();
}

export function plotBluetooth(data) {
  if (!data) { missing(['rssi', 'hid']); return; }
  record(data.measured_ms, {rssi:data.rssi, hid:data.max_gap_ms});
}

async function poll(id) {
  try {
    const data = await api('bringup/link_timing', target);
    if (id !== generation) return;
    record(data.measured_ms, {joy:data.joy.max_gap_ms, board:data.board.max_gap_ms});
    $('link-ros-state').textContent = [data.joy, data.board].map(s => s.topic+'（'+s.count+'件）').join(' ｜ ');
  } catch (e) {
    if (id === generation) {
      // Break the lines across failed queries without inventing a Jetson timestamp.
      missing(['joy','board']); $('link-ros-state').textContent = 'ROS計測失敗：'+e.message;
    }
  } finally {
    if (id === generation) timer = setTimeout(() => poll(id), 1500);
  }
}

function restart() {
  clearTimeout(timer); generation++;
  if (target && $('link-diagnostics').open) {
    $('link-ros-state').textContent = 'ROS計測中…';
    draw(); poll(generation);
  }
}
export function setPlotTarget(selected) {
  target = selected; latest = 0;
  for (const seq of Object.values(history)) seq.length = 0;
  $('link-plots').innerHTML = ''; $('link-ros-state').textContent = '';
  restart();
}
$('link-diagnostics').ontoggle = restart;
