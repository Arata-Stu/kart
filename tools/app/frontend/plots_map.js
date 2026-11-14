import { $, on, action, emit } from './api.js';
import { fit, neutral } from './canvas.js';

// Share the existing renderer and telemetry rather than opening another stream.
const map = $('map-canvas-wrap'), home = map.parentNode, next = map.nextSibling;
let recording = false;
function recordingView(enabled) {
  recording = enabled;
  document.body.classList.toggle('plots-recording', enabled);
  $('plots-fullscreen').textContent = enabled ? '通常表示へ戻る（Esc）' : '録画表示';
  $('plots-fullscreen').setAttribute('aria-pressed', String(enabled));
  requestAnimationFrame(() => { fit(); emit('view'); });
}
on('page', event => {
  const plots = event.detail === 'plots';
  if (plots) {
    neutral();
    $('plots-map-slot').append(map);
  } else {
    if (recording) recordingView(false);
    if (map.parentNode !== home) home.insertBefore(map, next);
  }
  if (plots || event.detail === 'map') requestAnimationFrame(() => { fit(); emit('view'); });
});
action('plots-map-fit', () => fit());
action('plots-fullscreen', () => recordingView(!recording));
document.addEventListener('keydown', event => {
  if (event.key === 'Escape' && recording) recordingView(false);
});
