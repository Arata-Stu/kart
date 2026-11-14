import assert from 'node:assert/strict';
const elements = new Map();
globalThis.document = {getElementById(id) {
  if (!elements.has(id)) elements.set(id, {value:''});
  return elements.get(id);
}};
const {state} = await import('../frontend/api.js');
const {restoreLineSettings} = await import('../frontend/line_settings.js');
restoreLineSettings({lines:{centerline:{settings:{centerline_method:'paired',centerline_max_points:600}},
  raceline:{settings:{optimizer:'local'}}}});
assert.equal(state.settings.centerline_method, 'paired');
assert.equal(state.settings.centerline_max_points, 600);
assert.equal(state.settings.optimizer, 'local');
restoreLineSettings({lines:{}});
assert.equal(state.settings.centerline_method, 'region');
assert.equal(state.settings.centerline_max_points, 800);
assert.equal(state.settings.optimizer, 'mincurv');
assert.equal(elements.get('centerline-method').value, 'region');
assert.equal(elements.get('raceline-method').value, 'mincurv');
console.log('Saved settings survive; new maps and lanes use region/800/mincurv');
