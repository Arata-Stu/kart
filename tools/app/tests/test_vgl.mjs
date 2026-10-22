import assert from 'node:assert/strict';
const elements = new Map();
function el(id) {
  if (!elements.has(id)) elements.set(id, {
    children: [], disabled: false, value: '', textContent: '', readOnly: false,
    addEventListener(event, fn) { this[event] = fn; },
    replaceChildren() { this.children = []; this.value = ''; },
    append(option) { this.children.push(option); if (this.children.length === 1) this.value = option.value; },
    showModal() { this.open = true; }, close() { this.open = false; },
  });
  return elements.get(id);
}
globalThis.document = {getElementById: el, createElement: () => ({})};
globalThis.CustomEvent = class extends Event {
  constructor(name, {detail}) { super(name); this.detail = detail; }
};
const paths = ['1280x720', '424x240', '640x480', '848x480', '640x360', '480x270'].map(s => `/models/${s}/runtime_models`);
const models = paths.map(path => {
  const [width, height] = path.split('/')[2].split('x').map(Number);
  return {path, width, height, error: ''};
});
const calls = [];
const data = {model_options: models, defaults: {width: 424, height: 240},
  name: 'course-vgl-v1', bundles: [], error: ''};
globalThis.fetch = async (url, options) => {
  let result;
  if (url === '/api/vgl?id=course') result = data;
  else if (url === '/api/vgl') { calls.push(JSON.parse(options.body)); result = {id: 'job'}; }
  else throw Error(url);
  return {ok: true, json: async () => result};
};
const {state} = await import('../frontend/api.js');
state.id = 'course';
await import('../frontend/vgl.js');
await el('vgl-open').click();
assert.equal(el('vgl-model').value, paths[1], 'Default must follow 424×240, not alphabetic first model');
assert.equal(el('vgl-model').readOnly, true);
for (const resolution of ['424x240', '480x270', '640x360', '640x480', '848x480', '1280x720']) {
  el('vgl-model-choice').value = `/models/${resolution}/runtime_models`;
  el('vgl-model-choice').change();
  const [width, height] = resolution.split('x').map(Number);
  assert.equal(Number(el('vgl-width').value), width);
  assert.equal(Number(el('vgl-height').value), height);
  assert.equal(el('vgl-width').readOnly, true);
  assert.equal(el('vgl-height').readOnly, true);
  assert.equal(el('vgl-model').value, `/models/${resolution}/runtime_models`);
  assert.match(el('vgl-selection').textContent, new RegExp(`/models/${resolution}/runtime_models`));
  assert.equal(el('vgl-start').disabled, false);
}
await el('vgl-start').click();
assert.deepEqual(calls[0], {id: 'course', name: 'course-vgl-v1',
  model_dir: paths[0]});
data.model_options = models.filter(m => m.width !== 424);
await el('vgl-open').click();
assert.equal(el('vgl-model').value, '');
assert.equal(el('vgl-start').disabled, true, 'Do not fall back to a different resolution');
assert.match(el('vgl-selection').textContent, /選択/);
el('vgl-model-choice').value = 'custom';
el('vgl-model-choice').change();
assert.equal(el('vgl-width').readOnly, false);
el('vgl-width').value = 640;
el('vgl-width').input();
assert.equal(el('vgl-start').disabled, true, 'Reject mismatched manual sizes');
el('vgl-model').value = '/external/legacy-model';
el('vgl-width').value = 640; el('vgl-height').value = 480;
el('vgl-model').input();
assert.equal(el('vgl-start').disabled, false);
assert.match(el('vgl-selection').textContent, /解像度未確認/);
console.log('VGL UI: model selection determines size, readonly known size, backend derives dimensions');
