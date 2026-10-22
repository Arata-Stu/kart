import assert from 'node:assert/strict';
const elements = new Map();
function el(id) {
  if (!elements.has(id)) elements.set(id, {
    children: [], disabled: false, value: '', textContent: '',
    addEventListener(event, fn) { this[event] = fn; },
    replaceChildren() { this.children = []; this.value = ''; },
    append(option) { this.children.push(option); if (this.children.length === 1) this.value = option.value; },
    showModal() { this.open = true; }, close() { this.open = false; },
  });
  return elements.get(id);
}
globalThis.document = { getElementById: el, createElement: () => ({}) };
globalThis.CustomEvent = class extends Event {
  constructor(name, {detail}) { super(name); this.detail = detail; }
};
let bundleOptions = [{path: '/maps/vgl/course-vgl-v1', input_shape: [1, 3, 480, 640],
  models: ['/models/640x480/runtime_models', '/models/640x480-v2/runtime_models'],
  default_model: '/models/640x480/runtime_models', error: ''}];
const calls = [];
globalThis.fetch = async (url, options) => {
  let result;
  if (url === '/api/records') result = [{id: 'recording'}];
  else if (url === '/api/vgl?id=course') result = {bundle_options: bundleOptions};
  else if (url === '/api/capture') {
    calls.push(JSON.parse(options.body)); result = {id: 'job'};
  } else throw Error(url);
  return {ok: true, json: async () => result};
};
const {state} = await import('../frontend/api.js');
state.id = 'course'; state.doc = {snapshot_status: 'pending', revision: 1};
await import('../frontend/capture.js');
await el('capture-open').click();
assert.equal(el('capture-start').disabled, false);
await el('capture-start').click();
assert.deepEqual(calls[0], {
  id: 'course', revision: 1, bag: 'recording',
  vgl_bundle: bundleOptions[0].path, model_dir: '/models/640x480/runtime_models',
});
assert.match(el('capture-info').textContent, /640 × 480/);
assert.match(el('capture-info').textContent, /models\/640x480\/runtime_models/);
el('capture-model').value = '/models/640x480-v2/runtime_models';
el('capture-model').change();
assert.equal(el('capture-start').disabled, false, 'Compatible alternative remains selectable');
bundleOptions.push({path: '/maps/vgl/course-vgl-v2', input_shape: [1, 3, 270, 480],
  models: ['/models/480x270/runtime_models'], default_model: '/models/480x270/runtime_models', error: ''});
await el('capture-open').click();
el('capture-vgl').value = bundleOptions[1].path;
el('capture-vgl').change();
assert.equal(el('capture-model').value, '/models/480x270/runtime_models');
assert.match(el('capture-info').textContent, /480 × 270/);
bundleOptions = [{path: '/maps/vgl/course-vgl-v3', input_shape: [1, 3, 720, 1280],
  models: [], default_model: '', error: '対応する実行用モデルがありません'}];
await el('capture-open').click();
assert.equal(el('capture-start').disabled, true);
assert.match(el('capture-info').textContent, /モデルがありません/);
bundleOptions = [];
await el('capture-open').click();
assert.equal(el('capture-start').disabled, true);
assert.match(el('capture-info').textContent, /先に/);
console.log('Capture UI: matching default, alternative selection, bundle switch, no-match gate');
