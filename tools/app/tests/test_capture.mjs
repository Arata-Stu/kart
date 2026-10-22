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
let bundles = ['/maps/vgl/course-vgl-v1'];
const calls = [];
globalThis.fetch = async (url, options) => {
  let result;
  if (url === '/api/records') result = [{id: 'recording'}];
  else if (url === '/api/vgl?id=course') result = {bundles, models: ['/models/640x480/runtime_models']};
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
  vgl_bundle: bundles[0], model_dir: '/models/640x480/runtime_models',
});
bundles = [];
await el('capture-open').click();
assert.equal(el('capture-start').disabled, true);
assert.match(el('capture-info').textContent, /先に/);
console.log('Capture UI: VGL selection submitted; missing bundle blocks start');
