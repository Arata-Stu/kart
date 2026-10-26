import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
const elements = new Map();
function make(id = '') {
  return { id, disabled: false, hidden: false, children: [], textContent: '', dataset: {},
    classList: { toggle() {} }, setAttribute(k, v) { this[k] = v; },
    addEventListener(k, fn) { this[k] = fn; }, append(...items) { this.children.push(...items); },
    replaceChildren(...items) { this.children = items; } };
}
function el(id) { if (!elements.has(id)) elements.set(id, make(id)); return elements.get(id); }
globalThis.document = { getElementById: el, createElement: () => make() };
globalThis.CustomEvent = class extends Event { constructor(name, { detail }) { super(name); this.detail = detail; } };
globalThis.setTimeout = () => 1; globalThis.clearTimeout = () => {};
const { state, emit } = await import('../frontend/api.js');
const flush = () => new Promise(resolve => setImmediate(resolve));
state.config = { connection: { user: 'kart', host: 'jetson', record_root: '/record' } };
const calls = [];
let rejectPull = false;
globalThis.fetch = async (url, options) => {
  const body = JSON.parse(options.body); calls.push({ url, body });
  let result;
  if (url === '/api/browse') result = { relative: body.relative, entries: body.relative
    ? [{ name: 'bag', relative: body.relative + '/bag', bag: true }]
    : ['2026-10-25', '2026-10-26'].map(name => ({ name, relative: name, bag: false })) };
  else if (url === '/api/pull') result = rejectPull ? { error: '同名のローカル記録があります' }
    : { jobs: body.relatives.map((path, i) => ({ id: String(i), title: path, status: 'queued' })) };
  else if (url === '/api/stop') result = { ...state.jobs.find(j => j.id === body.id), cancel_requested: true };
  else throw new Error(url);
  return { ok: !(url === '/api/pull' && rejectPull), json: async () => result };
};
// Only the unrelated map-building dialog is stubbed; transfer code/API/events are real.
let source = await readFile(new URL('../frontend/transfers.js', import.meta.url), 'utf8');
source = source.replace('"./api.js"', JSON.stringify(new URL('../frontend/api.js', import.meta.url).href))
  .replace('import { openBuild } from "./dialogs.js";', 'const openBuild = () => {};');
await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
function find(text, parent = el('remote-list')) {
  if (parent.textContent === text) return parent;
  for (const child of parent.children || []) { const found = find(text, child); if (found) return found; }
}
await el('browse-remote').click();
await find('▸ 2026-10-26').click(); await find('☐ bag').click();
await find('▸ 2026-10-25').click(); await find('☐ bag').click();
assert.equal(el('pull').textContent, '2件をキューに追加 ↓');
assert.equal(el('pull-selection').textContent, '1. 2026-10-26/bag\n2. 2026-10-25/bag');
rejectPull = true; await el('pull').click(); await flush();
assert.match(el('pull-selection').textContent, /2026-10-26/,'Failure retains selection');
rejectPull = false; await el('pull').click(); await flush();
assert.deepEqual(calls.filter(c => c.url === '/api/pull').at(-1).body.relatives,
  ['2026-10-26/bag', '2026-10-25/bag']);
assert.equal(el('pull').disabled, true, 'Empty selection stays disabled after action finally');
state.jobs = [{ id: 'one', title: '受信1', queue: 'scp', status: 'running', started: 1, message: '50%' },
  { id: 'two', title: '受信2', queue: 'scp', status: 'queued', started: 2 }];
emit('status'); await flush();
assert.match(el('transfer-queue-summary').textContent, /2件/);
assert.match(el('transfer-queue').children[0].children[0].textContent, /転送中/);
assert.match(el('transfer-queue').children[1].children[0].textContent, /待機中/);
const cancelButton = el('transfer-queue').children[1].children[1];
emit('status'); await flush();
assert.equal(el('transfer-queue').children[1].children[1], cancelButton, 'Polling preserves focus and stable controls');
await cancelButton.click();
assert.deepEqual(calls.at(-1).body, { id: 'two' });
assert.equal(el('transfer-queue').children[1].children[1].disabled, true);
await find('☐ bag').click(); emit('connection'); await flush();
assert.equal(el('pull').disabled, true, 'Changing connection clears selection');
console.log('Transfer UI: multi-selection across folders, FIFO request, failure retention, queue and cancellation passed');
