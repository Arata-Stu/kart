import assert from 'node:assert/strict';
const elements = new Map();
globalThis.document = {getElementById(id) {
  if (!elements.has(id)) elements.set(id, {});
  return elements.get(id);
}};
const {setPlotTarget, plotBluetooth} = await import('../frontend/link_plots.js');
const el = document.getElementById;
el('link-diagnostics').open = true;
globalThis.fetch = async () => ({ok:true,json:async()=>({
  measured_ms:1000, joy:{max_gap_ms:12,count:40,topic:'/joy'},
  board:{max_gap_ms:55,count:10,topic:'/vehicle/output_channels'},
})});
setPlotTarget({container:'test'});
await new Promise(resolve=>setImmediate(resolve));
assert.match(el('link-plots').innerHTML,/55.0 ms/);
plotBluetooth({measured_ms:1200,rssi:0,max_gap_ms:6});
plotBluetooth({measured_ms:3000,rssi:-3,max_gap_ms:8});
assert.match(el('link-plots').innerHTML,/-3.0 dB/);
assert.match(el('link-plots').innerHTML,/d="M[0-9.]+ [0-9.]+L/);
plotBluetooth(null);
assert.match(el('link-plots').innerHTML,/相対RSSI<\/strong> · 未取得/);
setPlotTarget(null);
assert.equal(el('link-plots').innerHTML,'');
