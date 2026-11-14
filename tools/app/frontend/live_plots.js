import { $, state, action, on, emit } from './api.js';
import { tuning, toggleTuningConnection } from './tuning.js';

const kinds=['all','lookahead','speed','tracking-timing','speed-timing'];
let selected='all';

export function plotVisible(kind) {
  return !!$('plots-page') && !$('plots-page').hidden
    && (selected==='all' || selected===kind);
}

function render() {
  const hasMap=!!state.id && !!state.doc;
  const target=state.config?.connection;
  const address=target ? `${target.user}@${target.host}:${target.port}` : '';
  const connected=tuning.connected && tuning.id===state.id;
  const fresh=connected && Date.now()-tuning.received<3000;
  $('plots-map-name').textContent=hasMap ? state.doc.title || state.doc.name || state.id : '未選択';
  $('plots-open-map').textContent=hasMap ? '地図を開く' : '地図を選ぶ';
  $('plots-connect').disabled=tuning.busy || !hasMap;
  $('plots-connect').textContent=connected ? '切断' : 'Jetsonに接続';
  $('plots-connection-status').textContent=!hasMap
    ? '地図未選択。「地図を選ぶ」から対象の地図を選択してください。'
    : tuning.error ? `接続エラー: ${tuning.error}`
    : tuning.busy && !connected ? `Jetsonの状態へ接続中${address ? ` · ${address}` : ''}`
    : fresh ? `接続中${address ? ` · ${address}` : ''} · 地図画面と同じ診断を表示中`
    : connected ? '受信停止。最新値は失効しています。接続先とJetsonの起動状態を確認してください。'
    : `未接続${address ? ` · ${address}` : ''}。Jetsonに接続するとライブプロットを表示します。`;
  for(const kind of kinds) {
    const button=$('plots-filter-'+kind);
    button.classList.toggle('active',selected===kind);
    button.setAttribute('aria-pressed',String(selected===kind));
  }
}

action('plots-connect',toggleTuningConnection);
action('plots-open-map',()=> $('nav-map').click());
for(const kind of kinds)$('plots-filter-'+kind).addEventListener('click',()=>{
  selected=kind;
  render();
  emit('plots-view');
});
for(const event of ['tuning','ready','status','document'])on(event,render);
for(const event of ['page','loaded','connection'])on(event,()=>{
  render();
  emit('plots-view');
});
setInterval(render,250);
render();
