import { $, on } from './api.js';
import { tuning } from './tuning.js';
import { speedPlot } from './speed_plot.js';
import { plotVisible } from './live_plots.js';

let history=null, received=0, epoch=tuning.epoch;
function render() {
  const panel=$('tuning-speed-panel');
  panel.hidden=!plotVisible('speed');
  if(panel.hidden)return;
  const connected=tuning.connected && Date.now()-tuning.received<3000;
  const now=history ? history.now+(performance.now()-received)/1000 : 0;
  const plot=speedPlot(history?.samples || [],now);
  const lines=[];
  for(let i=0;i<=4;i++) {
    const v=plot.min+(plot.max-plot.min)*i/4, y=plot.y(v);
    lines.push(`<line x1="48" y1="${y}" x2="748" y2="${y}" stroke="#dbe3ed"/><text x="42" y="${y+4}" text-anchor="end">${v.toFixed(1)}</text>`);
  }
  for(const seconds of [60,45,30,15,0]) {
    const x=48+(60-seconds)/60*700;
    lines.push(`<text x="${x}" y="160" text-anchor="${seconds===0?'end':'middle'}">${seconds===0?'現在':`−${seconds}s`}</text>`);
  }
  $('tuning-speed-chart').innerHTML=`<g fill="#64748b" font-size="11">${lines.join('')}<text x="4" y="12">m/s</text></g><path d="${plot.target}" fill="none" stroke="#2563eb" stroke-width="2"/><path d="${plot.measured}" fill="none" stroke="#d97706" stroke-width="2"/>`;
  const last=plot.last;
  const live=connected && last && now-last.t<2 && Number.isFinite(last.target) && Number.isFinite(last.measured);
  $('tuning-speed-target').textContent=live ? last.target.toFixed(2) : '—';
  $('tuning-speed-measured').textContent=live ? last.measured.toFixed(2) : '—';
  $('tuning-speed-note').textContent=!connected ? '未接続・受信停止' : !history ? '速度診断未対応・未受信（Jetson側の更新を確認）' : !live ? '速度制御が非active・診断無効・受信停止。欠測区間は線をつなぎません。' : 'PIDが使用する現在速度（後輪軸の前後速度）。約10 Hzの診断を約1秒ごとに取得。平滑化なし。';
}
function update() {
  if(epoch!==tuning.epoch){epoch=tuning.epoch;history=null;}
  const next=tuning.status?.speed_history;
  if(next && next!==history && Number.isFinite(next.now) && Array.isArray(next.samples)) {
    history=next;received=performance.now();
  }
  render();
}
on('tuning',update);
on('status',render);
on('plots-view',render);
on('loaded',()=>{history=null;render();});
on('connection',()=>{history=null;render();});
setInterval(render,250);
