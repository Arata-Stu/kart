import { $, on } from './api.js';
import { tuning } from './tuning.js';
import { timingPlot } from './timing_plot.js';

let histories=null, received=0, epoch=tuning.epoch;
const series={
  processing_ms:['処理時間（青）','#2563eb','plot-blue'],
  pose_age_ms:['自己位置の古さ（橙）','#d97706','plot-orange'],
  target_age_ms:['目標指令の古さ（青）','#2563eb','plot-blue'],
  measurement_age_ms:['速度計測の古さ（橙）','#d97706','plot-orange'],
};
function chart(id, history, keys, connected) {
  const now=history ? history.now+(performance.now()-received)/1000 : 0;
  const plot=timingPlot(history?.samples || [],now,keys), grid=[];
  for(let i=0;i<=4;i++) {
    const v=plot.min+(plot.max-plot.min)*i/4,y=plot.y(v);
    grid.push(`<line x1="48" y1="${y}" x2="748" y2="${y}" stroke="#dbe3ed"/><text x="42" y="${y+4}" text-anchor="end">${v.toFixed(1)}</text>`);
  }
  for(const seconds of [60,45,30,15,0])grid.push(`<text x="${48+(60-seconds)/60*700}" y="160" text-anchor="${seconds===0?'end':'middle'}">${seconds===0?'現在':`−${seconds}s`}</text>`);
  $(id+'-chart').innerHTML=`<g fill="#64748b" font-size="11">${grid.join('')}<text x="4" y="12">ms</text></g>`+keys.map(k=>`<path d="${plot.paths[k]}" fill="none" stroke="${series[k][1]}" stroke-width="2"/>`).join('');
  const live=connected&&plot.last&&now-plot.last.t<.5;
  $(id+'-legend').replaceChildren(...keys.map(key=>{
    const span=document.createElement('span');span.className='plot-key '+series[key][2];
    span.textContent=`${series[key][0]} ${live&&Number.isFinite(plot.last[key])?plot.last[key].toFixed(2):'—'} ms`;
    return span;
  }));
}
function render() {
  const target=$('tuning-parameters-target').value,panel=$('tuning-timing-panel');
  panel.hidden=!['tracking','speed'].includes(target);
  if(panel.hidden||!$('tuning-parameters-dialog').open)return;
  const connected=tuning.connected&&Date.now()-tuning.received<3000;
  const history=histories?.[target];
  chart('tuning-processing',history,['processing_ms'],connected);
  chart('tuning-age',history,target==='tracking'?['pose_age_ms']:['target_age_ms','measurement_age_ms'],connected);
  $('tuning-timing-status').textContent=!connected?'未接続・受信停止':!history?.samples.length?'診断未受信・未対応（Jetson側の更新とノード起動を確認）':'約10 Hzの表示用サンプル。未計測・無効・欠測区間は線をつなぎません。全制御周期のピークは保証しません。';
}
function update() {
  if(epoch!==tuning.epoch){epoch=tuning.epoch;histories=null;}
  const next=tuning.status?.timing_history;
  if(next&&next!==histories){histories=next;received=performance.now();}
  render();
}
on('tuning',update);
on('status',render);
on('loaded',()=>{histories=null;render();});
on('connection',()=>{histories=null;render();});
$('tuning-parameters-target').addEventListener('change',render);
$('tuning-parameters-open').addEventListener('click',render);
setInterval(render,250);
