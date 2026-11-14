import { $, on } from './api.js';
import { tuning } from './tuning.js';
import { lookaheadPlot, lookaheadSupported, lookaheadValue } from './lookahead_plot.js';
import { plotVisible } from './live_plots.js';

let history=null,received=0,epoch=tuning.epoch;
const series={
  lookahead_m:['tuning-lookahead-value','#2563eb'],
  target_distance_m:['target-distance-value','#d97706'],
  measured_speed_mps:['lookahead-measured-speed','#d97706'],
  target_speed_mps:['lookahead-target-speed','#2563eb'],
};
function chart(id,plot,keys,unit) {
  const grid=[];
  for(let i=0;i<=4;i++) {
    const value=plot.min+(plot.max-plot.min)*i/4,y=plot.y(value);
    grid.push(`<line x1="48" y1="${y}" x2="748" y2="${y}" stroke="#dbe3ed"/><text x="42" y="${y+4}" text-anchor="end">${value.toFixed(1)}</text>`);
  }
  for(const seconds of [60,45,30,15,0])grid.push(`<text x="${48+(60-seconds)/60*700}" y="160" text-anchor="${seconds===0?'end':'middle'}">${seconds===0?'現在':`−${seconds}s`}</text>`);
  $(id).innerHTML=`<g fill="#64748b" font-size="11">${grid.join('')}<text x="4" y="12">${unit}</text></g>`
    +keys.map(key=>`<path d="${plot.paths[key]}" fill="none" stroke="${series[key][1]}" stroke-width="2"/>`).join('');
}
function reset() {history=null;received=0;epoch=tuning.epoch;}
function render() {
  if(epoch!==tuning.epoch)reset();
  const panel=$('tuning-lookahead-panel');
  panel.hidden=!plotVisible('lookahead');
  if(panel.hidden)return;
  const connected=tuning.connected && Date.now()-tuning.received<3000;
  const now=history ? history.now+(performance.now()-received)/1000 : 0;
  const plot=lookaheadPlot(history?.samples || [],now),last=plot.last;
  chart('tuning-lookahead-chart',plot.distance,['lookahead_m','target_distance_m'],'m');
  chart('tuning-lookahead-speed-chart',plot.speed,['measured_speed_mps','target_speed_mps'],'m/s');
  const live=connected && last && now-last.t>=0 && now-last.t<2;
  for(const [key,[id]] of Object.entries(series)) {
    const value=live ? lookaheadValue(last,key) : null;
    $(id).textContent=value===null ? '—' : value.toFixed(2);
  }
  const supported=lookaheadSupported(last);
  $('tuning-lookahead-mode').textContent=!live || !supported ? '—'
    : last.controller==='mpc' ? 'MPC（予測終端直線距離）'
    : `Pure Pursuit（${last.lookahead_mode==='fixed'?'固定':'速度補間'}）`;
  const measured=lookaheadValue(last,'measured_speed_mps');
  const anyValue=Object.keys(series).some(key=>lookaheadValue(last,key)!==null);
  const semantics=last?.controller==='mpc'
    ? 'MPCの両距離は予測区間の終端参照点までの直線距離です。Pure Pursuitのlookaheadとは意味が異なります。'
    : last?.lookahead_mode==='fixed'
      ? '青は経路投影点からの経路長、橙は後輪軸から参照点への直線距離です。固定方式では速度をlookahead計算に使用しないため、推定速度は未計測です。'
      : '青は経路投影点からの経路長、橙は後輪軸から参照点への直線距離です。推定速度は制御が使用した後輪軸の前後VO速度で、独立した実測値ではありません。';
  const supportedHistory=history?.samples.some(lookaheadSupported);
  $('tuning-lookahead-note').textContent=!connected ? '未接続・受信停止。数値は失効しています。'
    : !history ? 'lookahead診断未対応・未受信（Jetson側の更新と横制御ノードの起動を確認）。'
    : !supportedHistory ? 'lookahead診断未対応・未受信、または横制御が非active・診断無効です。Jetson側の更新と横制御ノードの起動を確認してください。'
    : !live || !supported || !anyValue ? '横制御が非active・診断無効・受信停止。欠測区間は線をつなぎません。'
    : `${semantics} ${last.controller==='mpc'&&measured===null?'推定速度は未計測です。 ':''}約10 Hzの表示用サンプルをHTTPで約0.25〜1秒ごとに取得。平滑化なし。欠測・0.5秒以上の空白・方式変更では線をつなぎません。`;
}
function update() {
  if(epoch!==tuning.epoch)reset();
  const next=tuning.status?.lookahead_history;
  if(!next || !Number.isFinite(next.now) || !Array.isArray(next.samples))history=null;
  else if(next!==history) {
    // Each response is a full bounded snapshot. A new stream replaces every
    // old point, including when the diagnostic clock restarts at a lower time.
    if(next.stream!==history?.stream)reset();
    history=next;received=performance.now();
  }
  render();
}
on('tuning',update);
on('status',render);
on('plots-view',render);
on('loaded',()=>{reset();render();});
on('connection',()=>{reset();render();});
setInterval(render,250);
