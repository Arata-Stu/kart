import { $, api, action } from './api.js';
let request = 0, selected = null, selectedKind = "runs";
const ns = 'http://www.w3.org/2000/svg';
function svgNode(name, attributes, text) {
  const node = document.createElementNS(ns,name);
  for (const [key,value] of Object.entries(attributes)) node.setAttribute(key,value);
  if (text !== undefined) node.textContent = text;
  return node;
}
export async function showLoss(id, kind="runs") {
  const token = ++request;
  const report = await api('e2e/loss?id='+encodeURIComponent(id)+'&kind='+encodeURIComponent(kind));
  if (token !== request) return;
  selected = id; selectedKind = kind;
  const distill = report.metric === 'distillation_loss';
  const hasLoss = !distill && report.metrics.every(row => Number.isFinite(row.train_loss) && Number.isFinite(row.validation_loss));
  $('e2e-loss-metric-label').hidden = !hasLoss;
  const objective = hasLoss && $('e2e-loss-metric').value === 'loss';
  const label = distill ? '特徴蒸留loss' : objective ? 'loss' : 'MSE';
  $('e2e-loss-train-label').textContent = '● 学習' + label;
  $('e2e-loss-val-label').textContent = '● 検証' + label;
  $('e2e-loss-train-heading').textContent = '学習' + label;
  $('e2e-loss-val-heading').textContent = '検証' + label;
  $('e2e-loss-export').textContent = distill ? 'このTiny encoderで制御学習へ' : 'この実験のONNX exportへ';
  $('e2e-loss-note').textContent = distill
    ? '特徴蒸留lossはMSEとcosine距離の重み付き和。最良epochのencoder.ptを保存。操舵誤差とは異なります。'
    : (objective ? '学習で使用した損失。スケジューラは検証lossを監視します。' : '比較用の平均二乗誤差（MSE）。') + '平滑化なし。best.ptは検証MSE最小で選定。破線・★は表示指標の最小epochです。学習時と検証時はDropout等の動作が異なります。';
  const rows = report.metrics.map(row => (distill || objective) ? {...row, train_mse:row.train_loss, validation_mse:row.validation_loss} : row);
  const best = rows.reduce((a,b)=>b.validation_mse<a.validation_mse?b:a);
  const last = rows.at(-1);
  $('e2e-loss-title').textContent = '損失カーブ · ' + id;
  $('e2e-loss-summary').textContent = `最良 epoch ${best.epoch}：検証${label} ${best.validation_mse.toPrecision(5)} ／ 最終 epoch ${last.epoch}：学習 ${last.train_mse.toPrecision(5)}・検証 ${last.validation_mse.toPrecision(5)} ／ 最良からの検証損失変化 ${(last.validation_mse-best.validation_mse).toPrecision(3)}`;
  const chart = $('e2e-loss-chart'); chart.replaceChildren();
  const width=880,height=300,left=82,right=22,top=24,bottom=46;
  const max = rows.reduce((value,row)=>Math.max(value,row.train_mse,row.validation_mse),0) || 1;
  const upper = max*1.08;
  const first=rows[0].epoch, final=last.epoch;
  const x = epoch => left + (width-left-right)*(final===first?.5:(epoch-first)/(final-first));
  const y = value => height-bottom-(height-top-bottom)*value/upper;
  for (let i=0;i<=4;i++) {
    const value=upper*i/4, position=y(value);
    chart.append(svgNode('line',{x1:left,x2:width-right,y1:position,y2:position,stroke:'#d9e1e7'}));
    chart.append(svgNode('text',{x:left-10,y:position+4,'text-anchor':'end','font-size':13,fill:'#4f606e'},value.toPrecision(3)));
  }
  const ticks=[...new Set([first,Math.round((first+final)/2),final])];
  for (const epoch of ticks) chart.append(svgNode('text',{x:x(epoch),y:height-24,'text-anchor':'middle','font-size':13},epoch));
  chart.append(svgNode('text',{x:width/2,y:height-4,'text-anchor':'middle','font-size':14},'Epoch'));
  chart.append(svgNode('text',{x:12,y:15,'font-size':14},label));
  chart.append(svgNode('line',{x1:x(best.epoch),x2:x(best.epoch),y1:top,y2:height-bottom,stroke:'#9aa9b5','stroke-dasharray':'4 4'}));
  for (const [key,label,color] of [['train_mse','学習','#177f86'],['validation_mse','検証','#a04fa5']]) {
    chart.append(svgNode('polyline',{points:rows.map(row=>`${x(row.epoch)},${y(row[key])}`).join(' '),fill:'none',stroke:color,'stroke-width':2.5}));
    for (const row of (rows.length<=200 ? rows : [best])) {
      const point=svgNode('circle',{cx:x(row.epoch),cy:y(row[key]),r:3,fill:color});
      point.append(svgNode('title',{},`epoch ${row.epoch} · ${label} loss ${row[key]}`)); chart.append(point);
    }
  }
  const body=$('e2e-loss-rows'); body.replaceChildren();
  for (const row of rows) {
    const tr=document.createElement('tr');
    for (const value of [row.epoch+(row===best?' ★':''),row.train_mse.toPrecision(6),row.validation_mse.toPrecision(6),Number.isFinite(row.learning_rate)?row.learning_rate.toExponential(3):'—',Number.isFinite(row.next_learning_rate)?row.next_learning_rate.toExponential(3):'—']) {
      const td=document.createElement('td'); td.textContent=value;tr.append(td);
    }
    body.append(tr);
  }
  if (!$('e2e-loss-dialog').open) $('e2e-loss-dialog').showModal();
}
action('e2e-loss-close',()=>{request++;$('e2e-loss-dialog').close();});
action('e2e-loss-export',()=>{
  if (selectedKind === 'encoders') {
    $('e2e-initialization').value='pretrained';
    $('e2e-architecture').value='dinov3_vitt16';
    $('e2e-architecture').dispatchEvent(new Event('change',{bubbles:true}));
    $('e2e-tiny-encoder').value=selected;
    $('e2e-loss-dialog').close(); $('e2e-tab-train').click(); return;
  }
  $('e2e-export-run').value=selected;
  $('e2e-loss-dialog').close();
  $('e2e-export-run').dispatchEvent(new Event('change',{bubbles:true}));
  $('e2e-tab-export').click();
});

$('e2e-loss-metric').addEventListener('change',()=>{ if(selected) showLoss(selected,selectedKind); });
