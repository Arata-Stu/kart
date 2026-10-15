"""Self-contained offline HTML reports; all user text is escaped."""
import csv
import html
import json
import math
from pathlib import Path


def clean(value):
    if isinstance(value, dict): return {k: clean(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value): return None
    if hasattr(value, 'item'): return clean(value.item())
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(clean(value), ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def write_csv(path, rows):
    with Path(path).open('w', newline='') as stream:
        if rows:
            writer=csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(clean(rows))


def table(rows):
    if not rows: return '<p>該当データなし</p>'
    keys=list(rows[0])
    def cell(v):
        return html.escape(f'{v:.5g}' if isinstance(v,float) else str(v) if v is not None else '—')
    return '<div class="scroll"><table><thead><tr>'+''.join('<th>'+cell(k)+'</th>' for k in keys)+'</tr></thead><tbody>'+''.join(
        '<tr>'+''.join('<td>'+cell(r.get(k))+'</td>' for k in keys)+'</tr>' for r in rows)+'</tbody></table></div>'


def page(path, title, body, rows, panels, speed_note='速度はVSLAM推定値です。'):
    # Embed JSON as inert script data. Escaping '<' prevents HTML/script termination.
    payload=json.dumps(clean({'rows':rows,'panels':panels}),ensure_ascii=False,allow_nan=False).replace('<','\\u003c')
    content='''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>'''+html.escape(title)+'''</title><style>
body{font:15px system-ui,sans-serif;color:#183044;background:#f4f7fa;margin:0;padding:24px;max-width:1200px;margin:auto}
h1{font-size:25px}h2{font-size:19px;margin-top:28px}p{line-height:1.7}.card{background:white;border:1px solid #d6e1e8;border-radius:10px;padding:18px;margin:16px 0}
canvas{display:block;width:100%;height:220px}table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:7px;border-bottom:1px solid #ddd;text-align:left;white-space:nowrap}.scroll{overflow:auto}pre{white-space:pre-wrap;overflow-wrap:anywhere}a{color:#075ea8}input[type=range]{width:100%}.notice{border-left:4px solid #d18a00;padding-left:14px}#cursor{font-family:monospace;white-space:pre-wrap}
</style><h1>'''+html.escape(title)+'''</h1>'''+body+'''
<div class="card"><p>グラフ上にポインターを置くか、スライダーで時刻別の値を確認できます。'''+html.escape(speed_note)+'''</p><input id="time" type="range" min="0" value="0"><div id="cursor"></div></div><div id="plots"></div>
<script id="data" type="application/json">'''+payload+'''</script><script>
const {rows,panels}=JSON.parse(document.getElementById('data').textContent);
const slider=document.getElementById('time'),cursor=document.getElementById('cursor'),plots=document.getElementById('plots');
const colors=['#1668b2','#d05e26','#16866c','#9554a2']; slider.max=Math.max(0,rows.length-1);
const canvases=panels.map(([title,keys])=>{const box=document.createElement('div');box.className='card';const h=document.createElement('h2');h.textContent=title;box.append(h);const legend=document.createElement('p');keys.forEach((k,j)=>{const span=document.createElement('span');span.style.color=colors[j%4];span.textContent=k+'  ';legend.append(span)});box.append(legend);const c=document.createElement('canvas');box.append(c);plots.append(box);c.onpointermove=e=>{if(!rows.length)return;const r=c.getBoundingClientRect();const f=Math.max(0,Math.min(1,(e.clientX-r.left-55)/(r.width-75)));const t=rows[0].t+f*(rows.at(-1).t-rows[0].t);let lo=0,hi=rows.length-1;while(lo<hi){const m=(lo+hi)>>1;if(rows[m].t<t)lo=m+1;else hi=m}slider.value=lo;draw()};return [c,keys]});
function draw(){if(!rows.length){cursor.textContent='有効な時系列がありません';return}const index=Number(slider.value),current=rows[index];cursor.textContent=Object.entries(current).map(([k,v])=>k+': '+(Number.isFinite(v)?v.toFixed(4):v===null?'—':v)).join(' | ');for(const [c,keys] of canvases){const w=c.clientWidth,h=220,dpr=window.devicePixelRatio||1;c.width=w*dpr;c.height=h*dpr;const ctx=c.getContext('2d');ctx.scale(dpr,dpr);const values=rows.flatMap(r=>keys.map(k=>r[k]).filter(Number.isFinite));let low=values.reduce((a,b)=>Math.min(a,b),0),high=values.reduce((a,b)=>Math.max(a,b),1);const range=high-low;low-=range*.05;high+=range*.05;const t0=rows[0].t,t1=rows.at(-1).t;const x=t=>55+(t-t0)/Math.max(.001,t1-t0)*(w-75),y=v=>190-(v-low)/(high-low)*170;ctx.font='12px system-ui';ctx.fillStyle='#455';ctx.strokeStyle='#dce3e8';for(let j=0;j<5;j++){const v=low+(high-low)*j/4;ctx.beginPath();ctx.moveTo(55,y(v));ctx.lineTo(w-20,y(v));ctx.stroke();ctx.fillText(v.toFixed(2),2,y(v)+4)}ctx.fillText(t0.toFixed(1)+' s',55,212);ctx.fillText(t1.toFixed(1)+' s',w-70,212);keys.forEach((k,j)=>{ctx.strokeStyle=colors[j%4];ctx.beginPath();let active=false,prev=null;for(const r of rows){if(!Number.isFinite(r[k])){active=false;continue}if(prev!==null&&r.t-prev>.2)active=false;if(active)ctx.lineTo(x(r.t),y(r[k]));else ctx.moveTo(x(r.t),y(r[k]));active=true;prev=r.t}ctx.stroke()});ctx.strokeStyle='#667';ctx.beginPath();ctx.moveTo(x(current.t),15);ctx.lineTo(x(current.t),190);ctx.stroke()}}
slider.oninput=draw;window.onresize=draw;draw();</script></html>'''
    Path(path).write_text(content)


def analysis_report(output, summary, rows, segments, bins):
    output=Path(output)
    write_json(output/'summary.json',summary)
    write_json(output/'model.json',summary['model'])
    write_csv(output/'samples.csv',rows);write_csv(output/'segments.csv',segments);write_csv(output/'characteristics.csv',bins)
    body='<p class="notice">実機設定は変更していません。モデル・候補値は直線試験用の概算です。絶対速度・制動性能の保証ではありません。</p>'
    body+='<p>入力: '+html.escape(str(summary['source']))+'</p>'
    body+='<p>'+f"全{summary['samples']}点 / 解析採用{summary['selected_samples']}点 / {summary['segments']}区間"+'</p>'
    body+='<p><a href="summary.json">全設定・品質・結果 JSON</a> · <a href="samples.csv">時系列 CSV</a> · <a href="segments.csv">区間 CSV</a> · <a href="characteristics.csv">特性表 CSV</a></p>'
    kff=summary['kff_candidate'];model=summary['model']
    body+='<p><strong>kff候補: '+(f"{kff['value']:.4g}" if kff else 'データ不足')+' ／ モデル試算: '+('利用可能（別区間検証済み）' if model['usable'] else '利用不可')+'</strong></p>'
    body+='<details><summary>候補値・モデル・検証誤差の詳細</summary><pre>'+html.escape(json.dumps({'kff':kff,'model':model},ensure_ascii=False,indent=2))+'</pre></details>'
    body+='<details><summary>記録品質・除外理由・採用区間</summary><h2>記録品質</h2>'+table([{'stream':k,**v} for k,v in summary['quality'].items()])
    body+='<h2>除外理由（重複あり）</h2>'+table([{'reason':k,'count':v} for k,v in summary['excluded_reasons_overlapping'].items()])
    body+='<h2>採用区間</h2>'+table(segments)+'</details><details><summary>解釈の範囲</summary><ul>'+''.join('<li>'+html.escape(x)+'</li>' for x in summary['warnings'])+'</ul></details>'
    page(output/'report.html','kart 車両特性・速度制御解析',body,rows,[
        ('速度 [m/s]', ['speed_mps','smooth_speed_mps','target_mps']),
        ('操作指令 [正規化]', ['throttle','brake','steering']),
        ('VSLAM速度からの加速度 [m/s²]', ['acceleration_mps2']),
        ('IMU加速度ノルム [m/s²、重力込み]', ['imu_accel_norm_mps2'])])
