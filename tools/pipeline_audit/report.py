"""Standalone Plotly time series with shared zoom; full-resolution CSV is kept."""
import csv
import html
import json
from statistics import median
from pathlib import Path

PANELS = {
    'response_steering': '操舵指令 ↔ IMU角速度',
    'response_drive': '駆動・制動指令 ↔ IMU加速度',
    'age': 'Header → bag recorder age [ms] (clock offset / queueing included)',
    'evs_clock': 'EVS clock progress / offset change [ms] relative to segment start (not absolute physical lag)',
    'matched': 'Exact same frame: first recorded observation gaps [ms]',
    'cadence': 'Sensor / command intervals [ms]',
    'sync': 'Nearest sensor header − source header [ms] (not clock calibration)',
    'command': 'Control requests [normalized] (pre-bridge trim)',
    'state': 'Mode / E2E active / recovery / HOST path / diagnostic severity',
    'board': 'Board-reported output settings [µs] (not measured PWM)',
    'motion': 'IMU angular rate / acceleration / odometry speed proxy (units in legend)',
    'diagnostic': 'Numeric diagnostic values (enable relevant traces in legend)',
}


def downsample(points, maximum):
    """Preserve each bin's extrema and temporal order, including single-sample spikes."""
    if len(points) <= maximum:
        return points
    bins = max(1, maximum // 4)
    width = (len(points) + bins - 1) // bins
    selected = []
    for start in range(0, len(points), width):
        block = points[start:start + width]
        indices = {0, len(block) - 1, min(range(len(block)), key=lambda i: block[i][1]),
                   max(range(len(block)), key=lambda i: block[i][1])}
        selected.extend(block[i] for i in sorted(indices))
    return selected


def dump_csv(path, rows, keys):
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, keys, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def table(rows, columns=None, event_links=False):
    if not rows:
        return '<p>データなし／記録なし</p>'
    columns = columns or list(rows[0])
    def cell(value):
        return html.escape(f'{value:.4f}' if isinstance(value, float) else str(value) if value is not None else '—')
    body = ''
    for row in rows:
        attrs = f' data-time="{row["t_s"]:.9f}" tabindex="0"' if event_links else ''
        body += '<tr' + attrs + '>' + ''.join('<td>' + cell(row.get(c)) + '</td>' for c in columns) + '</tr>'
    return '<div class="scroll"><table><thead><tr>' + ''.join('<th>' + cell(c) + '</th>' for c in columns) + '</tr></thead><tbody>' + body + '</tbody></table></div>'


def response_series(series, topics):
    """Display-only duplicates; original units/times and the full CSV stay intact."""
    command_topic = next((topics[role] for role in ('final', 'auto', 'decoder')
                          if any(s['panel'] == 'command' and s['name'].startswith(topics[role] + ' ') for s in series)), None)
    result, groups = [], {}
    if command_topic is None:
        return result, groups, None
    for panel, prefix, fields in [('response_steering', 'gyro_', ('steering',)),
                                  ('response_drive', 'accel_', ('throttle', 'brake', 'reverse'))]:
        sensors = [s for s in series if s['panel'] == 'motion' and any(s['name'].endswith(prefix + axis) for axis in 'xyz')]
        commands = [s for s in series if s['panel'] == 'command' and s['name'] in [command_topic + ' ' + f for f in fields]]
        if not sensors or not commands:
            continue
        preferred_axis = 'y' if prefix == 'gyro_' else 'x'
        default = next((s for s in sensors if s['name'].startswith('/realsense/imu ') and s['name'].endswith(prefix + preferred_axis)),
                       next((s for s in sensors if s['name'].endswith(prefix + preferred_axis)), sensors[0]))
        result.extend({**s, 'panel': panel, 'secondary': False, 'color': '#1f77b4' if s['name'].endswith(('steering', 'throttle')) else '#b93838' if s['name'].endswith('brake') else '#7856a4'} for s in commands)
        for sensor in sensors:
            result.append({**sensor, 'panel': panel, 'secondary': True, 'response_group': prefix[:-1],
                           'visible': True if sensor is default else 'legendonly', 'color': '#c86118' if prefix == 'gyro_' else '#24895b'})
        groups[prefix[:-1]] = default['name']
    return result, groups, command_topic


def write_report(output, summary, matches, events, series):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    from plotly.offline import get_plotlyjs
    comparison, response_groups, command_topic = response_series(series, summary['config']['topics'])
    display_series = comparison + series
    summary['response_comparison'] = dict(clock='bag recorder time', command_topic=command_topic,
        available=bool(comparison), defaults=response_groups, auto_delay_estimation=False, timestamp_correction=False)
    panels = [p for p in PANELS if any(s['panel'] == p for s in display_series)] or ['age']
    fig = make_subplots(rows=len(panels), cols=1, shared_xaxes=True,
                        vertical_spacing=min(.025, .2 / len(panels)), subplot_titles=[PANELS[p] for p in panels],
                        specs=[[{'secondary_y': p.startswith('response_')}] for p in panels])
    response_choices = {}
    for trace in display_series:
        points = trace['points']
        sampled = downsample(points, summary['config']['plot_max_points'])
        gaps = [b[0] - a[0] for a, b in zip(points, points[1:]) if b[0] > a[0]]
        gap_limit = max(.1, median(gaps) * 3) if gaps else float('inf')
        # Split before downsampling to avoid drawing through missing observations.
        # Sparse diagnostics can legitimately have >1 second intervals.
        breaks = [b[0] for a, b in zip(points, points[1:]) if b[0] - a[0] > gap_limit]
        xs, ys, cursor = [], [], 0
        for time, value in sampled:
            interrupted = False
            while cursor < len(breaks) and breaks[cursor] <= time:
                interrupted = True
                cursor += 1
            if interrupted:
                xs.append(None); ys.append(None)
            xs.append(time); ys.append(value)
        name = trace['name'] + ' [' + trace['unit'] + ']'
        fig.add_trace(go.Scatter(x=xs, y=ys, name=name, mode='lines',
                                line_shape='hv' if trace['step'] else 'linear', connectgaps=False,
                                line=dict(color=trace.get('color')), showlegend=not trace.get('response_group'),
                                visible=trace.get('visible', 'legendonly' if trace['panel'] == 'diagnostic' else True),
                                hovertemplate='%{x:.6f} s<br>%{y:.5f}<extra>%{fullData.name}</extra>'),
                      row=panels.index(trace['panel']) + 1, col=1,
                      **({'secondary_y': trace['secondary']} if trace['panel'].startswith('response_') else {}))
        if trace.get('response_group'):
            response_choices.setdefault(trace['response_group'], []).append(dict(index=len(fig.data)-1, name=name, selected=trace['visible'] is True))
    fit_payload = None
    if 'gyro' in response_choices:
        model_index = len(fig.data)
        fig.add_trace(go.Scatter(x=[], y=[], mode='lines', name='遅延モデル予測 [rad/s]',
                                line=dict(color='#8a3fa0', dash='dash'), visible=False, connectgaps=False),
                      row=panels.index('response_steering')+1, col=1, secondary_y=True)
        command = next(s for s in series if s['panel']=='command' and s['name']==command_topic+' steering')
        fit_payload = dict(command=dict(name=command['name'], points=command['points']),
            gyro={str(c['index']): dict(name=display_series[c['index']]['name'], points=display_series[c['index']]['points'])
                  for c in response_choices['gyro']}, model_index=model_index, duration_s=summary['duration_s'],
            bag=summary.get('bag'), origin_bag_ns=str(summary['origin_bag_ns']), provenance=summary.get('provenance'))
        summary['response_comparison']['model_delay_estimation'] = 'optional signed gain + bias + lag scan, held-out validation'
    for panel, unit in [('response_steering', 'Angular rate [rad/s]'), ('response_drive', 'Acceleration [m/s²]')]:
        if panel in panels:
            row = panels.index(panel) + 1
            fig.update_yaxes(title_text='Command [normalized]', row=row, col=1, secondary_y=False)
            fig.update_yaxes(title_text=unit, row=row, col=1, secondary_y=True)
            fig.update_xaxes(showticklabels=True, title_text='Bag記録時間 [s]', row=row, col=1)
    fig.update_layout(height=max(600, len(panels) * 260), template='plotly_white',
                      margin=dict(l=70, r=80, t=55, b=220), hovermode='x unified',
                      legend=dict(groupclick='toggleitem', orientation='h', x=0, xanchor='left', y=-.03, yanchor='top', maxheight=160),
                      title='指令・IMU・時刻診断（共通のbag記録時間軸）')
    fig.update_xaxes(matches='x', showspikes=True, spikemode='across', spikesnap='cursor', showline=True)
    fig.update_xaxes(title_text='Seconds since first selected recorded message', row=len(panels), col=1)
    payload = fig.to_json().replace('<', '\\u003c')
    plot = '<script>' + get_plotlyjs() + '</script><div id="chart"></div><script id="figure" type="application/json">' + payload + '</script>'
    latency_rows = [dict(stage=k, **v) for k, v in summary['qualified_latency_ms'].items()]
    body = '<p class="notice">これは記録時刻に基づくオフライン診断です。遅延のスパイクと故障原因は同義ではありません。未計測区間を正常と推定しません。</p>'
    body += '<p>bag: ' + html.escape(str(summary.get('bag'))) + '</p>'
    body += '<p><a href="summary.json">summary.json</a> · <a href="timeline.csv">全解像度の時系列 CSV</a> · <a href="matches.csv">フレーム照合 CSV</a> · <a href="events.csv">スパイク・状態変化 CSV</a></p>'
    body += '<h2>画像 → 各段階の最初の記録：条件確認済みフレーム [ms]</h2>' + table(latency_rows)
    body += '<p>count=0は0msや正常という意味ではありません。AUTO・E2E active・復帰非動作を確認できない制御指令は集計から除外し、下のcoverageとCSVに理由を残します。</p>'
    body += '<h2>同一stampの最初の記録差：状態未確認も含む [ms]</h2>' + table([dict(stage=k, **v) for k, v in summary['observed_latency_ms'].items()])
    body += '<p>この表とsource state unverifiedの線は、同じstampの観測差です。復帰の未記録等によりE2E由来と確認できないサンプルも含むため、条件確認済みの集計とは分けています。</p>'
    body += '<h2 id="timeseries">時系列</h2><p>ドラッグで拡大、ダブルクリックで全体へ戻ります。凡例クリックで線を切替できます。数値診断（CPU・温度等）は必要な線を凡例から有効にしてください。表示だけ極値を残して間引き、CSVは間引きません。</p>'
    response_controls = ''
    if comparison:
        response_controls += '<p>先頭2段は指令とIMUの応答比較です（加速度未記録なら上段だけ）。左右で単位を分け、記録時刻をそのまま使います。IMUはsensor座標系で、z軸を車体のヨー軸と仮定しません。加速度は重力を除去していません。</p>'
        for group, label in [('gyro', '角速度 topic / axis'), ('accel', '加速度 topic / axis')]:
            if group in response_choices:
                options = ''.join('<option value="' + str(c['index']) + '"' + (' selected' if c['selected'] else '') + '>' + html.escape(c['name']) + '</option>' for c in response_choices[group])
                response_controls += '<label>' + label + ' <select id="response-' + group + '">' + options + '</select></label> '
        response_controls += '<p>Aは指令が変わった点、BはIMUが変化し始めた点を順にクリックしてください。一定入力・弱い応答・振動では遅れを判定できません。<button id="response-clear">A/Bを選び直す</button><output id="response-delta">A点を選択</output></p>'
        response_controls += '<p>表示するΔtは手動で選んだ記録上の時間差です。機械系・タイヤ・速度・路面・記録時刻のずれを含み、通信遅延の校正値にはしません。元データの時刻は移動・補正しません。</p>'
    else:
        response_controls = '<p>指令＋IMUの比較には、制御指令と有効な角速度／加速度の記録が必要です。</p>'
    response_payload = json.dumps(response_choices, ensure_ascii=False).replace('<', '\\u003c')
    body += response_controls + '<script id="response-choices" type="application/json">' + response_payload + '</script>'
    body += '<div class="controls">表示範囲 [s] <input id="start" type="number" step="0.1" value="0"> — <input id="end" type="number" step="0.1" value="' + str(round(summary['duration_s'], 3)) + '"><button id="zoom">拡大</button><button id="reset">全体</button></div>'
    if fit_payload:
        body += '<details class="response-fit"><summary>操舵と角速度から応答遅れをモデル推定</summary><p><b>操舵 → 選択した角速度のモデル推定</b><br>ω(t) ≈ b + G·steering(t−τ)。符号付きGを適合し、逆相関も扱います。表示範囲を選択して推定してください。元の全解像度データを使い、先頭70%で適合・末尾30%で検証します。</p>'
        body += '<label>τの探索上限 [ms] <input id="response-fit-max" type="number" min="10" max="2000" step="5" value="500"></label><button id="response-fit">応答遅れを推定</button><button id="response-fit-save" disabled>推定JSONを保存</button>'
        body += '<p id="response-fit-result">区間と角速度軸を選び「応答遅れを推定」を押してください</p><p>速度一定は未確認です。固定スロットルでも速度一定とは限りません。τは車体のなまりや記録のずれを含む見かけの遅れで、通信遅延の校正値ではありません。</p>'
        body += '<details><summary>推定の詳細（JSON）</summary><textarea id="response-fit-json" readonly rows="8" style="width:100%"></textarea></details><div id="response-fit-cost" hidden></div></details>'
        body += '<script id="response-fit-data" type="application/json">' + json.dumps(fit_payload, ensure_ascii=False, allow_nan=False).replace('<', '\\u003c') + '</script>'
    body += plot
    body += '<h2>スパイク・状態変化（先頭500件、全件はCSV）</h2><p>行をクリックすると、その時刻の前後1秒を全グラフで表示します。配信gapは欠落確定数ではありません。</p>' + table(events[:500], event_links=True)
    body += '<h2>段階別の照合・除外理由</h2>' + table(summary['coverage'], sorted({k for r in summary['coverage'] for k in r}))
    body += '<h2>各topicの配信と時計</h2>' + table(summary['streams'])
    body += '<h2>センサ間の最近傍timestamp差 [ms]</h2>' + table(summary['synchronization'])
    body += '<h2>不足している記録</h2>' + table(summary['missing_topics'])
    body += '<h2>bag内の全topic</h2>' + table([dict(topic=k, **v) for k, v in summary['inventory'].items()])
    body += '<h2>測定範囲と限界</h2><ul>' + ''.join('<li>' + html.escape(v) + '</li>' for v in summary['limitations']) + '</ul>'
    document = '''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>kart pipeline timing audit</title>
<style>body{font:14px system-ui,sans-serif;margin:24px;color:#193047;background:#f7f9fc}h1{font-size:26px}h2{margin-top:32px}.notice{border-left:4px solid #b77e16;padding:12px;background:#fff5df}p,li{line-height:1.7}.scroll{overflow:auto;max-height:420px;background:white}table{border-collapse:collapse;font-size:12px}td,th{padding:7px;border-bottom:1px solid #ddd;text-align:left;white-space:nowrap}th{position:sticky;top:0;background:#eaf0f7}tr[data-time]{cursor:pointer}tr[data-time]:hover{background:#e4f0ff}input{width:110px}button,input,select{padding:7px;margin:4px}select{max-width:100%}output{font-weight:600}.controls{position:sticky;top:0;background:#f7f9fc;z-index:3}</style>
<h1>kart センサ・E2E・制御の時系列診断</h1>''' + body + '<script>' + Path(__file__).with_name('response_delay.js').read_text() + '\n' + Path(__file__).with_name('response_delay_ui.js').read_text() + '''
const figure=JSON.parse(document.getElementById('figure').textContent);
const chart=document.getElementById('chart');
Plotly.newPlot(chart,figure.data,figure.layout,{responsive:true,scrollZoom:true,displaylogo:false});
function zoom(a,b){if(!Number.isFinite(a)||!Number.isFinite(b)||a>=b)return;const changes={};for(const k of Object.keys(figure.layout)){if(/^xaxis\\d*$/.test(k)){changes[k+'.range']=[a,b];changes[k+'.autorange']=false;}}Plotly.relayout(chart,changes);document.getElementById('start').value=a;document.getElementById('end').value=b;}
document.getElementById('zoom').onclick=()=>zoom(Number(document.getElementById('start').value),Number(document.getElementById('end').value));
document.getElementById('reset').onclick=()=>{const c={};for(const k of Object.keys(figure.layout))if(/^xaxis\\d*$/.test(k))c[k+'.autorange']=true;Plotly.relayout(chart,c);};
const responseElement=document.getElementById('response-choices');
const responseChoices=responseElement?JSON.parse(responseElement.textContent):{};
for(const group of Object.keys(responseChoices)){
 const select=document.getElementById('response-'+group);
 select.onchange=()=>{const choices=responseChoices[group];Plotly.restyle(chart,{visible:choices.map(c=>String(c.index)===select.value?true:'legendonly')},choices.map(c=>c.index));};
}
const clearResponse=document.getElementById('response-clear');
if(clearResponse&&typeof chart.on==='function'){
 let a=null,b=null;
 const output=document.getElementById('response-delta');
 const draw=()=>{const shapes=[a,b].filter(v=>v!==null).map((v,i)=>({type:'line',xref:'x',yref:'paper',x0:v,x1:v,y0:0,y1:1,line:{color:i?'#bd4b18':'#2263ad',width:1.5,dash:'dot'}}));Plotly.relayout(chart,{shapes});};
 clearResponse.onclick=()=>{a=b=null;output.textContent='A点を選択';draw();};
 chart.on('plotly_click',event=>{const value=event.points?.[0]?.x;if(!Number.isFinite(value))return;
  if(a===null||b!==null){a=value;b=null;output.textContent='A = '+a.toFixed(6)+' s · 次にB点を選択';}
  else{b=value;output.textContent='A = '+a.toFixed(6)+' s · B = '+b.toFixed(6)+' s · Δt (B − A) = '+((b-a)*1000).toFixed(2)+' ms（定性的な応答差）';}draw();
 });
}
const fitData=document.getElementById('response-fit-data');
bindResponseDelayUI(chart,figure,fitData?JSON.parse(fitData.textContent):null);
document.querySelectorAll('tr[data-time]').forEach(row=>{const select=()=>{const t=Number(row.dataset.time);zoom(Math.max(0,t-1),t+1);chart.scrollIntoView({block:'start'});};row.onclick=select;row.onkeydown=e=>{if(e.key==='Enter')select();};});
</script></html>'''
    (output / 'report.html').write_text(document, encoding='utf-8')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    dump_csv(output / 'matches.csv', matches, ['t_s', 'topic', 'stage', 'bag_ns', 'source_stamp_ns', 'source_bag_ns',
             'source_to_stage_recorded_ms', 'header_age_ms', 'state', 'repeated', 'qualified', 'reason'])
    dump_csv(output / 'events.csv', events, ['t_s', 'bag_ns', 'topic', 'kind', 'value', 'threshold', 'detail',
             'mode', 'e2e_status', 'recovery_active', 'host_path', 'esc_reported_us'])
    dump_csv(output / 'timeline.csv', (dict(t_s=t, panel=s['panel'], series=s['name'], unit=s['unit'], value=v)
             for s in series for t, v in s['points']), ['t_s', 'panel', 'series', 'unit', 'value'])
