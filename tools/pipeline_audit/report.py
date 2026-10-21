"""Standalone Plotly time series with shared zoom; full-resolution CSV is kept."""
import csv
import html
import json
from statistics import median

PANELS = {
    'age': 'Header → bag recorder age [ms] (clock offset / queueing included)',
    'matched': 'Exact same frame: first recorded observation gaps [ms]',
    'cadence': 'Sensor / command intervals [ms]',
    'sync': 'Nearest sensor header − source header [ms] (not clock calibration)',
    'command': 'Control requests [normalized] (pre-bridge trim)',
    'state': 'Mode / E2E active / recovery / HOST path / diagnostic severity',
    'board': 'Board-reported output settings [µs] (not measured PWM)',
    'motion': 'IMU angular rate / odometry speed proxy (units in legend)',
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


def write_report(output, summary, matches, events, series):
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    from plotly.offline import get_plotlyjs
    panels = [p for p in PANELS if any(s['panel'] == p for s in series)] or ['age']
    fig = make_subplots(rows=len(panels), cols=1, shared_xaxes=True,
                        vertical_spacing=min(.025, .2 / len(panels)), subplot_titles=[PANELS[p] for p in panels])
    for trace in series:
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
                                visible='legendonly' if trace['panel'] == 'diagnostic' else True,
                                hovertemplate='%{x:.6f} s<br>%{y:.5f}<extra>%{fullData.name}</extra>'),
                      row=panels.index(trace['panel']) + 1, col=1)
    fig.update_layout(height=max(600, len(panels) * 260), template='plotly_white',
                      margin=dict(l=70, r=20, t=55, b=60), hovermode='x unified',
                      legend=dict(groupclick='toggleitem'), title='Same time axis — recorded bag time [s]')
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
    body += '<div class="controls">表示範囲 [s] <input id="start" type="number" step="0.1" value="0"> — <input id="end" type="number" step="0.1" value="' + str(round(summary['duration_s'], 3)) + '"><button id="zoom">拡大</button><button id="reset">全体</button></div>' + plot
    body += '<h2>スパイク・状態変化（先頭500件、全件はCSV）</h2><p>行をクリックすると、その時刻の前後1秒を全グラフで表示します。配信gapは欠落確定数ではありません。</p>' + table(events[:500], event_links=True)
    body += '<h2>段階別の照合・除外理由</h2>' + table(summary['coverage'], sorted({k for r in summary['coverage'] for k in r}))
    body += '<h2>各topicの配信と時計</h2>' + table(summary['streams'])
    body += '<h2>センサ間の最近傍timestamp差 [ms]</h2>' + table(summary['synchronization'])
    body += '<h2>不足している記録</h2>' + table(summary['missing_topics'])
    body += '<h2>bag内の全topic</h2>' + table([dict(topic=k, **v) for k, v in summary['inventory'].items()])
    body += '<h2>測定範囲と限界</h2><ul>' + ''.join('<li>' + html.escape(v) + '</li>' for v in summary['limitations']) + '</ul>'
    document = '''<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>kart pipeline timing audit</title>
<style>body{font:14px system-ui,sans-serif;margin:24px;color:#193047;background:#f7f9fc}h1{font-size:26px}h2{margin-top:32px}.notice{border-left:4px solid #b77e16;padding:12px;background:#fff5df}p,li{line-height:1.7}.scroll{overflow:auto;max-height:420px;background:white}table{border-collapse:collapse;font-size:12px}td,th{padding:7px;border-bottom:1px solid #ddd;text-align:left;white-space:nowrap}th{position:sticky;top:0;background:#eaf0f7}tr[data-time]{cursor:pointer}tr[data-time]:hover{background:#e4f0ff}input{width:110px}button,input{padding:7px;margin:4px}.controls{position:sticky;top:0;background:#f7f9fc;z-index:3}</style>
<h1>kart センサ・E2E・制御の時系列診断</h1>''' + body + '''
<script>
const figure=JSON.parse(document.getElementById('figure').textContent);
const chart=document.getElementById('chart');
Plotly.newPlot(chart,figure.data,figure.layout,{responsive:true,scrollZoom:true,displaylogo:false});
function zoom(a,b){if(!Number.isFinite(a)||!Number.isFinite(b)||a>=b)return;const changes={};for(const k of Object.keys(figure.layout)){if(/^xaxis\\d*$/.test(k)){changes[k+'.range']=[a,b];changes[k+'.autorange']=false;}}Plotly.relayout(chart,changes);document.getElementById('start').value=a;document.getElementById('end').value=b;}
document.getElementById('zoom').onclick=()=>zoom(Number(document.getElementById('start').value),Number(document.getElementById('end').value));
document.getElementById('reset').onclick=()=>{const c={};for(const k of Object.keys(figure.layout))if(/^xaxis\\d*$/.test(k))c[k+'.autorange']=true;Plotly.relayout(chart,c);};
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
