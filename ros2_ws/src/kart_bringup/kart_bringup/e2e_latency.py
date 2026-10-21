"""Offline latency session planning, lifecycle and steady-clock report (ROS-free)."""
import argparse
import csv
import hashlib
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import sys
from datetime import datetime
import yaml


def plan(config, bag, model_dir, output_dir, rate=''):
    from kart_e2e.contract import model_contract, runtime_settings, input_contract, routing_settings
    config = Path(config)
    settings = json.loads((config/'evaluation/e2e_latency.json').read_text())
    expected = {'schema','namespace_prefix','image_topic','camera_info_topic','rate','clock_hz',
                'read_ahead_queue_size','drain_s','use_fixed_throttle'}
    if set(settings) != expected or settings['schema'] != 1:
        raise ValueError('Unsupported E2E latency configuration keys/schema')
    if any(type(settings[k]) is not int or settings[k] <= 0 for k in ('clock_hz','read_ahead_queue_size')):
        raise ValueError('clock_hz/read_ahead_queue_size must be positive integers')
    if type(settings['use_fixed_throttle']) is not bool or not math.isfinite(settings['drain_s']) or settings['drain_s'] < 0:
        raise ValueError('Invalid fixed-throttle selection or drain_s')
    if any(not isinstance(settings[k], str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z_0-9/]*',settings[k])
           for k in ('image_topic','camera_info_topic')):
        raise ValueError('Replay input topics must be absolute ROS names')
    speed = float(rate if rate != '' else settings['rate'])
    if not math.isfinite(speed) or not 0 < speed <= 4:
        raise ValueError('Replay rate must be finite and in (0, 4]')
    bag, model, output = (Path(p).expanduser().resolve() for p in (bag, model_dir, output_dir))
    if not str(bag) or not (bag/'metadata.yaml').is_file():
        raise ValueError('Select a rosbag directory with metadata.yaml')
    if output == bag or bag in output.parents or output == model or model in output.parents:
        raise ValueError('Output must be separate from input bag/model')
    prefix = settings['namespace_prefix']
    if not isinstance(prefix, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', prefix):
        raise ValueError('Latency namespace_prefix must be a nonempty ROS name')
    settings['namespace'] = prefix + '_' + hashlib.sha256(str(output).encode()).hexdigest()[:8]
    try:
        metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        available = {r['topic_metadata']['name']: r for r in metadata['topics_with_message_count'] if r['message_count'] > 0}
    except (KeyError, TypeError, yaml.YAMLError) as exc:
        raise ValueError(f'Invalid rosbag metadata: {bag}') from exc
    for name, kind in [('image_topic','sensor_msgs/msg/Image'), ('camera_info_topic','sensor_msgs/msg/CameraInfo')]:
        topic = settings[name]
        if topic not in available or available[topic]['topic_metadata']['type'] != kind:
            raise ValueError(f'Bag lacks required {kind}: {topic}')
    model_contract(model)
    runtime = runtime_settings(model)
    if input_contract(model)[0] != 'rgb':
        raise ValueError('E2E latency replay currently requires an RGB model; EVS RAW/tensor replay is unsupported')
    if routing_settings(model):
        raise ValueError('Section-routed model needs pose/head replay; select a single-head RGB model for this latency session')
    ns = '/' + settings['namespace']
    playback = ['ros2','bag','play',str(bag),'--rate',str(speed), '--clock',str(settings['clock_hz']),
                '--read-ahead-queue-size',str(settings['read_ahead_queue_size']),
                '--start-paused','--disable-keyboard-controls',
                '--topics',settings['image_topic'],settings['camera_info_topic'],
                '--remap',settings['image_topic']+':='+ns+'/replay/image',
                settings['camera_info_topic']+':='+ns+'/replay/camera_info',
                '/clock:='+ns+'/clock','__ns:='+ns]
    return dict(schema='kart.e2e_latency.session.v1', bag=str(bag), model_dir=str(model),
                output_dir=str(output), rate=speed, settings=settings, playback=playback,
                model_sha256=hashlib.sha256((model/'model.onnx').read_bytes()).hexdigest(),
                image_count=available[settings['image_topic']]['message_count'],
                runtime=runtime, configuration_sha256={str(p.relative_to(config)):hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [config/'evaluation/e2e_latency.json',config/'evaluation/e2e_latency.yaml',
                              config/'e2e/image_encoder.yaml',config/'e2e/tensor_rt.yaml',config/'e2e/control_decoder.yaml']})


def statistics(values):
    values = sorted(values)
    def percentile(q):
        index = (len(values)-1)*q
        a, b = math.floor(index), math.ceil(index)
        return values[a]+(values[b]-values[a])*(index-a)
    return dict(count=len(values), **({k: v for k,v in [('mean',sum(values)/len(values)),
        ('p50',percentile(.5)),('p95',percentile(.95)),('p99',percentile(.99)),
        ('max',values[-1])]} if values else {k:None for k in ('mean','p50','p95','p99','max')}))


def report(output, session, returncode):
    from plotly.subplots import make_subplots
    import plotly.graph_objects as go
    output = Path(output)
    samples = output/'samples.csv'
    rows = []
    malformed = 0
    if samples.exists():
        with samples.open() as stream:
            for r in csv.DictReader(stream):
                try:
                    if not r['event'] or r['warmup'] not in ('0','1') or r['first'] not in ('0','1'):
                        raise ValueError('Partial row')
                    if not math.isfinite(float(r['elapsed_s'])):raise ValueError('Nonfinite time')
                    int(r['source_stamp_ns'])
                    if r['event'] in ('command','decoded') and not math.isfinite(float(r['latency_ms'])):
                        raise ValueError('Nonfinite latency')
                    r['detail'] = r.get('detail') or ''
                    rows.append(r)
                except (KeyError, TypeError, ValueError):
                    malformed += 1
    inputs = [r for r in rows if r['event']=='input']
    measured = [r for r in inputs if r['warmup']=='0']
    ambiguous = {r['source_stamp_ns'] for r in rows if r['event']=='ambiguous_input'}
    stamps = {r['source_stamp_ns'] for r in measured} - ambiguous
    metrics = {}
    for event in ('decoded','command'):
        selected = [r for r in rows if r['event']==event and r['first']=='1' and r['warmup']=='0' and r['source_stamp_ns'] in stamps]
        covered = {r['source_stamp_ns'] for r in selected}
        metrics[event] = dict(latency_ms=statistics([float(r['latency_ms']) for r in selected]),
                              input_frames_without_observed_stage=len(stamps-covered))
    playback = output/'playback.json'
    player = json.loads(playback.read_text()) if playback.exists() else None
    complete = returncode == 0 and player is not None and player['returncode'] == 0
    valid = complete and all(metrics[stage]['latency_ms']['count'] > 0 for stage in ('decoded','command'))
    input_gaps = [(float(b['elapsed_s'])-float(a['elapsed_s']))*1000 for a,b in zip(inputs,inputs[1:])]
    result = dict(schema='kart.e2e_latency.report.v1', status='completed' if valid else 'incomplete',
                  launch_returncode=returncode, playback=player, input_frames=len(inputs),
                  received_image_messages=len(inputs)+sum(r['event']=='ambiguous_input' for r in rows),
                  bag_images_not_observed_at_ingress=max(0, session['image_count']-len(inputs)-sum(r['event']=='ambiguous_input' for r in rows)),
                  source_bag_image_count=session['image_count'], measured_input_frames=len(stamps),
                  malformed_csv_rows=malformed,
                  ambiguous_inputs=sum(r['event']=='ambiguous_input' for r in rows),
                  relayed_input_interval_ms=statistics(input_gaps), stages=metrics,
                  command_republications=sum(r['event']=='command' and r['first']=='0' for r in rows),
                  session=session,
                  limitations=[
                      'Steady-clock latency begins at replay ingress callback, not exposure, original bag time or player publish.',
                      'Decoded event follows CUDA output readback; command includes the production decoder timer and this observer callback.',
                      'Decoded frames and first command per frame are separate. Decoder runs at its configured cadence, so not every decoded frame produces a command.',
                      'Original header timestamps are retained for exact matching; reused/regressed stamps are excluded, never nearest-matched.',
                      'STOP warmup then synthetic AUTO is offline-only. No bridge, mux, recovery or physical actuator is included.',
                      'Warmup inputs are excluded from summary latency. Raw CSV preserves warmup, repeated and unmatched events.',
                      'Ingress relay and lightweight completion observer introduce measurement overhead; sensor USB and physical response are not replayed.',
                      'Statuses after the last input include the drain/watchdog tail. Missing stage is not proof of a camera or inference drop.',
                      'Replay rate changes load. GPU type, engine, other processes and recorder activity affect timing.',
                  ])
    (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    figure = make_subplots(rows=3,cols=1,shared_xaxes=True,subplot_titles=[
        'Replay ingress → decoded / first command (steady ms)', 'Replay ingress interval (ms)', 'Decoder gate status'])
    for event in ('decoded','command'):
        rr=[r for r in rows if r['event']==event and r['first']=='1' and r['source_stamp_ns'] not in ambiguous]
        figure.add_trace(go.Scattergl(x=[float(r['elapsed_s']) for r in rr],y=[float(r['latency_ms']) for r in rr],
                                     mode='lines+markers',marker_size=3,name=event,
                                     text=['warmup' if r['warmup']=='1' else 'measured' for r in rr]),row=1,col=1)
    figure.add_trace(go.Scattergl(x=[float(r['elapsed_s']) for r in inputs[1:]],y=input_gaps,name='input interval'),row=2,col=1)
    rr=[r for r in rows if r['event']=='status']
    figure.add_trace(go.Scatter(x=[float(r['elapsed_s']) for r in rr],y=[r['detail'] for r in rr],mode='markers',name='status'),row=3,col=1)
    figure.update_layout(height=920,title=f"E2E offline latency · {result['status']} · rate {session['rate']}x",
                         hovermode='x unified')
    figure.update_xaxes(title_text='Session steady elapsed (s)',row=3,col=1)
    html = figure.to_html(full_html=True,include_plotlyjs=True)
    import html as escape
    explanation = '<p>計測範囲: 再生入力の受信 → デコード完了／最初の制御topic受信。撮影・USB・車両出力は含みません。</p>'
    explanation += '<p>STOP warmup → synthetic AUTO。全点を表示し、warmupもhoverに残します。集計ではwarmupを除外します。</p>'
    explanation += '<p><a href="summary.json">summary.json</a> · <a href="samples.csv">samples.csv</a> · <a href="session.json">session.json</a></p>'
    explanation += '<pre>'+escape.escape(json.dumps(metrics,ensure_ascii=False,indent=2))+'</pre>'
    (output/'report.html').write_text(html.replace('<body>','<body>'+explanation))
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description='Offline RGB E2E latency replay; no physical control')
    for key in ('bag','model-dir'):
        parser.add_argument('--'+key,required=True)
    parser.add_argument('--output',default='')
    parser.add_argument('--rate',default='')
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args(argv)
    from ament_index_python.packages import get_package_share_directory
    config=Path(get_package_share_directory('kart_bringup'))/'config'
    output=Path(args.output).expanduser() if args.output else Path('analysis/e2e_latency')/datetime.now().strftime('%Y-%m-%d/%H-%M-%S-%f')
    session=plan(config,args.bag,args.model_dir,output,args.rate)
    if args.dry_run:
        print(json.dumps(session,ensure_ascii=False,indent=2));return 0
    # Check report dependency before any GPU work or playback is started.
    import plotly  # noqa: F401
    output=Path(session['output_dir'])
    output.mkdir(parents=True,exist_ok=False)
    (output/'session.json').write_text(json.dumps(session,ensure_ascii=False,indent=2)+'\n')
    command=['ros2','launch','kart_bringup','e2e_latency.launch.py',f'bag:={session["bag"]}',
             f'model_dir:={session["model_dir"]}',f'output_dir:={output}',f'rate:={session["rate"]}']
    process=None;code=1
    try:
        with (output/'launch.log').open('w') as log:
            process=subprocess.Popen(command,start_new_session=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            for line in process.stdout:
                log.write(line);log.flush();print(line,end='',flush=True)
            code=process.wait()
    except KeyboardInterrupt:
        code=130
        if process is not None and process.poll() is None:
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);process.wait()
    finally:
        result=report(output,session,code)
        print(f'Latency result ({result["status"]}): {output}/report.html',flush=True)
    return code if code else (0 if result['status']=='completed' else 1)


if __name__=='__main__':
    sys.exit(main())
