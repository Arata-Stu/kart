"""Synthetic evidence and real SQLite/MCAP containers; no ROS/GPU/vehicle claim."""
import copy
import json
from pathlib import Path
import struct
import shutil
import subprocess

import pytest

from tools.pipeline_audit.analysis import analyze, validate
from tools.pipeline_audit.bag import COMMAND, header_stamp, read_bag
from tools.pipeline_audit.cli import main
from tools.pipeline_audit.report import downsample

CONFIG = json.loads(Path(__file__).with_name('config.json').read_text())
BASE = 1_790_000_000_000_000_000


def sample(role, ms, stamp_ms=None, payload=None, kind=None):
    topic = CONFIG['topics'].get(role, role)
    kinds = {'source': 'sensor_msgs/msg/Image', 'auto': COMMAND, 'final': COMMAND,
             'decoder': COMMAND, 'mode': 'kart_interfaces/msg/OperationModeState',
             'e2e_status': 'std_msgs/msg/String', 'recovery_active': 'std_msgs/msg/Bool'}
    return dict(topic=topic, type=kind or kinds.get(role, 'std_msgs/msg/String'),
                bag_ns=BASE + round(ms * 1e6), stamp_ns=None if stamp_ms is None else BASE + round(stamp_ms * 1e6),
                payload=payload or {}, error=None, connection=1)


def fixture():
    rows = []
    for t in range(0, 1000, 100):
        rows.extend([sample('mode', t, t, {'mode': 1}), sample('e2e_status', t, payload={'data': 'active'}),
                     sample('recovery_active', t, payload={'data': False})])
    for t in range(20, 900, 40):
        rows.append(sample('source', t + 5, t))
        rows.append(sample('auto', t + 20, t, {'steering': .1, 'throttle': .2, 'brake': 0., 'reverse': 0.}))
        for delta in (30, 40, 50):
            rows.append(sample('final', t + delta, t, {'steering': .1, 'throttle': .2, 'brake': 0., 'reverse': 0.}))
    return dict(schema=1, bag='synthetic', inventory={}, records=sorted(rows, key=lambda r: r['bag_ns']))


def test_first_frame_latency_and_repeated_control_are_separate():
    summary, matches, _, _ = analyze(fixture(), CONFIG)
    metric = summary['qualified_latency_ms']['final']
    assert metric['count'] == 22 and metric['mean'] == 25
    final = [r for r in matches if r['stage'] == 'final']
    assert sum(r['repeated'] for r in final) == 44
    assert len([r for r in final if r['qualified']]) == 22
    # Raw image-header age is 30ms, distinct from receipt-to-receipt's 25ms.
    assert final[0]['header_age_ms'] == 30


@pytest.mark.parametrize('change, reason', [('manual', 'not_auto'), ('inactive', 'e2e_not_confirmed_active'),
                                         ('recovery', 'recovery_active'), ('missing', 'recovery_unobserved'),
                                         ('mode_missing', 'mode_unknown_or_stale')])
def test_source_classification_never_reports_manual_or_recovery_as_inference(change, reason):
    data = fixture()
    if change in ('missing', 'mode_missing'):
        target = CONFIG['topics']['recovery_active' if change == 'missing' else 'mode']
        data['records'] = [r for r in data['records'] if r['topic'] != target]
    for row in data['records']:
        if change == 'manual' and row['topic'] == CONFIG['topics']['mode']:
            row['payload']['mode'] = 2
        if change == 'inactive' and row['topic'] == CONFIG['topics']['e2e_status']:
            row['payload']['data'] = 'inactive'
        if change == 'recovery' and row['topic'] == CONFIG['topics']['recovery_active']:
            row['payload']['data'] = True
    summary, matches, _, series = analyze(data, CONFIG)
    assert summary['qualified_latency_ms']['final']['count'] == 0
    assert summary['observed_latency_ms']['final']['count'] == 22
    assert all(m['state'] == reason for m in matches if m['stage'] == 'final')
    assert any('source state unverified' in s['name'] for s in series)


def test_spikes_future_stamps_gaps_regression_and_invalid_timestamps_are_visible():
    data = fixture()
    data['records'].extend([sample('source', 1200, 1600), sample('source', 1400, 1300),
                            sample('source', 1410, 1310), sample('source', 1420, -BASE / 1e6)])
    summary, _, events, _ = analyze(data, CONFIG)
    kinds = {e['kind'] for e in events}
    assert {'future_stamp', 'receipt_gap', 'stamp_regression', 'invalid_stamp'} <= kinds
    assert summary['streams']


def test_matching_is_exact_and_ambiguous_reused_stamps_are_rejected():
    data = fixture()
    first = next(r for r in data['records'] if r['topic'] == CONFIG['topics']['source'])
    data['records'].append({**first, 'bag_ns': first['bag_ns'] + 1_000_000})
    _, matches, _, _ = analyze(data, CONFIG)
    assert matches[0]['reason'] == 'ambiguous_source_stamp'
    data = fixture()
    for row in data['records']:
        if row['topic'] == CONFIG['topics']['final']:
            row['stamp_ns'] += 1  # One nanosecond is not the same image.
    summary, matches, _, _ = analyze(data, CONFIG)
    assert summary['observed_latency_ms']['final']['count'] == 0
    assert all(m['reason'] == 'no_exact_source_stamp' for m in matches if m['stage'] == 'final')


def test_recorder_order_inversions_are_not_negative_processing_latencies():
    data = fixture()
    for row in data['records']:
        if row['topic'] == CONFIG['topics']['source']:
            row['bag_ns'] += 100_000_000
    summary, matches, events, _ = analyze(data, CONFIG)
    assert summary['qualified_latency_ms']['final']['count'] == 0
    assert any(e['kind'] == 'recorder_order_inversion' for e in events)
    assert any(m['source_to_stage_recorded_ms'] < 0 for m in matches)


def test_future_state_is_not_used_and_stale_state_does_not_qualify():
    data = fixture()
    for row in data['records']:
        if row['topic'] == CONFIG['topics']['mode']:
            row['stamp_ns'] += 1_000_000_000
    summary, _, _, _ = analyze(data, CONFIG)
    assert summary['qualified_latency_ms']['final']['count'] == 0
    data = fixture()
    data['records'] = [r for r in data['records'] if r['topic'] != CONFIG['topics']['mode'] or r['bag_ns'] == BASE]
    _, matches, _, _ = analyze(data, CONFIG)
    assert any(m['state'] == 'mode_unknown_or_stale' for m in matches)


def test_event_context_and_nearest_sensor_difference():
    data = fixture()
    data['records'].append(sample('active_path', 0, payload={'data': 2}, kind='std_msgs/msg/UInt8'))
    data['records'].append(sample('output_channels', 100, payload={'data': [1500, 2000]}, kind='std_msgs/msg/Int32MultiArray'))
    data['records'].append(sample('logs', 150, payload={'level': 40, 'name': 'test', 'message': 'warning'}, kind='rcl_interfaces/msg/Log'))
    for t in range(20, 900, 40):
        data['records'].append(sample(CONFIG['sync_topics'][0], t + 8, t + 2, kind='sensor_msgs/msg/Image'))
    summary, _, events, _ = analyze(data, CONFIG)
    warning = next(e for e in events if e['kind'] == 'ros_warning')
    assert warning['mode'] == 1 and warning['e2e_status'] == 'active'
    assert warning['recovery_active'] is False and warning['esc_reported_us'] == 2000
    assert summary['synchronization'][0]['mean'] == 2


def test_event_image_recording_age_is_separate_from_header_matching():
    data = fixture()
    topic = '/event_camera/event_image'
    assert topic in CONFIG['sync_topics']
    # Same header as an RGB frame, but received 300 ms after that header.
    # Header matching alone must not be called physical synchronization.
    for t in range(20, 900, 40):
        data['records'].append(sample(topic, t + 300, t, kind='sensor_msgs/msg/Image'))
    summary, _, _, _ = analyze(data, CONFIG)
    stream = next(s for s in summary['streams'] if s['topic'] == topic)
    assert stream['age_mean'] == 300
    sync = next(s for s in summary['synchronization'] if s['topic'] == topic)
    assert sync['mean'] == 0
    missing, _, _, _ = analyze(fixture(), CONFIG)
    sync = next(s for s in missing['synchronization'] if s['topic'] == topic)
    assert sync['mean'] is None


def test_spike_preserving_downsampling_and_chronological_order():
    points = [(i / 100, 500. if i == 54321 else -200. if i == 54322 else 1.) for i in range(100000)]
    out = downsample(points, 1000)
    assert (543.21, 500.) in out and (543.22, -200.) in out
    assert out == sorted(out) and len(out) <= 1000


@pytest.mark.parametrize('endian, encapsulation', [('<', b'\0\1\0\0'), ('>', b'\0\0\0\0')])
def test_cdr_header_prefix_without_decoding_pixels(endian, encapsulation):
    assert header_stamp(encapsulation + struct.pack(endian + 'iI', 1790000000, 123456789)) == BASE + 123456789
    with pytest.raises(ValueError):
        header_stamp(encapsulation + struct.pack(endian + 'iI', 1, 1000000000))
    with pytest.raises(ValueError):
        header_stamp(b'\0\7\0\0' + b'0' * 8)


@pytest.mark.parametrize('storage', ['SQLITE3', 'MCAP'])
def test_real_container_images_custom_messages_and_board_values(tmp_path, monkeypatch, storage):
    import numpy as np
    from rosbags.rosbag2 import Writer, StoragePlugin
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    from rosbags.highlevel import AnyReader
    store = get_typestore(Stores.LATEST)
    root = Path(__file__).resolve().parents[2]
    for name in ('ControlCommand', 'OperationModeState'):
        store.register(get_types_from_msg((root / 'ros2_ws/src/kart_interfaces/msg' / (name + '.msg')).read_text(), 'kart_interfaces/msg/' + name))
    def make(kind, *args):
        return store.types[kind](*args)
    def header(ns):
        return make('std_msgs/msg/Header', make('builtin_interfaces/msg/Time', *divmod(ns, 10**9)), 'camera')
    image_type = 'sensor_msgs/msg/Image'
    image = make(image_type, header(BASE + 20_000_000), 1, 2, 'rgb8', 0, 6, np.zeros(6, np.uint8))
    values = [('mode', BASE, make('kart_interfaces/msg/OperationModeState', header(BASE), 1, 'test')),
              ('e2e_status', BASE, make('std_msgs/msg/String', 'active')),
              ('recovery_active', BASE, make('std_msgs/msg/Bool', False)),
              ('source', BASE + 25_000_000, image),
              ('final', BASE + 50_000_000, make(COMMAND, header(BASE + 20_000_000), .1, .2, 0., 0.)),
              ('output_channels', BASE + 51_000_000, make('std_msgs/msg/Int32MultiArray',
                 make('std_msgs/msg/MultiArrayLayout', [], 0), np.array([1500, 2000], np.int32)))]
    bag = tmp_path / storage
    with Writer(bag, version=9, storage_plugin=getattr(StoragePlugin, storage)) as writer:
        for role, t, msg in values:
            kind = msg.__msgtype__
            conn = writer.add_connection(CONFIG['topics'][role], kind, typestore=store)
            writer.write(conn, t, store.serialize_cdr(msg, kind))
    original = AnyReader.deserialize
    def guarded(self, raw, kind):
        assert kind != image_type, 'Image payload must not be deserialized'
        return original(self, raw, kind)
    monkeypatch.setattr(AnyReader, 'deserialize', guarded)
    data = read_bag(bag, CONFIG)
    summary, _, _, series = analyze(data, CONFIG)
    assert summary['qualified_latency_ms']['final']['mean'] == 25
    assert any(s['panel'] == 'board' and s['points'][0][1] == 2000 for s in series)
    assert not any(row.get('error') for row in data['records'])


def test_evs_metadata_source_and_wrong_type_are_explicit(tmp_path):
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.LATEST)
    config = copy.deepcopy(CONFIG)
    config['topics']['source'] = config['topics']['evs_timing']
    validate(config)
    bag = tmp_path / 'evs'
    timing = dict(schema='openeb.tensor_timing.v1', tensor_stamp_ros_ns=BASE + 20_000_000,
                  window_start_sensor_us=0, window_end_sensor_us=40000,
                  stream_id='test', sensor_epoch=0, sequence=0, generated_ros_ns=BASE + 21_000_000)
    with Writer(bag, version=9) as writer:
        kind = 'std_msgs/msg/String'
        conn = writer.add_connection(config['topics']['source'], kind, typestore=store)
        writer.write(conn, BASE + 25_000_000, store.serialize_cdr(store.types[kind](json.dumps(timing)), kind))
        bad = writer.add_connection(config['topics']['mode'], kind, typestore=store)
        writer.write(bad, BASE, store.serialize_cdr(store.types[kind]('not a mode'), kind))
    data = read_bag(bag, config)
    source = next(r for r in data['records'] if r['topic'] == config['topics']['source'])
    assert source['stamp_ns'] == BASE + 20_000_000
    assert source['payload']['evs_window_ms'] == 40
    summary, _, events, _ = analyze(data, config)
    assert summary['qualified_latency_ms']['final']['count'] == 0
    assert any(e['kind'] == 'decode_error' and 'unexpected type' in e['detail'] for e in events)


def test_input_recording_output_is_refused_before_read(tmp_path):
    bag = tmp_path / 'input'
    bag.mkdir()
    assert main(['--bag', str(bag), '--output', str(bag / 'analysis')]) == 2
    assert not (bag / 'analysis').exists()


def test_cli_self_contained_report_safe_text_and_refuses_overwrite(tmp_path):
    data = fixture()
    data['bag'] = '</script><script>alert(1)</script>'
    data['records'].append(sample('logs', 150, payload={'level': 40, 'name': 'test', 'message': '<img src=x onerror=alert(1)>'}, kind='rcl_interfaces/msg/Log'))
    path = tmp_path / 'fixture.json'
    path.write_text(json.dumps(data))
    output = tmp_path / 'report'
    assert main(['--records-json', str(path), '--output', str(output)]) == 0
    report = (output / 'report.html').read_text()
    assert 'plotly.js' in report and 'cdn.plot.ly/' not in report.split('<div id="chart">')[1]
    assert '<img src=x onerror=alert(1)>' not in report
    assert '&lt;img src=x' in report
    assert {p.name for p in output.iterdir()} == {'report.html', 'summary.json', 'events.csv', 'matches.csv', 'timeline.csv', 'metadata.jsonl'}
    before = (output / 'summary.json').read_bytes()
    assert main(['--records-json', str(path), '--output', str(output)]) == 2
    assert (output / 'summary.json').read_bytes() == before


def test_generated_html_shared_zoom_and_event_navigation_handlers(tmp_path):
    node = shutil.which('node')
    if not node:
        pytest.skip('Node.js is needed for generated report interaction handlers')
    from tools.pipeline_audit.report import write_report
    summary, matches, events, series = analyze(fixture(), CONFIG)
    write_report(tmp_path, summary, matches, events, series)
    script = r'''
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[1],'utf8');
const payload=html.match(/<script id="figure" type="application\/json">([\s\S]*?)<\/script>/)[1];
const handlers=[...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].at(-1)[1];
const elements={figure:{textContent:payload},chart:{scrollIntoView(){this.scrolled=true;}},
 start:{value:'.1'},end:{value:'.2'},zoom:{},reset:{}};
const event={dataset:{time:'.5'}};
let changes=null,plotted=null;
const document={getElementById:id=>elements[id],querySelectorAll:()=>[event]};
const Plotly={newPlot:(chart,data,layout)=>{plotted={data,layout};},relayout:(chart,value)=>{changes=value;}};
vm.runInNewContext(handlers,{document,Plotly,Number,Object});
assert.ok(plotted.data.length>5);
assert.ok(Object.keys(plotted.layout).filter(k=>/^xaxis\d*$/.test(k)).length>1);
elements.zoom.onclick();
assert.ok(Object.keys(changes).filter(k=>k.endsWith('.range')).length>1);
assert.ok(Object.values(changes).filter(Array.isArray).every(v=>v[0]===.1&&v[1]===.2));
elements.reset.onclick();assert.ok(Object.values(changes).every(v=>v===true));
event.onclick();assert.ok(elements.chart.scrolled);
assert.ok(Object.values(changes).filter(Array.isArray).every(v=>v[0]===0&&v[1]===1.5));
console.log('Generated report: shared zoom, reset and event navigation passed (no browser rendering)');
'''
    result = subprocess.run([node, '-e', script, str(tmp_path / 'report.html')], text=True, capture_output=True)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('key,value', [('gap_factor', 0), ('match_window_s', float('nan')),
                                     ('plot_max_points', 4), ('future_tolerance_ms', -1)])
def test_invalid_configuration_is_rejected(key, value):
    config = copy.deepcopy(CONFIG)
    config[key] = value
    with pytest.raises(ValueError):
        validate(config)


def response_fixture():
    rows = []
    for t in range(0, 1000, 10):
        rows.append(sample('final', t, t, dict(steering=0. if t < 200 else .4, throttle=.15, brake=0., reverse=0.)))
        rows.append(sample('/realsense/imu', t + 1, t - 1,
                    dict(frame_id='camera_imu_optical_frame', gyro_x=.01, gyro_y=.02,
                         gyro_z=0. if t < 280 else .3, accel_x=.2, accel_y=0., accel_z=9.81),
                    kind='sensor_msgs/msg/Imu'))
    return dict(bag='response fixture', records=rows, inventory={})


def test_imu_acceleration_covariance_and_frame_from_real_mcap(tmp_path):
    import numpy as np
    from rosbags.rosbag2 import Writer, StoragePlugin
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.LATEST)
    make = lambda kind, *args: store.types[kind](*args)
    kind = 'sensor_msgs/msg/Imu'
    header = make('std_msgs/msg/Header', make('builtin_interfaces/msg/Time', *divmod(BASE, 10**9)), 'imu_optical')
    unavailable = np.zeros(9); unavailable[0] = -1.
    message = make(kind, header, make('geometry_msgs/msg/Quaternion', 0.,0.,0.,1.), unavailable,
                   make('geometry_msgs/msg/Vector3', 0.,0.,0.), unavailable,
                   make('geometry_msgs/msg/Vector3', 1.,2.,9.81), np.zeros(9))
    bag = tmp_path/'imu'
    with Writer(bag, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        conn = writer.add_connection('/imu_accel_only', kind, typestore=store)
        writer.write(conn, BASE+10_000_000, store.serialize_cdr(message, kind))
    data = read_bag(bag, CONFIG)
    row = data['records'][0]
    assert row['payload']['frame_id'] == 'imu_optical'
    assert row['payload']['gyro_z'] is None  # No false zero gyro from an accel-only message.
    assert row['payload']['accel_z'] == 9.81
    assert row['stamp_ns'] == BASE and row['bag_ns'] == BASE+10_000_000
    _, _, _, series = analyze(data, CONFIG)
    assert not any('gyro_' in s['name'] for s in series)
    assert any(s['name'].endswith('accel_z') and s['unit']=='m/s²' for s in series)


def test_response_dual_axes_preserve_times_units_and_choose_final_command(tmp_path):
    from tools.pipeline_audit.report import response_series, write_report
    summary, matches, events, series = analyze(response_fixture(), CONFIG)
    comparison, groups, topic = response_series(series, CONFIG['topics'])
    assert topic == CONFIG['topics']['final']
    assert groups['gyro'].endswith('gyro_y') and groups['accel'].endswith('accel_x')
    steer = next(s for s in comparison if s['name'].endswith(' steering'))
    gyro = next(s for s in comparison if s['name'].endswith('gyro_z'))
    assert next(t for t,v in steer['points'] if v != 0) == .2
    assert next(t for t,v in gyro['points'] if v != 0) == .281  # Recorder time, not .279 header time.
    assert steer['secondary'] is False and gyro['secondary'] is True
    assert steer['unit']=='normalized' and gyro['unit']=='rad/s'
    write_report(tmp_path, summary, matches, events, series)
    text = (tmp_path/'report.html').read_text()
    assert 'response-gyro' in text and 'response-accel' in text and 'response-delta' in text
    assert summary['response_comparison']['timestamp_correction'] is False
    assert 'response_' not in (tmp_path/'timeline.csv').read_text() # Render duplicates do not duplicate raw CSV.
    import re
    figure=json.loads(re.search(r'<script id="figure" type="application/json">(.*?)</script>',text).group(1))
    assert figure['layout']['yaxis2']['overlaying']=='y'
    assert figure['layout']['yaxis4']['overlaying']=='y3'
    assert figure['layout']['yaxis2']['title']['text']=='Angular rate [rad/s]'


def test_response_ui_selects_imu_axis_and_reads_manual_time_difference(tmp_path):
    node = shutil.which('node')
    if node is None:pytest.skip('Node needed for generated interaction handlers')
    from tools.pipeline_audit.report import write_report
    write_report(tmp_path, *analyze(response_fixture(), CONFIG))
    script = r'''
const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const html=fs.readFileSync(process.argv[1],'utf8');
const value=id=>html.match(new RegExp('<script id="'+id+'" type="application/json">([\\s\\S]*?)</script>'))[1];
let clicked,updated,restyled;
const elements={figure:{textContent:value('figure')},'response-choices':{textContent:value('response-choices')},
 chart:{on(name,handler){clicked=handler;}},'response-gyro':{},'response-accel':{},
 'response-clear':{},'response-delta':{},start:{value:'0'},end:{value:'1'},zoom:{},reset:{}};
const document={getElementById:id=>elements[id],querySelectorAll:()=>[]};
const Plotly={newPlot(){},relayout(chart,changes){updated=changes;},restyle(chart,changes,indices){restyled={changes,indices};}};
const handlers=[...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].at(-1)[1];
vm.runInNewContext(handlers,{document,Plotly,Number,Object,String,JSON});
const choices=JSON.parse(elements['response-choices'].textContent).gyro;
elements['response-gyro'].value=String(choices[0].index);elements['response-gyro'].onchange();
assert.ok(restyled.indices.length===3);
assert.equal(restyled.changes.visible.filter(v=>v===true).length,1);
clicked({points:[{x:.2}]});clicked({points:[{x:.281}]});
assert.ok(elements['response-delta'].textContent.includes('81.00 ms'));
assert.equal(updated.shapes.length,2);
elements['response-clear'].onclick();assert.equal(updated.shapes.length,0);
assert.equal(elements['response-delta'].textContent,'A点を選択');
console.log('Response axes and manual A/B time difference passed (no browser rendering)');
'''
    result=subprocess.run([node,'-e',script,str(tmp_path/'report.html')],text=True,capture_output=True)
    assert result.returncode==0,result.stderr


def test_response_model_gets_full_resolution_samples_and_distinct_prediction_axis(tmp_path):
    from tools.pipeline_audit.report import write_report
    import re
    data=response_fixture()
    # Force display decimation while preserving the original samples for fitting.
    config=copy.deepcopy(CONFIG);config['plot_max_points']=100
    rows=[]
    for t in range(0,4000,5):
        rows.append(sample('final',t,t,dict(steering=(t%500)/1000.,throttle=.15,brake=0.,reverse=0.)))
        rows.append(sample('/realsense/imu',t+1,t-1,dict(frame_id='imu',gyro_y=-.5),kind='sensor_msgs/msg/Imu'))
    data['records']=rows
    write_report(tmp_path,*analyze(data,config))
    text=(tmp_path/'report.html').read_text()
    load=lambda key:json.loads(re.search(r'<script id="'+key+r'" type="application/json">(.*?)</script>',text).group(1))
    fit=load('response-fit-data');fig=load('figure')
    sensor=next(iter(fit['gyro'].values()))
    assert len(sensor['points'])==800
    assert sensor['points'][0][0]==.001
    assert fit['origin_bag_ns']==str(BASE)
    assert fig['data'][fit['model_index']]['yaxis']=='y2'
    assert fig['data'][fit['model_index']]['visible'] is False
    assert len(fig['data'][1]['x'])<=100
    assert '操舵と角速度から応答遅れをモデル推定' in text


def test_response_delay_model_portable_node_suite():
    node=shutil.which('node')
    if node is None:pytest.skip('Node needed for response estimator contracts')
    result=subprocess.run([node,'--test',str(Path(__file__).with_name('test_response_delay.cjs'))],capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stdout+result.stderr
