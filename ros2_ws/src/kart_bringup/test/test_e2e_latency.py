"""Portable contracts; no ROS, TensorRT or physical output is exercised here."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
import sys
from unittest.mock import Mock, patch
import subprocess
import pytest
import yaml
from kart_bringup.e2e_latency import plan, report
from kart_e2e.contract import SPEC

PACKAGE=Path(__file__).resolve().parents[1]
CONFIG=PACKAGE/'config'
ROOT=PACKAGE.parents[2]


@pytest.fixture
def inputs(tmp_path):
    bag=tmp_path/'bag';bag.mkdir()
    metadata=dict(rosbag2_bagfile_information=dict(topics_with_message_count=[
        dict(topic_metadata=dict(name=name,type=kind),message_count=10) for name,kind in [
            ('/realsense/color/image_raw','sensor_msgs/msg/Image'),
            ('/realsense/color/camera_info','sensor_msgs/msg/CameraInfo'),
            ('/vehicle/control_cmd','kart_interfaces/msg/ControlCommand'),
            ('/clock','rosgraph_msgs/msg/Clock')]]))
    (bag/'metadata.yaml').write_text(yaml.safe_dump(metadata))
    model=tmp_path/'model';model.mkdir();(model/'model.onnx').write_bytes(b'fixture')
    (model/'metadata.json').write_text(json.dumps(dict(schema=1,model_spec=SPEC,output_mode='steer_only',
        input_name='image',input_shape=[1,3,120,212],output_name='control',output_shape=[1,1],runtime=dict(fixed_throttle=.15,max_throttle=.3),
        onnx_sha256=hashlib.sha256(b'fixture').hexdigest())))
    return bag,model,tmp_path/'result'


def test_only_camera_replayed_private_clock_and_missing_inputs(inputs):
    bag,model,output=inputs
    run=plan(CONFIG,bag,model,output)
    argv=run['playback']
    assert argv[argv.index('--topics')+1:argv.index('--remap')]==['/realsense/color/image_raw','/realsense/color/camera_info']
    ns='/'+run['settings']['namespace']
    assert '/clock:='+ns+'/clock' in argv and '__ns:='+ns in argv
    assert '--start-paused' in argv and run['rate']==1
    assert not output.exists()
    for bad in ['nan','inf','0','-1','5']:
        with pytest.raises(ValueError):plan(CONFIG,bag,model,output,bad)
    with pytest.raises(ValueError):plan(CONFIG,bag,model,bag/'result')
    metadata=yaml.safe_load((bag/'metadata.yaml').read_text())
    metadata['rosbag2_bagfile_information']['topics_with_message_count'][0]['message_count']=0
    (bag/'metadata.yaml').write_text(yaml.safe_dump(metadata))
    with pytest.raises(ValueError,match='required'):plan(CONFIG,bag,model,output)


def test_reject_unreplayable_modality_or_section_model(inputs):
    with patch('kart_e2e.contract.input_contract',return_value=('evs','input_tensor',[1,20,120,160])):
        with pytest.raises(ValueError,match='RGB'):plan(CONFIG,*inputs)
    with patch('kart_e2e.contract.routing_settings',return_value={'heads':[1,2]}),patch('kart_e2e.contract.model_contract'),patch('kart_e2e.contract.input_contract',return_value=('rgb','image',[1,3,120,212])):
        with pytest.raises(ValueError,match='pose/head'):plan(CONFIG,*inputs)


def test_summary_excludes_warmup_repeated_unmatched_and_ambiguous(inputs):
    run=plan(CONFIG,*inputs);output=inputs[-1];output.mkdir()
    fields='event,elapsed_s,source_stamp_ns,latency_ms,first,warmup,steering,throttle,brake,reverse,detail'.split(',')
    records=[('input',0,1,'',0,1),('decoded',.01,1,10,1,1),
             ('input',2,2,'',0,0),('decoded',2.02,2,20,1,0),('command',2.03,2,30,1,0),
             ('command',2.04,2,40,0,0),('unmatched_command',2.05,999,'',0,0),
             ('input',3,3,'',0,0),('decoded',3.1,3,100,1,0),('ambiguous_input',3.2,3,'',0,0),
             ('input',4,4,'',0,0),('status',4.1,0,'',0,0)]
    with (output/'samples.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for record in records:writer.writerow(dict(zip(fields,record)))
    (output/'playback.json').write_text('{"returncode":0}')
    result=report(output,run,0)
    assert result['status']=='completed'
    assert result['stages']['decoded']['latency_ms']['p50']==20
    assert result['stages']['command']['latency_ms']['p50']==30
    assert result['stages']['command']['input_frames_without_observed_stage']==1
    assert result['command_republications']==1
    assert result['ambiguous_inputs']==1
    assert (output/'report.html').stat().st_size>100000
    assert report(output,run,130)['status']=='incomplete'
    assert 'plotly.js' in (output/'report.html').read_text()


def test_missing_csv_is_not_zero_latency(inputs):
    run=plan(CONFIG,*inputs);output=inputs[-1];output.mkdir()
    result=report(output,run,1)
    assert result['status']=='incomplete'
    assert result['stages']['decoded']['latency_ms']['mean'] is None


def test_tui_and_cli_select_same_entry(inputs):
    bag,model,output=inputs
    script=ROOT/'scripts/lib/bringup.py'
    for mode in [('e2e-latency',),('eval','--eval-localization','e2e_latency')]:
        command=[sys.executable,str(script),'--mode',*mode,'--bag',str(bag),'--e2e-model-dir',str(model),
                 '--latency-output',str(output),'--dry-run']
        result=subprocess.run(command,capture_output=True,text=True)
        assert result.returncode==0,result.stderr
        assert 'ros2 run kart_bringup e2e_latency' in result.stdout
        assert 'mission.launch' not in result.stdout and not output.exists()
    spec=importlib.util.spec_from_file_location('bringup_tui',script)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    selected=[]
    def choose(title,values,*args,**kwargs):
        selected.append((title,values))
        if title=='用途':return 'eval'
        if title=='オフライン解析':return 'e2e_latency'
        raise AssertionError(title)
    with patch.object(module,'choose',side_effect=choose),patch.object(sys.stdin,'isatty',return_value=True),patch.object(sys,'argv',[str(script),'--bag',str(bag),'--e2e-model-dir',str(model),'--latency-output',str(output),'--dry-run']):
        module.main()
    assert any('e2e_latency' in values for _,values in selected)


def test_launch_wiring_is_observation_only_and_private(inputs):
    lu=ModuleType('isaac_ros_launch_utils');lut=ModuleType('isaac_ros_launch_utils.all_types')
    lu.get_path=lambda package,suffix:PACKAGE/suffix
    lu.log_info=str;lu.component_container=Mock(return_value='container')
    lu.load_composable_nodes=lambda owner,nodes:('load',owner,nodes)
    lu.include=lambda package,path,**kw:('include',path,kw['launch_arguments'])
    lut.ComposableNode=lambda **kw:kw
    actions=ModuleType('launch.actions')
    for name in ['ExecuteProcess','RegisterEventHandler','TimerAction','EmitEvent','GroupAction']:
        setattr(actions,name,lambda *args,_name=name,**kw:(_name,args,kw))
    handlers=ModuleType('launch.event_handlers');handlers.OnProcessExit=lambda **kw:kw
    events=ModuleType('launch.events');events.Shutdown=lambda **kw:kw
    ros=ModuleType('launch_ros.actions');ros.PushRosNamespace=lambda name:('namespace',name)
    with patch.dict(sys.modules,{'isaac_ros_launch_utils':lu,'isaac_ros_launch_utils.all_types':lut,
        'launch.actions':actions,'launch.event_handlers':handlers,'launch.events':events,'launch_ros.actions':ros}):
        spec=importlib.util.spec_from_file_location('latency_launch',PACKAGE/'launch/e2e_latency.launch.py')
        mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    bag,model,output=inputs
    result=mod.add_latency(SimpleNamespace(bag=str(bag),model_dir=str(model),output_dir=str(output),rate=''))
    group=next(r for r in result if isinstance(r,tuple) and r[0]=='GroupAction')[1][0]
    includes=[r for r in group if isinstance(r,tuple) and r[0]=='include']
    assert len(includes)==2
    inference=includes[-1][2]
    assert inference['drive_enabled']=='false' and inference['recovery']=='off'
    ns=inference['node_namespace']
    assert ns.startswith('/e2e_latency_') and inference['clock_topic']==ns+'/clock'
    assert inference['publish_decoded_frames']=='true'
    probe=next(r for r in group if isinstance(r,tuple) and r[0]=='load')[2][0]
    params=probe['parameters'][1]
    assert len(params['ready_topics'])==10
    assert ns+'/e2e/tensor_output' in params['ready_topics']
    assert params['resume_service']==ns+'/rosbag2_player/resume'


def test_private_prefix_reaches_every_encoder_and_decoder_node(inputs):
    lu=ModuleType('isaac_ros_launch_utils');lut=ModuleType('isaac_ros_launch_utils.all_types')
    lu.get_path=lambda package,suffix:PACKAGE/suffix
    lu.log_info=str;lu.load_composable_nodes=Mock()
    lut.ComposableNode=lambda **kw:kw
    loaded=[]
    with patch.dict(sys.modules,{'isaac_ros_launch_utils':lu,'isaac_ros_launch_utils.all_types':lut}):
        for name in ('image_encoder','inference'):
            spec=importlib.util.spec_from_file_location('latency_'+name,PACKAGE/f'launch/modules/e2e/{name}.launch.py')
            mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);loaded.append(mod)
    encoder,inference=loaded
    ns='/e2e_latency_fixture'
    args=SimpleNamespace(container_name=ns+'/pipeline',model_dir=str(inputs[1]),image_topic=ns+'/input/image',
        camera_info_topic=ns+'/replay/camera_info',tensor_topic_prefix=ns+'/e2e',node_namespace=ns,clock_topic=ns+'/clock',
        use_sim_time='true',publish_decoded_frames='true',drive_enabled='false',recovery='off',
        use_fixed_throttle='true',fixed_throttle='',force_engine_update='',input_topic='')
    encoder.add_nodes(args)
    with patch.object(inference,'require_engine'):
        inference.add_nodes(args)
    nodes=[node for call in lu.load_composable_nodes.call_args_list for node in call.args[1]]
    assert len(nodes)==8
    for node in nodes:
        assert node['namespace']==ns
        assert ('/clock',ns+'/clock') in node['remappings']
        assert all(not target.startswith('/e2e/') for _,target in node['remappings'])
    decoder=nodes[-1]
    assert ('e2e/tensor_output',ns+'/e2e/tensor_output') in decoder['remappings']
    assert decoder['parameters'][0]['publish_decoded_frames'] is True
    assert decoder['parameters'][0]['drive_enabled'] is False


def test_partial_csv_and_absent_command_are_incomplete(inputs):
    run=plan(CONFIG,*inputs);output=inputs[-1];output.mkdir()
    (output/'playback.json').write_text('{"returncode":0}')
    (output/'samples.csv').write_text('event,elapsed_s,source_stamp_ns,latency_ms,first,warmup,steering,throttle,brake,reverse,detail\n'
        'input,1,101,,0,0,,,,,\n'
        'decoded,1.01,101,10,1,0,,,,,\n'
        'command,2.1')
    result=report(output,run,0)
    assert result['status']=='incomplete'
    assert result['stages']['decoded']['latency_ms']['count']==1
    assert result['stages']['command']['latency_ms']['mean'] is None
    assert result['malformed_csv_rows']==1


def test_output_namespace_is_different_for_parallel_sessions(inputs):
    bag,model,output=inputs
    a=plan(CONFIG,bag,model,output)
    b=plan(CONFIG,bag,model,output.with_name('other'))
    assert a['settings']['namespace']!=b['settings']['namespace']
    assert a['playback']!=b['playback']


def test_ctrl_c_stops_owned_launch_before_partial_report(inputs):
    from kart_bringup.e2e_latency import main
    import signal
    bag,model,output=inputs
    packages=ModuleType('ament_index_python.packages')
    packages.get_package_share_directory=lambda package:str(PACKAGE)
    class InterruptedOutput:
        def __iter__(self):
            yield 'fixture startup\n'
            raise KeyboardInterrupt()
    child=SimpleNamespace(pid=123456,stdout=InterruptedOutput(),poll=lambda:None,wait=Mock(return_value=130))
    with patch.dict(sys.modules,{'ament_index_python.packages':packages}),patch('kart_bringup.e2e_latency.subprocess.Popen',return_value=child),patch('kart_bringup.e2e_latency.os.killpg') as kill:
        code=main(['--bag',str(bag),'--model-dir',str(model),'--output',str(output)])
    assert code==130
    kill.assert_called_once_with(child.pid,signal.SIGINT)
    child.wait.assert_called_once_with(timeout=15)
    assert (output/'launch.log').read_text()=='fixture startup\n'
    assert json.loads((output/'summary.json').read_text())['status']=='incomplete'
    assert (output/'report.html').exists()
