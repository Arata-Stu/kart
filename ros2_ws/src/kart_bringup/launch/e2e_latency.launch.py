"""Camera-only replay -> production RGB encoder/TRT/decoder -> private observation.

No bridge, Joy, command mux, recovery, physical sensor or vehicle command publisher.
The session CLI creates the output directory and reports after all processes exit.
"""
import json
from pathlib import Path
import isaac_ros_launch_utils as lu
from kart_bringup.launch_arguments import LaunchArguments
import isaac_ros_launch_utils.all_types as lut
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, EmitEvent, GroupAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import PushRosNamespace
from kart_bringup.e2e_latency import plan


def add_latency(args):
    config = lu.get_path('kart_bringup', 'config')
    run = plan(config, args.bag, args.model_dir, args.output_dir, args.rate)
    namespace = run['settings']['namespace']
    prefix = '/' + namespace + '/e2e'
    container = '/' + namespace + '/pipeline'
    pipeline = [
        ('input/image', 'e2e_resize'), ('replay/camera_info', 'e2e_resize'),
        ('e2e/resize/image', 'e2e_format'), ('e2e/rgb/image', 'e2e_to_tensor'),
        ('e2e/image_tensor', 'e2e_normalize'), ('e2e/normalized_tensor', 'e2e_planar'),
        ('e2e/planar_tensor', 'e2e_reshape'), ('e2e/tensor_input', 'e2e_tensor_rt'),
        ('e2e/tensor_output', 'e2e_control_decoder'),
        ('operation_mode/state', 'e2e_control_decoder'),
    ]
    probe = lut.ComposableNode(
        name='e2e_latency_probe', namespace='/' + namespace, package='kart_e2e',
        plugin='kart_e2e::LatencyProbeNode', parameters=[str(config/'evaluation/e2e_latency.yaml'),
          dict(csv_path=str(Path(args.output_dir)/'samples.csv'),
               resume_service='/'+namespace+'/rosbag2_player/resume',
               ready_topics=['/'+namespace+'/'+topic for topic, _ in pipeline],
               ready_nodes=[name for _, name in pipeline])],
        remappings=[("/clock", "/"+namespace+"/clock")],
        extra_arguments=[{'use_intra_process_comms': True}])
    owner = lu.component_container('pipeline', container_type='multithreaded')
    player = ExecuteProcess(cmd=run['playback'], output='screen')
    def finished(event, context):
        Path(args.output_dir, 'playback.json').write_text(json.dumps(dict(returncode=event.returncode)))
        return [TimerAction(period=run['settings']['drain_s'], actions=[EmitEvent(event=Shutdown(reason='E2E latency replay finished'))])]
    return [
        lu.log_info(f'E2E latency: {run}; original stamps + simulated ROS clock; measurements use steady time'),
        RegisterEventHandler(OnProcessExit(target_action=player, on_exit=finished)),
        RegisterEventHandler(OnProcessExit(target_action=owner, on_exit=[EmitEvent(event=Shutdown(reason='E2E latency container exited'))])),
        GroupAction([
            PushRosNamespace(namespace), owner,
            lu.load_composable_nodes(container, [probe]),
            lu.include('kart_bringup', 'launch/modules/e2e/image_encoder.launch.py',
                launch_arguments=dict(container_name=container, image_topic='/'+namespace+'/input/image',
                                      camera_info_topic='/'+namespace+'/replay/camera_info',
                                      use_sim_time='true', tensor_topic_prefix=prefix,
                                      node_namespace='/'+namespace, clock_topic='/'+namespace+'/clock'), scoped=True, forwarding=False),
            lu.include('kart_bringup', 'launch/modules/e2e/inference.launch.py',
                launch_arguments=dict(container_name=container, model_dir=args.model_dir,
                                      use_sim_time='true', drive_enabled='false', recovery='off',
                                      use_fixed_throttle=str(run['settings']['use_fixed_throttle']).lower(),
                                      tensor_topic_prefix=prefix, publish_decoded_frames='true',
                                      node_namespace='/'+namespace, clock_topic='/'+namespace+'/clock'), scoped=True, forwarding=False),
        ]),
        player,
    ]


def generate_launch_description():
    args = LaunchArguments()
    for key in ('bag', 'model_dir', 'output_dir'):
        args.add_arg(key, cli=True)
    args.add_arg('rate', '', cli=True)
    args.add_opaque_function(add_latency)
    return lut.LaunchDescription(args.get_launch_actions())
