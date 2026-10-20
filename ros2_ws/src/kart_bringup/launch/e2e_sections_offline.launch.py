"""Offline registered-map poses for section datasets. No vehicle/control/sensor drivers."""
import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from launch_ros.actions import Node
from kart_e2e.sections import load, validate_maps
from kart_bringup.localization import resolve
from kart_bringup.replay import playback, imu_topics


def add_offline(args):
    root=lu.get_path('kart_bringup','config')
    routing=load(args.routing_file)
    validate_maps(routing,args.map_file,args.map_dir)
    params,_,_,_=resolve(root/'localization',args.map_dir,'',
                         dict(use_sim_time='true',base_frame=routing['base_frame'],tracking_mode=str(args.tracking_mode)),enable_vgl=False)
    playback(root,args.bag,args.rate,extra_topics=imu_topics(root,str(args.tracking_mode))+[args.camera_info_topic])
    selector=Node(package='kart_e2e',executable='e2e_section_selector',name='e2e_section_selector',
                  output='screen',parameters=[str(root/'e2e/section_selector.yaml'),
                  dict(use_sim_time=True,routing_file=args.routing_file,trajectory_output=args.trajectory_output,
                       camera_info_topic=args.camera_info_topic)])
    return [selector,
        lu.include('kart_bringup','launch/localization.launch.py',scoped=True,forwarding=False,
                   launch_arguments=dict(map_dir=args.map_dir,model_dir='',enable_vgl='false',use_sim_time='true',
                                         base_frame=routing['base_frame'],tracking_mode=str(args.tracking_mode))),
        lu.include('kart_bringup','launch/modules/evaluation/replay.launch.py',scoped=True,forwarding=False,
                   launch_arguments=dict(bag=args.bag,rate=args.rate,enable_vgl='false',tracking_mode=str(args.tracking_mode),
                                         section_camera_info_topic=args.camera_info_topic,
                                         exit_when_done=getattr(args,"exit_when_done","false"),
                                         completion_file=getattr(args,"completion_file","")))]


def generate_launch_description():
    args=lu.ArgumentContainer()
    for key in ('bag','map_file','map_dir','routing_file','trajectory_output'):
        args.add_arg(key,cli=True)
    args.add_arg('exit_when_done','false',cli=True)
    args.add_arg('completion_file','',cli=True)
    args.add_arg('tracking_mode','0',cli=True)
    args.add_arg('rate','',cli=True)
    args.add_arg('camera_info_topic','/realsense/color/camera_info',cli=True)
    args.add_opaque_function(add_offline)
    return lut.LaunchDescription(args.get_launch_actions())
