"""Saved-map closed-loop controller; simulator is owned by terminal 1."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from kart_bringup.mission import hdmap_choices
from kart_bringup.localization import resolve
from kart_bringup.vgl_assets import inspect_engines


def build(context):
    root=Path(get_package_share_directory('kart_bringup'))
    values={key:LaunchConfiguration(key).perform(context) for key in
            ('map_file','map_dir','model_dir','lane_id','line_type','enable_vgl')}
    if values['enable_vgl'] not in ('true','false'): raise ValueError('Invalid enable_vgl')
    if (values['lane_id'],values['line_type']) not in hdmap_choices(values['map_file']):
        raise ValueError('HDMap lane/line unavailable')
    resolve(root/'config/localization',values['map_dir'],values['model_dir'],
            {'use_sim_time':True,'base_frame':'base_link'},inspect_engines,
            enable_vgl=values['enable_vgl']=='true')
    from kart_bringup.recovery import route
    recovery_remaps, recovery_actions = route(root/'config/recovery', 'true',
                                               root/'config/sim/drive_recovery.yaml')
    components=[]
    for name,pkg,plugin in (
        ('operation_mode_manager','kart_system','kart_system::ModeManagerNode'),
        ('command_mux','kart_system','kart_system::CommandMuxNode'),
        ('pure_pursuit','kart_control','kart_control::PurePursuitNode'),
        ('speed_controller','kart_control','kart_control::SpeedControllerNode')):
        components.append(ComposableNode(package=pkg,plugin=plugin,name=name,
            parameters=[str(root/'config/sim'/f'drive_{name}.yaml')],
            remappings=recovery_remaps if name=='speed_controller' else [],
            extra_arguments=[{'use_intra_process_comms':True}]))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/localization.launch.py')),
                launch_arguments={**{k:values[k] for k in ('map_dir','model_dir','enable_vgl')},
                                  'use_sim_time':'true','base_frame':'base_link'}.items()),
            Node(package='kart_hdmap',executable='hdmap_server',name='hdmap_server',output='screen',
                 parameters=[str(root/'config/sim/drive_hdmap.yaml'),
                    {k:values[k] for k in ('map_file','lane_id','line_type')}]),
            ComposableNodeContainer(name='kart_sim_control_container',namespace='',package='rclcpp_components',
                executable='component_container_mt',output='screen',on_exit=Shutdown(),
                composable_node_descriptions=components), *recovery_actions]


def generate_launch_description():
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in
        [('map_file',''),('map_dir',''),('model_dir',''),('lane_id',''),('line_type','centerline'),('enable_vgl','false')]],OpaqueFunction(function=build)])
