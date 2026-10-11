"""cuVSLAM evaluation against kart_sim; simulation is launched separately."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from kart_bringup.replay import playback


def build(context):
    root=Path(get_package_share_directory('kart_bringup'))/'config'
    config=root/'sim/vslam.yaml'
    parameters=yaml.safe_load(config.read_text())['/**/visual_slam']['ros__parameters']
    value=LaunchConfiguration('tracking_mode').perform(context)
    overrides={}
    if value:
        if value not in ('0','1'): raise ValueError('tracking_mode must be 0 (VO) or 1 (VIO)')
        overrides['tracking_mode']=int(value)
    visualize=LaunchConfiguration('visualize').perform(context)
    if visualize:
        if visualize.lower() not in ('true','false'): raise ValueError('visualize must be true or false')
        overrides.update({name:visualize.lower()=='true' for name in
            ('enable_slam_visualization','enable_landmarks_view','enable_observations_view')})
    rviz=LaunchConfiguration('rviz').perform(context)
    if rviz and rviz.lower() not in ('true','false'): raise ValueError('rviz must be true or false')
    show_rviz=rviz.lower()=='true' if rviz else overrides.get('enable_slam_visualization',parameters['enable_slam_visualization'])
    bag=LaunchConfiguration('bag').perform(context)
    rate=LaunchConfiguration('rate').perform(context)
    if rate and not bag: raise ValueError('rate requires bag')
    mode=overrides.get('tracking_mode',parameters['tracking_mode'])
    command=playback(root,bag,rate,extra_topics=['/realsense/imu'] if mode==1 else []) if bag else None
    actions=[LogInfo(msg=f'cuVSLAM config={config}, overrides={overrides}; no saved map is loaded; live kart_sim publish_truth_tf must be false'),
        ComposableNodeContainer(name='kart_sim_vslam_container',namespace='',package='rclcpp_components',
            executable='component_container_mt',output='screen',on_exit=Shutdown(),composable_node_descriptions=[
                ComposableNode(package='isaac_ros_cuvslam',plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                    name='visual_slam',parameters=[str(config),overrides],remappings=[
                        ('visual_slam/image_0','/realsense/infra1/image_rect_raw'),
                        ('visual_slam/image_1','/realsense/infra2/image_rect_raw'),
                        ('visual_slam/camera_info_0','/realsense/infra1/camera_info'),
                        ('visual_slam/camera_info_1','/realsense/infra2/camera_info'),
                        ('visual_slam/imu','/realsense/imu')],
                    extra_arguments=[{'use_intra_process_comms':True}])])]
    if show_rviz:
        actions.append(Node(package='rviz2',executable='rviz2',name='rviz2',output='screen',
            arguments=['-d',str(root/'sim/vslam.rviz')],parameters=[str(root/'evaluation/rviz.yaml')]))
    if command:
        actions += [LogInfo(msg=f'Sim bag replay (filtered, initially paused): {command}'),
            ExecuteProcess(cmd=command,output='screen'),
            Node(package='kart_bringup',executable='replay_ready',name='replay_ready',output='screen',
                parameters=[str(root/'evaluation/replay_ready.yaml'),{'subscriber_nodes':['visual_slam']}])]
    return actions


def generate_launch_description():
    return LaunchDescription([*[DeclareLaunchArgument(name,default_value='') for name in
        ('tracking_mode','visualize','rviz','bag','rate')],OpaqueFunction(function=build)])
