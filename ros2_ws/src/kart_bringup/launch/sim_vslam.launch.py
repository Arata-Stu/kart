"""cuVSLAM evaluation against kart_sim; simulation is launched separately."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def build(context):
    config=Path(get_package_share_directory('kart_bringup'))/'config/sim/vslam.yaml'
    value=LaunchConfiguration('tracking_mode').perform(context)
    overrides={}
    if value:
        if value not in ('0','1'): raise ValueError('tracking_mode must be 0 (VO) or 1 (VIO)')
        overrides['tracking_mode']=int(value)
    return [LogInfo(msg=f'cuVSLAM config={config}, overrides={overrides}; kart_sim publish_truth_tf must be false'),
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


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('tracking_mode',default_value=''),OpaqueFunction(function=build)])
