from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('shutdown_timeout', default_value='20'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('kart_bag_manager'), 'config', 'bag_manager.yaml'])),
        Node(package='kart_bag_manager', executable='kart_bag_manager_node', name='kart_bag_manager',
             namespace=LaunchConfiguration('namespace'), output='screen',
             sigterm_timeout=LaunchConfiguration('shutdown_timeout'), sigkill_timeout='5',
             parameters=[LaunchConfiguration('config')]),
    ])
