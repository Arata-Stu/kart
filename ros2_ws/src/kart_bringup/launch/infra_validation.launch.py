from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration


def start(context):
    mode = LaunchConfiguration('mode').perform(context)
    topics = ['/validation/infra_stamps']
    if mode == 'with_camera':
        for side in ('infra1', 'infra2'):
            topics.extend([f'/realsense/{side}/image_rect_raw', f'/realsense/{side}/camera_info'])
    output = LaunchConfiguration('output').perform(context)
    recorder = ['ros2', 'bag', 'record', '-s', 'mcap']
    if output:
        recorder.extend(['-o', output])
    return [
        ExecuteProcess(cmd=[*recorder, '--topics', *topics], output='screen', on_exit=Shutdown()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='results_only',
                              choices=['results_only', 'with_camera']),
        DeclareLaunchArgument('output', default_value=''),
        OpaqueFunction(function=start),
    ])
