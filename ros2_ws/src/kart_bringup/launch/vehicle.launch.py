"""Operational values live in config/. Only explicit runtime arguments override them."""
from pathlib import Path
import isaac_ros_launch_utils as lu
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from kart_bringup.configuration import load, RUNTIME_KEYS


def start(context):
    config = LaunchConfiguration('config').perform(context)
    overrides = {key: LaunchConfiguration(key).perform(context) for key in RUNTIME_KEYS}
    settings, modules = load(config, overrides)
    namespace = settings['namespace']
    definitions = []
    if settings['enable_joy']:
        definitions += [
            ('joy', 'kart_joy', 'kart_joy::JoyNode', 'kart_joy_node', 'kart_joy_node'),
            ('joy_manager', 'kart_system', 'kart_system::JoyManagerNode', 'kart_joy_manager_node', 'kart_joy_manager'),
        ]
    if settings['enable_control']:
        definitions += [
            ('mode_manager', 'kart_system', 'kart_system::ModeManagerNode', 'kart_operation_mode_manager', 'operation_mode_manager'),
            ('command_mux', 'kart_system', 'kart_system::CommandMuxNode', 'kart_command_mux', 'command_mux'),
        ]
    if settings['enable_bridge']:
        definitions.append(('bridge', 'kart_vehicle', 'kart_vehicle::BridgeNode', 'kart_bridge_node', 'kart_bridge'))

    def parameters(key):
        module = modules[key]
        return [module['file']] + ([module['overrides']] if module['overrides'] else [])

    actions = [LogInfo(msg=f'Kart config: {config}; launch={settings}; explicit overrides={ {k:v for k,v in overrides.items() if v} }')]
    for key in [d[0] for d in definitions] + (['bag_manager'] if settings['enable_bag_manager'] else []):
        actions.append(LogInfo(msg=f"Kart {key}: {modules[key]['file']}; effective parameters={modules[key]['effective']}"))
    if settings['composed'] and definitions:
        components = [ComposableNode(package=pkg, plugin=plugin, name=name, namespace=namespace,
                                      parameters=parameters(key), extra_arguments=[{'use_intra_process_comms': True}])
                      for key, pkg, plugin, executable, name in definitions]
        actions.append(ComposableNodeContainer(name=settings['container_name'], namespace=namespace,
                       package='rclcpp_components', executable='component_container', output='screen',
                       composable_node_descriptions=components))
    else:
        actions += [Node(package=pkg, executable=executable, name=name, namespace=namespace,
                         output='screen', parameters=parameters(key))
                    for key, pkg, plugin, executable, name in definitions]
    if settings['enable_bag_manager']:
        actions.append(Node(package='kart_bag_manager', executable='kart_bag_manager_node',
                       name='kart_bag_manager', namespace=namespace, output='screen',
                       sigterm_timeout=str(settings['bag_shutdown_timeout']), sigkill_timeout=str(settings['bag_sigkill_timeout']),
                       parameters=parameters('bag_manager')))
    if settings['enable_jetson_stats']:
        actions.append(lu.include('kart_bringup', 'launch/modules/monitoring/jetson_stats.launch.py',
                                  scoped=True, forwarding=False))
    return actions


def generate_launch_description():
    config = Path(get_package_share_directory('kart_bringup')) / 'config/bringup.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(config)),
        # Empty is a sentinel: preserve the YAML value. Use namespace:=/ to explicitly select root.
        *[DeclareLaunchArgument(key, default_value='', description='Explicit runtime override; empty uses config')
          for key in RUNTIME_KEYS],
        OpaqueFunction(function=start),
    ])
