"""ROS launch arguments stay text until a configuration helper validates them.

Isaac's ArgumentContainer evaluates Python literals. That loses leading zeroes
in identifiers and changes OFF/false values before callbacks can interpret them.
Use ROS substitutions directly; keep the small add_arg API used by our launches.
"""

from types import SimpleNamespace


class LaunchArguments:
    def __init__(self):
        self._configurations = {}
        self._declarations = []
        self._callbacks = []

    def add_arg(self, name, default=None, *, description=None, choices=None, cli=False):
        from launch.actions import DeclareLaunchArgument
        from launch.substitutions import LaunchConfiguration

        default = None if default is None else str(default)
        configuration = LaunchConfiguration(name, default=default)
        self._configurations[name] = configuration
        setattr(self, name, configuration)
        if cli:
            self._declarations.append(DeclareLaunchArgument(
                name, default_value=default, description=description, choices=choices))
        return configuration

    def add_opaque_function(self, function, condition=None):
        from launch.actions import OpaqueFunction

        def evaluate(context):
            values = {key: value.perform(context) for key, value in self._configurations.items()}
            return function(SimpleNamespace(**values))

        action = OpaqueFunction(function=evaluate, condition=condition)
        self._callbacks.append(action)
        return action

    def get_launch_actions(self):
        return self._declarations + self._callbacks
