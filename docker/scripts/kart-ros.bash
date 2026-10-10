# Sourced by interactive Bash inside the kart development container.
if [ -f /opt/ros/lyrical/setup.bash ]; then
    source /opt/ros/lyrical/setup.bash
fi
if [ -f /workspaces/ros2_ws/install/setup.bash ]; then
    source /workspaces/ros2_ws/install/setup.bash
fi

# Convenience command available from any directory in interactive container Bash.
webui() {
    bash /workspaces/scripts/webui.sh "$@"
}
