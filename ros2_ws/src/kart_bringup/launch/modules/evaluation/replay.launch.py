"""Filtered bag player and startup readiness monitor; no physical drivers."""

import isaac_ros_launch_utils as lu
import isaac_ros_launch_utils.all_types as lut
from kart_bringup.replay import playback, imu_topics
from kart_bringup.mission import boolean
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def add_replay(args):
    root = lu.get_path("kart_bringup", "config")
    imu=imu_topics(root,args.tracking_mode)
    camera_info=getattr(args,"section_camera_info_topic","")
    command = playback(root,args.bag,args.rate,extra_topics=imu+([camera_info] if camera_info else []))
    overrides = (
        {} if boolean(args.enable_vgl) else {"subscriber_nodes": ["visual_slam"]}
    )
    if imu:overrides['imu_topic']=imu[0]
    player=ExecuteProcess(cmd=command,output="screen")
    completion=[]
    if boolean(getattr(args,'exit_when_done','false')):
        def finished(event,context):
            import json
            from pathlib import Path
            marker=getattr(args,'completion_file','')
            if marker:
                Path(marker).write_text(json.dumps(dict(returncode=event.returncode)))
            return [TimerAction(period=1.,actions=[EmitEvent(event=Shutdown(reason='Offline bag playback finished'))])]
        completion=[RegisterEventHandler(OnProcessExit(target_action=player,on_exit=finished))]
    return [
        *completion,
        lu.log_info(f"Replay (filtered, initially paused): {command}"),
        player,
        Node(
            package="kart_bringup",
            executable="replay_ready",
            name="replay_ready",
            parameters=[str(root / "evaluation/replay_ready.yaml"), overrides],
            output="screen",
        ),
    ]


def generate_launch_description():
    args = lu.ArgumentContainer()
    args.add_arg("section_camera_info_topic", "", cli=True)
    args.add_arg("exit_when_done", "false", cli=True)
    args.add_arg("completion_file", "", cli=True)
    args.add_arg("bag", cli=True)
    args.add_arg("enable_vgl", "true", cli=True)
    args.add_arg("rate", "", cli=True)
    args.add_arg("tracking_mode", "0", cli=True)
    args.add_opaque_function(add_replay)
    return lut.LaunchDescription(args.get_launch_actions())
