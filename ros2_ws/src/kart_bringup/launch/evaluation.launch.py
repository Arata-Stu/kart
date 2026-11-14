"""Bag-only localization review: optional output recording, no vehicle, sensor drivers, or control."""

import json

import isaac_ros_launch_utils as lu
from kart_bringup.launch_arguments import LaunchArguments
import isaac_ros_launch_utils.all_types as lut
import yaml
from kart_bringup.localization import resolve
from kart_bringup.mission import boolean, hdmap_choices
from kart_bringup.replay import playback, imu_topics, validate_bag_shape
from kart_bringup.lap_maintenance import evaluation_lap_parameters


def add_evaluation(args):
    root = lu.get_path("kart_bringup", "config")
    settings = yaml.safe_load((root / "evaluation/replay.yaml").read_text())
    lap_vgl = boolean(settings['lap_vgl'] if getattr(args, 'lap_vgl', '') == '' else args.lap_vgl)
    base_frame = args.base_frame if args.base_frame != "" else settings["base_frame"]
    overrides={"use_sim_time": "true", "base_frame": base_frame}
    overrides.update({k:getattr(args,k) for k in ('tracking_mode','imu_frame','multicam_mode') if getattr(args,k)!=""})
    parameters,_,_,profile=resolve(
        root / "localization",
        args.map_dir,
        args.model_dir,
        overrides,
        enable_vgl=boolean(args.enable_vgl),
    )
    mode=parameters['cuvslam']['tracking_mode']
    playback(root,args.bag,args.rate,extra_topics=imu_topics(root,mode))
    if boolean(args.enable_vgl):
        validate_bag_shape(root, args.bag, profile["input_shape"])
    if (args.lane_id, args.line_type) not in hdmap_choices(args.map_file):
        raise ValueError("Select a generated HDMap line")
    lap_parameters = evaluation_lap_parameters(root, args.map_file, args.lane_id, args.line_type,
        enabled=lap_vgl, enable_vgl=args.enable_vgl, base_frame=parameters['cuvslam'].get('base_frame'),
        odom_frame=parameters['cuvslam'].get('odom_frame'), map_frame=parameters['cuvslam'].get('map_frame'))
    if lap_parameters:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory('kart_lap_manager')
    rviz = settings['rviz'] if args.rviz == '' else boolean(args.rviz)

    def include(file, values):
        return lu.include(
            "kart_bringup",
            "launch/" + file,
            launch_arguments=values,
            scoped=True,
            forwarding=False,
        )

    from kart_bringup.evaluation_recording import recording_plan, start_recording
    record = recording_plan(root, getattr(args, "eval_record", ""), getattr(args, "eval_output", ""))
    actions = []
    if lap_parameters:
        from launch_ros.actions import Node
        actions += [lu.log_info(f"Offline lap VGL config: {lap_parameters}"),
                    Node(package='kart_lap_manager', executable='lap_manager', name='lap_manager',
                         parameters=[lap_parameters], output='screen')]
    if record:
        if lap_parameters:
            record['topics'] = list(dict.fromkeys(record['topics'] + [
                lap_parameters['lap_events_topic'], lap_parameters['lap_status_topic']]))
        for key in ("enable_slam_visualization", "enable_landmarks_view", "enable_observations_view"):
            parameters["cuvslam"][key] = True
        actions += start_recording(root, record, dict(source_bag=str(args.bag), rate=args.rate or settings["rate"],
            map_dir=args.map_dir, model_dir=args.model_dir, map_file=args.map_file, parameters=parameters,
            lap_vgl=lap_vgl, lap_parameters=lap_parameters))
    actions += [
        include(
            "localization.launch.py",
            {
                "map_dir": args.map_dir,
                "model_dir": args.model_dir,
                "use_sim_time": "true",
                "visualize": "true",
                "base_frame": base_frame,
                "enable_vgl": args.enable_vgl,
                "tracking_mode": str(mode),
                "imu_frame": args.imu_frame,
                "multicam_mode": args.multicam_mode,
            },
        ),
        include(
            "hdmap.launch.py",
            {
                "map_file": args.map_file,
                "lane_id": args.lane_id,
                "line_type": args.line_type,
                "use_sim_time": "true",
            },
        ),
    ]
    if rviz:
        actions.append(include("modules/evaluation/rviz.launch.py", {}))
    actions.append(
        include(
            "modules/evaluation/replay.launch.py",
            {"bag": args.bag, "rate": args.rate, "enable_vgl": args.enable_vgl,"tracking_mode":str(mode),
             "recorder_node": record["node_name"] if record else "",
             "record_topics_json": json.dumps(record["topics"] if record else []),
             "lap_node": 'lap_manager' if lap_parameters else '',
             "lap_input_topics_json": json.dumps([lap_parameters[key] for key in (
                 'reference_topic', 'odometry_topic', 'diagnostics_topic')]) if lap_parameters else '',
             "lap_status_topic": lap_parameters['lap_status_topic'] if lap_parameters else ''},
        )
    )
    return actions


def generate_launch_description():
    args = LaunchArguments()
    for key in ("bag", "map_dir", "map_file", "lane_id", "line_type"):
        args.add_arg(key, cli=True)
    args.add_arg("model_dir", "", cli=True)
    args.add_arg("enable_vgl", "false", cli=True)
    args.add_arg('lap_vgl', '', cli=True)
    args.add_arg("rate", "", cli=True)
    args.add_arg("base_frame", "", cli=True)
    args.add_arg("multicam_mode", "", cli=True)
    args.add_arg("tracking_mode", "", cli=True)
    args.add_arg("imu_frame", "", cli=True)
    args.add_arg("rviz", "", cli=True)
    args.add_arg("eval_record", "", cli=True)
    args.add_arg("eval_output", "", cli=True)
    args.add_opaque_function(add_evaluation)
    return lut.LaunchDescription(args.get_launch_actions())
