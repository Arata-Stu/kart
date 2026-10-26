#!/usr/bin/env python3
"""Searchable fzf terminal selector; exec ROS directly so Ctrl-C reaches ros2 launch."""

import argparse
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [
    str(ROOT / "ros2_ws/src/kart_bringup"),
    str(ROOT / "ros2_ws/src/kart_hdmap"),
    str(ROOT / "ros2_ws/src/kart_e2e"),
    str(ROOT / "ros2_ws/src/kart_bag_manager"),
]


def choose(title, values, label=str, default=0):
    if not values:
        raise ValueError(
            f"{title}: 候補がありません。探索先や地図/モデルの準備状態を確認してください"
        )
    if not 0 <= default < len(values):
        raise ValueError("選択肢の既定値が範囲外です")
    executable = shutil.which("fzf")
    if not executable:
        raise ValueError(
            "fzfがありません。Dockerイメージを再ビルドするか、"
            "コンテナ内で sudo apt-get install -y fzf を実行してください"
        )
    # Keep IDs separate from labels: duplicate names and paths with whitespace are valid.
    order = [default] + [i for i in range(len(values)) if i != default]
    rows = {}
    for i in order:
        display = "".join(c if c.isprintable() else " " for c in str(label(values[i])))
        row = f"{i}\t{display}" + (" [既定]" if i == default else "")
        rows[row] = values[i]
    env = dict(os.environ)
    # User fzf bindings/commands must not alter selection or execute unrelated actions.
    for key in ("FZF_DEFAULT_OPTS", "FZF_DEFAULT_OPTS_FILE", "FZF_DEFAULT_COMMAND"):
        env.pop(key, None)
    result = subprocess.run(
        [executable, "--read0", "--print0", "--delimiter=\t", "--with-nth=2..",
         "--no-multi", "--no-sort", "--layout=reverse", "--height=60%", "--border",
         "--prompt", f"{title} > ",
         "--header", "↑↓ 選択 / 文字入力で検索 / Enter 決定 / Esc・Ctrl-C 中止"],
        input="\0".join(rows) + "\0", stdout=subprocess.PIPE,
        text=True, encoding="utf-8", env=env, check=False,
    )
    if result.returncode in (1, 130, -2):
        raise KeyboardInterrupt
    if result.returncode != 0:
        raise ValueError(f"fzfの実行に失敗しました（exit={result.returncode}）")
    selected = result.stdout.removesuffix("\0")
    if selected not in rows:
        raise ValueError("fzfが不正な選択結果を返しました")
    print(f"{title}: {selected.split(chr(9), 1)[1]}")
    return rows[selected]


def latency_entry(args, config, interactive):
    from kart_bringup.e2e_latency import plan
    from datetime import datetime
    if args.evs or args.recovery or args.auto_fixed_throttle is not None or args.e2e_fixed_throttle is not None:
        raise ValueError('E2E latencyはRGB bag・モデルのみを使います。実センサ／復帰／スロットル上書き指定を外してください')
    if not args.bag or not args.e2e_model_dir:
        raise ValueError('E2E latencyには--bagと--e2e-model-dirが必要です')
    output = args.latency_output or str(ROOT/'analysis/e2e_latency'/datetime.now().strftime('%Y-%m-%d/%H-%M-%S-%f'))
    run = plan(config, args.bag, args.e2e_model_dir, output, args.rate)
    command = ['ros2', 'run', 'kart_bringup', 'e2e_latency', '--bag', run['bag'],
               '--model-dir', run['model_dir'], '--output', run['output_dir'], '--rate', str(run['rate'])]
    print('E2E offline latency: RGB → 実運用encoder/TensorRT/decoder。実センサ・車両出力なし。')
    print('STOP warmup → 計測専用AUTO。元stampは保持し、所要時間はsteady clockで計測。')
    print('全pipelineの購読準備後に再生開始。完了／Ctrl-CでCSV・JSON・時系列HTMLを保存。')
    print('保存先:', run['output_dir'])
    print(shlex.join(command))
    if args.dry_run:
        return
    if interactive:
        choose('この構成を起動', [True, False], lambda v: '起動' if v else '終了') or sys.exit(0)
    if not shutil.which('ros2'):
        raise ValueError('ROSコンテナ内でscripts/build.shを実行してから起動してください')
    os.execvp(command[0], command)


def main():
    parser = argparse.ArgumentParser(
        description="kart: 収集 / 地図走行 / E2E / offline評価。--mode指定時は非対話。"
    )
    parser.add_argument("--mode", choices=("collect", "environment", "drive", "e2e", "eval", "e2e-latency"))
    parser.add_argument("--map-root", type=Path, default=ROOT / "map")
    parser.add_argument("--model-root", type=Path, default=ROOT / "models")
    parser.add_argument("--rgb-fps", choices=("0", "30", "60", "90"))
    parser.add_argument("--infra-fps", choices=("0", "30", "60", "90"))
    parser.add_argument("--rgb-profile", default="", help="RGB WIDTHxHEIGHTxFPS; 0 disables")
    parser.add_argument("--imu-profile", choices=("low", "high", "off"), default="", help="IMU: low=accel100/gyro200Hz, high=accel200/gyro400Hz, off=なし; 未指定はYAML")
    parser.add_argument("--infra-profile", default="", help="Infra WIDTHxHEIGHTxFPS; 0 disables")
    for key in ("device", "map-file", "map-dir", "model-dir", "lane-id", "line-type"):
        parser.add_argument("--" + key, default="")
    parser.add_argument("--tuning", action=argparse.BooleanOptionalAction, default=None,
                        help="driveでMap Studioの動的調整を有効化（--no-tuningで無効化）")
    parser.add_argument("--record-dir", default=str(ROOT / "record"))
    parser.add_argument(
        "--evs",
        action="store_true",
        help="SilkyEvCam direct Tensor・画像・RAW連携",
    )
    parser.add_argument("--no-evs", action="store_true", help="EVSを無効にする")
    parser.add_argument("--evs-backend", choices=("cpu", "cuda", "cuda_async"), default="")
    parser.add_argument("--evs-aggregation", choices=("events", "evt3_runs", "evt3_runs_gpu", "evt3_runs_gpu_direct", "evt3_runs_grouped_gpu"), default="", help="cuda_asyncの集計方式。指定時は未指定backendをcuda_asyncにする")
    parser.add_argument("--evs-image", action=argparse.BooleanOptionalAction, default=None,
                        help="イベント可視化画像の生成をON/OFF。未指定はYAMLを保持")
    parser.add_argument("--evs-serial", default="")
    parser.add_argument("--evs-tensor-config", default="", help="Model-derived full OpenEB tensor YAML")
    parser.add_argument("--evs-bias-file", default="", help=".bias path; @defaultでカメラ既定")
    parser.add_argument("--bias-root", type=Path, default=ROOT / "bias/evs")
    parser.add_argument("--foxglove", action="store_true")
    parser.add_argument("--no-bridge", action="store_true", help="車両基板なしの確認用")
    parser.add_argument(
        "--dry-run", action="store_true", help="選択・検証とコマンド表示のみ"
    )
    parser.add_argument("--auto-fixed-throttle", type=float, default=None,
                        help="Collection AUTO throttle, manual steering (collect/drive): -1 normal, 0..speed_scale fixed (not vehicle speed)")
    parser.add_argument("--e2e-fixed-throttle", type=float, default=None,
                        help="E2E model steering + fixed throttle; defaults to model runtime metadata")
    from kart_bringup.recovery import CHOICES, LABELS, configuration as recovery_configuration, validate_inputs
    parser.add_argument("--recovery", choices=CHOICES, default="",
                        help="自動復帰: off / infra_imu / rgb_imu / evs_assisted（未指定はYAML）")
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--e2e-model-dir", default="")
    parser.add_argument("--eval-localization", choices=("vslam", "vgl", "fresh", "e2e_latency"), default=None)
    parser.add_argument("--multicam-mode", choices=("0","1","2"), default="", help="速度・精度: 0=Moderate / 1=Performance / 2=Precision")
    parser.add_argument("--tracking-mode", choices=("0", "1"), default="", help="offline評価: 0=stereo VO / 1=VIO")
    parser.add_argument("--base-frame", default="", help="offline評価の基準TF frame（例: camera_link）")
    parser.add_argument("--imu-frame", default="", help="offline VIO評価のIMU TF frame")
    parser.add_argument("--latency-output", default="", help="E2E offline latencyの新規保存先（既定analysis/e2e_latency/日時）")
    parser.add_argument("--bag", default="")
    parser.add_argument("--rate", default="")
    args = parser.parse_args()
    from kart_bringup.evs import BACKENDS, BACKEND_LABELS, AGGREGATIONS, AGGREGATION_LABELS, DEFAULT_BIAS, bias_files, bias_selection, pipeline_configuration
    selected_evs = args.evs or any((args.evs_backend, args.evs_serial, args.evs_bias_file, args.evs_tensor_config, args.evs_aggregation, args.evs_image is not None))
    if args.no_evs and selected_evs:
        raise ValueError("--no-evsとEVS指定は併用できません")
    args.evs = selected_evs
    if args.evs_aggregation:
        if args.evs_backend and args.evs_backend != "cuda_async":
            raise ValueError("--evs-aggregationは--evs-backend cuda_asyncと組み合わせてください")
        args.evs_backend = "cuda_async"
    from kart_bringup.mission import discover, hdmap_choices, sensor_parameters, camera_config, recording_config

    interactive = args.mode is None
    if interactive and not sys.stdin.isatty():
        parser.error("TUIには端末が必要です。非対話では--modeを指定してください")
    config = ROOT / "ros2_ws/src/kart_bringup/config"
    if interactive:
        args.mode = choose(
            "用途",
            ["collect", "environment", "drive", "e2e", "eval"],
            lambda m: {
                "collect": "データ収集 → 後でoffline mapping",
                "environment": "環境記録（高解像度RGB・左右Infra・Depth・IMU）",
                "drive": "地図走行（VSLAM + VGL + ライン追従）",
                "e2e": "E2E（TensorRT、localizationなし）",
                "eval": "offline bag評価（VO / VIO / 保存map localization・RViz）",
            }[m],
        )
        if args.mode != "eval":
            sensor = sensor_parameters(camera_config(config, args.mode))
            for kind, key in (
                ("rgb", "rgb_camera.color_profile"),
                ("infra", "depth_module.infra_profile"),
            ):
                if getattr(args, kind + "_fps") is None and not getattr(args, kind + "_profile"):
                    from kart_bringup.mission import CAMERA_PROFILES
                    profiles = list(dict.fromkeys([sensor[key], *CAMERA_PROFILES["RGB" if kind == "rgb" else "Infra"], "0"]))
                    enabled = sensor["enable_color" if kind == "rgb" else "enable_infra1"]
                    setattr(args, kind + "_profile", choose(
                        ("Infra（有効時はDepthも同じprofile）" if kind == "infra" and args.mode == "environment" else kind.upper()) + " 解像度 × FPS（USB3 D455候補）",
                        profiles,
                        label=lambda v: "なし" if v == "0" else v.replace("x", " × "),
                        default=profiles.index(sensor[key] if enabled else "0"),
                    ))
            if not args.imu_profile:
                current = ("off" if not sensor["enable_accel"] and not sensor["enable_gyro"] else
                           "low" if (sensor["accel_fps"], sensor["gyro_fps"]) == (100, 200) else
                           "high" if (sensor["accel_fps"], sensor["gyro_fps"]) == (200, 400) else "")
                choices = ["", "low", "high", "off"]
                args.imu_profile = choose("IMU 取得レート", choices,
                    label=lambda v: {"": "設定を保持（YAML）", "low": "低Hz（加速度100 / ジャイロ200Hz）",
                                     "high": "高Hz（加速度200 / ジャイロ400Hz）", "off": "なし"}[v],
                    default=choices.index(current))
            if args.mode == "drive" and (args.infra_fps == "0" or args.infra_profile == "0"):
                raise ValueError("VSLAM/VGLにはInfraが必要です（なしは選択できません）")
            args.evs = not args.no_evs and (args.evs or choose(
                "EVS（Tensor・画像・RAW記録）",
                [False, True],
                lambda v: "有効" if v else "無効",
            ))
            if args.evs:
                if not args.evs_backend:
                    import yaml
                    default_backend = yaml.safe_load((config / "sensors/openeb_tensor_pipeline.yaml").read_text())["/**/tensor_pipeline"]["ros__parameters"]["tensor_backend"]
                    args.evs_backend = choose("EVS Tensor処理方式", list(BACKENDS),
                        label=lambda v: BACKEND_LABELS[v], default=BACKENDS.index(default_backend))
                if args.evs_backend == "cuda_async" and not args.evs_aggregation:
                    import yaml
                    source = Path(args.evs_tensor_config).expanduser() if args.evs_tensor_config else config / "sensors/openeb_tensor_cuda_async.yaml"
                    current = yaml.safe_load(source.read_text())["/**/event_tensor"]["ros__parameters"].get("event_aggregation", "events")
                    args.evs_aggregation = choose("EVS 集計方式", list(AGGREGATIONS),
                        label=lambda v: AGGREGATION_LABELS[v], default=AGGREGATIONS.index(current))
                if args.evs_image is None:
                    import yaml
                    image_default = yaml.safe_load((config / "sensors/openeb_visualization.yaml").read_text())["/**/event_preprocessor"]["ros__parameters"]["event_image_enabled"]
                    args.evs_image = choose("EVS event image可視化", [False, True],
                        label=lambda v: "ON（画像生成あり）" if v else "OFF（画像生成なし・Tensorのみ）",
                        default=int(image_default))
                if not args.evs_bias_file:
                    args.evs_bias_file = str(choose("EVS bias file", [DEFAULT_BIAS] + bias_files(args.bias_root), lambda p: "カメラ既定" if p == DEFAULT_BIAS else str(p)))
            args.foxglove = args.foxglove or choose(
                "Foxglove Bridge", [False, True], lambda v: "有効" if v else "無効"
            )
        if args.mode == "eval" and args.eval_localization is None:
            args.eval_localization = choose(
                "オフライン解析", (["vslam_vio"] if args.tracking_mode == "1" else ["vslam"] if args.tracking_mode == "0" else ["vslam", "vslam_vio"]) + ["vgl"] +
                (["fresh_vio" if args.tracking_mode == "1" else "fresh_vo"] if args.tracking_mode else ["fresh_vo", "fresh_vio"]) + ["e2e_latency"],
                lambda v: {"e2e_latency": "E2E latency（RGB bag → TensorRT → 制御topic、車両出力なし）", "vslam": "保存mapでVO localization", "vslam_vio": "保存mapでVIO localization（左右画像＋IMU）", "vgl": "VSLAM＋VGL（準備済みbundle）",
                           "fresh_vo": "mapなしVO評価（左右画像・特徴点群・軌跡）",
                           "fresh_vio": "mapなしVIO評価（左右画像＋IMU・特徴点群・軌跡）"}[v])
            if args.eval_localization == "vslam_vio":
                args.tracking_mode="1"
                args.eval_localization="vslam"
            elif args.eval_localization == "vslam" and not args.tracking_mode:
                args.tracking_mode="0"
            if args.eval_localization in ("fresh_vo", "fresh_vio"):
                args.tracking_mode = "1" if args.eval_localization == "fresh_vio" else "0"
                args.eval_localization = "fresh"
        if args.mode == "eval" and args.eval_localization == "e2e_latency":
            args.mode = "e2e-latency"
        if args.mode == "eval" and args.eval_localization in ("fresh", "vgl") and not args.tracking_mode:
            args.tracking_mode = choose("追跡方式", ["0", "1"], lambda v: "stereo VO（IMUなし）" if v == "0" else "VIO（IMUあり）")
        if args.mode in ("drive","eval") and not args.multicam_mode:
            import yaml
            from kart_bringup.multicam import LABELS
            preset='sim/vslam.yaml' if args.mode=='eval' and args.eval_localization=='fresh' else 'localization/cuvslam.yaml'
            default=str(yaml.safe_load((config/preset).read_text())['/**/visual_slam']['ros__parameters']['multicam_mode'])
            args.multicam_mode=choose("VSLAM速度・精度",list(LABELS),lambda v:LABELS[v],default=list(LABELS).index(default))
        if args.mode == "drive" or (args.mode == "eval" and args.eval_localization != "fresh"):
            args.map_file = args.map_file or str(
                choose("HDMap", discover(args.map_root, "hdmap"))
            )
            options = hdmap_choices(args.map_file)
            options = [
                x
                for x in options
                if (not args.lane_id or x[0] == args.lane_id)
                and (not args.line_type or x[1] == args.line_type)
            ]
            args.lane_id, args.line_type = choose(
                "lane / 追従ライン", options, lambda x: " / ".join(x)
            )
            vslam_only = args.mode == "eval" and args.eval_localization == "vslam"
            args.map_dir = args.map_dir or str(
                choose(
                    "VSLAM地図（HDMapと同じ座標系）" if vslam_only else "同じ座標系のVSLAM/VGL bundle（HDMapとは別に選択）",
                    discover(args.map_root, "vslam" if vslam_only else "bundle"),
                )
            )
            models = sorted(
                set(
                    discover(args.model_root, "models")
                    + discover(args.map_root, "models")
                )
            )
            args.model_dir = "" if vslam_only else args.model_dir or str(
                choose("VGLモデル（実行GPU用engine）", models)
            )
        if args.mode == "drive" and args.tuning is None:
            import yaml
            default_tuning = yaml.safe_load((config / "hdmap/hdmap.yaml").read_text())["/**/hdmap_server"]["ros__parameters"]["tuning_enabled"]
            args.tuning = choose("Map Studio 動的調整", [False, True],
                                 lambda enabled: "有効（HDMap・自己位置表示、停止中に編集を適用）" if enabled else "無効（通常走行）",
                                 default=int(default_tuning))
        if args.mode in ("e2e", "e2e-latency"):
            candidates = sorted(
                set(
                    discover(args.model_root, "e2e")
                    + discover(ROOT / "e2e/models", "e2e")
                )
            )
            args.e2e_model_dir = args.e2e_model_dir or str(
                choose("E2E ONNXモデル（metadata付き）", candidates)
            )
        if args.mode in ("eval", "e2e-latency"):
            args.bag = args.bag or str(
                choose("再生rosbag", discover(args.record_dir, "bag"))
            )
        elif args.run_name is None:
            args.run_name = input("run_name [run] > ").strip() or "run"
    if args.mode == "eval" and args.eval_localization == "e2e_latency":
        args.mode = "e2e-latency"
    if args.mode == "e2e-latency":
        return latency_entry(args, config, interactive)
    if args.latency_output:
        raise ValueError("--latency-outputはe2e-latency専用です")
    section_routing = None
    if args.mode == 'e2e':
        from kart_e2e.contract import routing_settings
        section_routing = routing_settings(args.e2e_model_dir)
        if section_routing and interactive:
            args.map_file = args.map_file or str(choose('sectionと同じHDMap', discover(args.map_root, 'hdmap')))
            args.map_dir = args.map_dir or str(choose('sectionと同じVSLAM地図', discover(args.map_root, 'vslam')))
        if section_routing:
            from kart_e2e.sections import validate_maps
            validate_maps(section_routing, args.map_file, args.map_dir)
    if args.tuning is not None and args.mode != 'drive':
        raise ValueError('--tuning / --no-tuningはdrive専用です')
    tuning_enabled = False
    tuning_map_dir = ''
    if args.mode == 'drive':
        import yaml
        settings = yaml.safe_load((config / 'hdmap/hdmap.yaml').read_text())["/**/hdmap_server"]["ros__parameters"]
        tuning_enabled = settings['tuning_enabled'] if args.tuning is None else args.tuning
        if tuning_enabled:
            from kart_bringup.studio_tuning import map_root
            tuning_map_dir = map_root(args.map_file, args.map_dir)
        print('Map Studio 動的調整: ' + ('有効 / 地図: ' + tuning_map_dir if tuning_enabled else '無効'))
    if args.mode not in ('collect', 'environment', 'drive') and args.auto_fixed_throttle is not None:
        raise ValueError('--auto-fixed-throttleは収集用（ステア手動）です。collect/environment/driveで指定してください。E2Eは--e2e-fixed-throttleを使用します')
    if args.e2e_fixed_throttle is not None and args.mode != 'e2e':
        raise ValueError('--e2e-fixed-throttleはe2e専用です')
    if interactive and args.mode in ('collect', 'environment', 'drive') and args.auto_fixed_throttle is None:
        from kart_bringup.configuration import load
        _, modules = load(config / 'bringup.yaml')
        fixed = modules['joy_manager']['effective']['auto_fixed_throttle']
        limit = modules['joy_manager']['effective']['speed_scale']
        selection = choose('AUTOスロットル', ['normal', 'fixed'],
                           lambda v: '通常AUTO（既存制御）' if v == 'normal' else '固定値で収集（ステア手動）',
                           default=int(fixed >= 0))
        if selection == 'normal':
            args.auto_fixed_throttle = -1.0
        else:
            default = fixed if fixed >= 0 else ''
            raw = input(f'固定スロットル 0..{limit}' + (f' [{default}]' if default != '' else '') + ' > ').strip()
            if not raw and default == '':
                raise ValueError('固定スロットルの値を入力してください')
            args.auto_fixed_throttle = float(raw or default)
    if args.recovery and args.mode not in ('drive', 'e2e'):
        raise ValueError('--recoveryはdrive/e2e専用です')
    if interactive and args.mode in ('drive', 'e2e') and not args.recovery:
        enabled, recovery_params = recovery_configuration(config / 'recovery')
        default = recovery_params['sensor_profile'] if enabled else 'off'
        args.recovery = choose('壁接触・スタック自動復帰', list(CHOICES),
                               lambda v: LABELS[v], default=CHOICES.index(default))
    if args.multicam_mode and args.mode not in ('drive','eval'):
        raise ValueError('--multicam-modeはVSLAMを使用するdrive/evalで指定してください')
    fresh = args.mode == "eval" and args.eval_localization == "fresh"
    if (args.base_frame or args.imu_frame or args.tracking_mode) and args.mode != "eval":
        raise ValueError("--base-frame/--imu-frame/--tracking-modeはoffline evalで使用してください")
    if fresh and any((args.map_file, args.map_dir, args.model_dir, args.lane_id, args.line_type)):
        raise ValueError("mapなし診断では地図・モデル・ラインを指定しないでください")
    if args.mode == "eval" and args.evs:
        raise ValueError("evalでは実EVSを起動しません")
    if args.imu_profile and args.mode in ("eval", "e2e-latency"):
        raise ValueError("--imu-profileは実機のcollect/environment/drive/e2eで使用してください")
    if args.evs:
        paths, overrides = pipeline_configuration(config / "sensors", args.evs_backend, args.evs_serial, args.evs_bias_file, args.evs_tensor_config, args.evs_aggregation, "" if args.evs_image is None else str(args.evs_image).lower())
        if args.evs_bias_file and args.evs_bias_file != DEFAULT_BIAS:
            args.evs_bias_file = bias_selection(args.evs_bias_file)
        print(f"EVS: {overrides}; packet topic OFF; configs={paths}")
    sensors = sensor_parameters(
        camera_config(config, args.mode), args.rgb_fps or "", args.infra_fps or "",
        args.rgb_profile, args.infra_profile, args.imu_profile
    )
    if section_routing:
        from kart_bringup.localization import resolve
        from kart_bringup.vehicle_tf import load_transforms
        load_transforms(config / 'vehicle/transforms.yaml', require_camera=True)
        if not sensors['enable_infra1'] or not sensors['enable_infra2']:
            raise ValueError('Section E2E needs stereo Infra for localization')
        localization_params, _, _, _ = resolve(config / 'localization', args.map_dir, '',
                                                  {'base_frame': section_routing['base_frame']}, enable_vgl=False)
        from kart_bringup.mission import validate_localization_imu
        validate_localization_imu(localization_params, sensors)
    if args.mode in ("drive", "eval") and not fresh:
        if args.mode == "drive" and not sensors["enable_infra1"]:
            raise ValueError("VSLAM/VGLにはInfraが必要です（なしは選択できません）")
        vslam_only = args.mode == "eval" and args.eval_localization != "vgl"
        if not args.map_file or not args.map_dir or (not vslam_only and not args.model_dir):
            raise ValueError(
                "drive/evalには--map-file / --map-dir / --model-dirが必要です（TUIなら一覧選択）"
            )
        if (args.lane_id, args.line_type) not in hdmap_choices(args.map_file):
            raise ValueError("HDMapに存在する--lane-id / --line-typeを選んでください")
        from kart_bringup.localization import resolve

        overrides={'multicam_mode':args.multicam_mode} if args.multicam_mode else {}
        if args.mode=="eval":
            import yaml
            base=args.base_frame or yaml.safe_load((config/'evaluation/replay.yaml').read_text())['base_frame']
            overrides.update({'base_frame':base,'use_sim_time':True})
            overrides.update({k:getattr(args,k) for k in ('tracking_mode','imu_frame') if getattr(args,k)})
        parameters, _, _, profile = resolve(
            config / "localization", args.map_dir, args.model_dir, overrides=overrides, enable_vgl=not vslam_only
        )
        if args.mode == "drive":
            from kart_bringup.mission import validate_localization_imu
            validate_localization_imu(parameters, sensors)
        if args.mode=="eval":args.tracking_mode=str(parameters['cuvslam']['tracking_mode'])
        if not vslam_only:
            if args.mode == "eval":
                from kart_bringup.replay import validate_bag_shape
                sizes = validate_bag_shape(config, args.bag, profile["input_shape"])
                print(f"VGL bag入力解像度確認（先頭左右画像・CameraInfo）: {sizes}")
            else:
                width, height, _ = sensors["depth_module.infra_profile"].split("x")
                if profile["input_shape"][2:] != [int(height), int(width)]:
                    raise ValueError("VGLモデルとInfra解像度が一致しません")
    if args.mode != "eval":
        from kart_bringup.configuration import load

        _, modules = load(config / "bringup.yaml", {
            "enable_bridge": str(not args.no_bridge).lower(), "device": args.device,
            "auto_fixed_throttle": '-1' if args.mode == 'e2e' else ('' if args.auto_fixed_throttle is None else str(args.auto_fixed_throttle)),
        })
        if not args.no_bridge:
            print(f"車両基板: {modules['bridge']['effective']['device']}")
        fixed = modules['joy_manager']['effective']['auto_fixed_throttle']
        if args.mode != 'e2e':
            print('AUTOスロットル: ' + ('通常AUTO（既存制御）、MANUALはR2/L2' if fixed < 0 else
              f'固定 {fixed:.6f}（□でAUTO、△で通常MANUALへ直接切替、ステア手動、○/L2/×で解除・STOP）'))
    if args.mode in ('drive', 'e2e'):
        recovery_enabled, recovery_params = recovery_configuration(config / 'recovery', args.recovery)
        if recovery_enabled:
            validate_inputs(recovery_params, sensors, args.evs, fixed)
        print('自動復帰: ' + (f"{recovery_params['sensor_profile']} ({recovery_params['image_topic']})" if recovery_enabled else '無効'))
    if args.mode == "e2e":
        from kart_e2e.contract import model_contract, runtime_settings, input_contract

        modality, _, _ = input_contract(args.e2e_model_dir)
        if modality == 'evs':
            from kart_bringup.e2e import validate_evs_tensor
            if not args.evs:
                raise ValueError('EVS E2Eには--evsが必要です')
            validate_evs_tensor(args.evs_tensor_config, args.e2e_model_dir)
        elif not sensors["enable_color"]:
            raise ValueError("RGB E2EにはRGBが必要です")
        _, mode = model_contract(args.e2e_model_dir)
        runtime = runtime_settings(args.e2e_model_dir)
        import math
        if interactive and args.e2e_fixed_throttle is None:
            raw = input(f'E2E固定スロットル 0..{runtime["max_throttle"]} [{runtime["fixed_throttle"]}] > ').strip()
            args.e2e_fixed_throttle = float(raw) if raw else runtime['fixed_throttle']
        fixed = runtime['fixed_throttle'] if args.e2e_fixed_throttle is None else args.e2e_fixed_throttle
        if not math.isfinite(fixed) or not 0 <= fixed <= runtime['max_throttle']:
            raise ValueError('E2E固定スロットルは有限値で0..モデルのmax_throttle以内にしてください')
        print(f"E2E AUTO: モデルのステア + 固定スロットル {fixed}; max={runtime['max_throttle']}; model={mode}")
    if args.mode != "eval":
        from kart_bag_manager.recorder import Settings

        Settings(recording_name=args.run_name or "run").validate()
    else:
        from kart_bringup.replay import playback, imu_topics

        playback(config,args.bag,args.rate,extra_topics=imu_topics(config,args.tracking_mode or "0"))
    values = {
        "mode": args.mode,
        "recovery": args.recovery,
        "auto_fixed_throttle": None if args.auto_fixed_throttle is None else str(args.auto_fixed_throttle),
        "e2e_fixed_throttle": None if args.e2e_fixed_throttle is None else str(args.e2e_fixed_throttle),
        "rgb_profile": args.rgb_profile,
        "infra_profile": args.infra_profile,
        "imu_profile": args.imu_profile,
        "rgb_fps": args.rgb_fps,
        "infra_fps": args.infra_fps,
        "enable_bridge": str(not args.no_bridge).lower(),
        "device": args.device,
        "enable_evs": str(args.evs).lower(),
        "evs_backend": args.evs_backend,
        "evs_serial": args.evs_serial,
        "evs_bias_file": args.evs_bias_file,
        "evs_tensor_config": args.evs_tensor_config,
        "evs_aggregation": args.evs_aggregation,
        "evs_image": "" if args.evs_image is None else str(args.evs_image).lower(),
        "enable_foxglove": str(args.foxglove).lower(),
        "record_dir": args.record_dir,
        "run_name": args.run_name or "run",
        "multicam_mode": args.multicam_mode,
    }
    if args.mode in ("drive", "eval") and not fresh:
        values.update(
            {
                k: getattr(args, k)
                for k in ("map_file", "map_dir", "model_dir", "lane_id", "line_type")
            }
        )
    if args.mode == "drive":
        values.update(tuning_enabled=str(tuning_enabled).lower(), tuning_map_dir=tuning_map_dir)
    if args.mode == "e2e":
        values["e2e_model_dir"] = args.e2e_model_dir
        if section_routing:
            values.update(map_file=args.map_file, map_dir=args.map_dir)
    launch = "mission.launch.py"
    if args.mode == "eval":
        launch = "evaluation.launch.py"
        values = {
            k: getattr(args, k)
            for k in (
                "map_file",
                "map_dir",
                "model_dir",
                "lane_id",
                "line_type",
                "bag",
                "rate",
                "base_frame",
                "imu_frame",
                "tracking_mode",
                "multicam_mode",
            )
        }
    if fresh:
        launch = "sim_vslam.launch.py"
        values = {"bag": args.bag, "rate": args.rate, "tracking_mode": args.tracking_mode,
                  "base_frame": args.base_frame, "imu_frame": args.imu_frame,
                  "multicam_mode": args.multicam_mode,
                  "visualize": "true", "rviz": "true"}
    elif args.mode == "eval":
        values["enable_vgl"] = str(args.eval_localization == "vgl").lower()
    command = ["ros2", "launch", "kart_bringup", launch] + [
        f"{k}:={v}" for k, v in values.items() if v is not None and v != ""
    ]
    if args.mode != "eval":
        print(
            "\n起動構成:",
            args.mode,
            "RGB:",
            sensors["rgb_camera.color_profile"] if sensors["enable_color"] else "OFF",
            "Infra:",
            sensors["depth_module.infra_profile"]
            if sensors["enable_infra1"]
            else "OFF",
        )
    if args.mode != "eval":
        print("IMU:", f"加速度{sensors['accel_fps']} / ジャイロ{sensors['gyro_fps']}Hz"
              if sensors["enable_accel"] and sensors["enable_gyro"] else "OFF")
    if args.mode == "environment":
        print("環境記録: Depth Z16:", sensors["depth_module.depth_profile"],
              "IMU: accel=", sensors["enable_accel"], "gyro=", sensors["enable_gyro"],
              "unite_imu_method=", sensors["unite_imu_method"])
        print("センサ設定:", camera_config(config, args.mode))
        print("録画設定:", recording_config(config, args.mode, args.evs))
    print(shlex.join(command))
    if fresh:
        print("mapを読み込まず新規SLAMで追跡。RViz: 左右画像・観測点・ランドマーク・軌跡。終了はCtrl-C。")
    if args.mode == "eval":
        print(
            "実センサ・車両・制御は起動しません。RVizで確認。入力購読準備後に指定速度で再生（既定1倍）。終了はCtrl-C。"
        )
    else:
        print(
            f"保存先: {args.record_dir}/<日付>/<起動時刻>/{args.run_name or 'run'}（最初のSTARTで生成）"
        )
        print("録画はJoy設定の開始/停止ボタンで操作（標準bringupはR1開始/L1停止）。起動では録画・AUTOを開始しません。終了はCtrl-C。")
    if args.mode == "drive":
        print(
            "車体→カメラTFは別途必要です。仮TFは配信しません。舵角・PIDの実車設定も確認してください。"
        )
    if args.dry_run:
        return
    if interactive:
        choose(
            "この構成を起動", [True, False], lambda v: "起動" if v else "終了"
        ) or sys.exit(0)
    if not shutil.which("ros2"):
        raise ValueError(
            "ros2がありません。scripts/dev.shでROSコンテナへ入り、scripts/build.shでビルドしてください"
        )
    os.execvp(command[0], command)


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("\n中止しました", file=sys.stderr)
        sys.exit(130)
    except ImportError as error:
        print(
            f"依存が不足しています: {error}。ROS開発コンテナで実行してください（python3-yamlが必要です）",
            file=sys.stderr,
        )
        sys.exit(2)
    except (ValueError, OSError) as error:
        print(f"エラー: {error}", file=sys.stderr)
        sys.exit(2)
