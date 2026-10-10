#!/usr/bin/env python3
"""Numbered terminal selector; exec ROS directly so Ctrl-C reaches ros2 launch."""

import argparse
import glob
import os
import shlex
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
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
    print(f"\n{title}")
    for i, value in enumerate(values, 1):
        print(f"  {i}. {label(value)}" + (" [既定]" if i - 1 == default else ""))
    while True:
        answer = input("番号 / Enter=既定 / q=終了 > ").strip()
        if answer.lower() == "q":
            raise KeyboardInterrupt
        if not answer:
            return values[default]
        if answer.isdigit() and 1 <= int(answer) <= len(values):
            return values[int(answer) - 1]
        print("一覧の番号を入力してください")


def main():
    parser = argparse.ArgumentParser(
        description="kart: 収集 / 地図走行 / E2E / offline評価。--mode指定時は非対話。"
    )
    parser.add_argument("--mode", choices=("collect", "drive", "e2e", "eval"))
    parser.add_argument("--map-root", type=Path, default=ROOT / "map")
    parser.add_argument("--model-root", type=Path, default=ROOT / "models")
    parser.add_argument("--rgb-fps", choices=("0", "30", "60", "90"))
    parser.add_argument("--infra-fps", choices=("0", "30", "60", "90"))
    for key in ("device", "map-file", "map-dir", "model-dir", "lane-id", "line-type"):
        parser.add_argument("--" + key, default="")
    parser.add_argument("--record-dir", default=str(ROOT / "record"))
    parser.add_argument(
        "--evs",
        action="store_true",
        help="SilkyEvCam direct Tensor・画像・RAW連携",
    )
    parser.add_argument("--no-evs", action="store_true", help="EVSを無効にする")
    parser.add_argument("--evs-backend", choices=("cpu", "cuda", "cuda_async"), default="")
    parser.add_argument("--evs-serial", default="")
    parser.add_argument("--evs-bias-file", default="", help=".bias path; @defaultでカメラ既定")
    parser.add_argument("--bias-root", type=Path, default=ROOT / "bias/evs")
    parser.add_argument("--foxglove", action="store_true")
    parser.add_argument("--no-bridge", action="store_true", help="車両基板なしの確認用")
    parser.add_argument(
        "--dry-run", action="store_true", help="選択・検証とコマンド表示のみ"
    )
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--e2e-model-dir", default="")
    parser.add_argument("--bag", default="")
    parser.add_argument("--rate", default="")
    args = parser.parse_args()
    from kart_bringup.evs import BACKENDS, DEFAULT_BIAS, bias_files, bias_selection, pipeline_configuration
    selected_evs = args.evs or any((args.evs_backend, args.evs_serial, args.evs_bias_file))
    if args.no_evs and selected_evs:
        raise ValueError("--no-evsとEVS指定は併用できません")
    args.evs = selected_evs
    from kart_bringup.mission import discover, hdmap_choices, sensor_parameters

    interactive = args.mode is None
    if interactive and not sys.stdin.isatty():
        parser.error("TUIには端末が必要です。非対話では--modeを指定してください")
    config = ROOT / "ros2_ws/src/kart_bringup/config"
    if interactive:
        args.mode = choose(
            "用途",
            ["collect", "drive", "e2e", "eval"],
            lambda m: {
                "collect": "データ収集 → 後でoffline mapping",
                "drive": "地図走行（VSLAM + VGL + ライン追従）",
                "e2e": "E2E（TensorRT、localizationなし）",
                "eval": "bagでlocalization確認（Notebook・RViz）",
            }[m],
        )
        if args.mode != "eval":
            sensor = sensor_parameters(config / "sensors/realsense.yaml")
            for kind, key in (
                ("rgb", "rgb_camera.color_profile"),
                ("infra", "depth_module.infra_profile"),
            ):
                if getattr(args, kind + "_fps") is None:
                    rates = ["30", "60", "90", "0"]
                    default = rates.index(sensor[key].split("x")[-1])
                    setattr(
                        args,
                        kind + "_fps",
                        choose(
                            kind.upper() + " Hz（機種の対応profileを使用）",
                            rates,
                            label=lambda v: "なし" if v == "0" else v,
                            default=default,
                        ),
                    )
            if args.mode == "drive" and args.infra_fps == "0":
                raise ValueError("VSLAM/VGLにはInfraが必要です（なしは選択できません）")
            if args.mode == "e2e" and args.rgb_fps == "0":
                raise ValueError("E2EにはRGBが必要です")
            args.evs = not args.no_evs and (args.evs or choose(
                "EVS（Tensor・画像・RAW記録）",
                [False, True],
                lambda v: "有効" if v else "無効",
            ))
            if args.evs:
                if not args.evs_backend:
                    import yaml
                    default_backend = yaml.safe_load((config / "sensors/openeb_tensor_pipeline.yaml").read_text())["/**/tensor_pipeline"]["ros__parameters"]["tensor_backend"]
                    args.evs_backend = choose("EVS decoder", list(BACKENDS), default=BACKENDS.index(default_backend))
                if not args.evs_bias_file:
                    args.evs_bias_file = str(choose("EVS bias file", [DEFAULT_BIAS] + bias_files(args.bias_root), lambda p: "カメラ既定" if p == DEFAULT_BIAS else str(p)))
            args.foxglove = args.foxglove or choose(
                "Foxglove Bridge", [False, True], lambda v: "有効" if v else "無効"
            )
            if args.mode != "eval" and not args.no_bridge and not args.device:
                ports = sorted(
                    set(
                        glob.glob("/dev/serial/by-id/*")
                        + glob.glob("/dev/ttyACM*")
                        + glob.glob("/dev/ttyUSB*")
                    )
                )
                device = choose(
                    "車両基板",
                    ports + [None],
                    lambda p: str(p) if p else "基板なし（確認用）",
                )
                args.device, args.no_bridge = device or "", device is None
        if args.mode in ("drive", "eval"):
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
            args.map_dir = args.map_dir or str(
                choose(
                    "同じ座標系のVSLAM/VGL bundle（HDMapとは別に選択）",
                    discover(args.map_root, "bundle"),
                )
            )
            models = sorted(
                set(
                    discover(args.model_root, "models")
                    + discover(args.map_root, "models")
                )
            )
            args.model_dir = args.model_dir or str(
                choose("VGLモデル（実行GPU用engine）", models)
            )
        if args.mode == "e2e":
            candidates = sorted(
                set(
                    discover(args.model_root, "e2e")
                    + discover(ROOT / "e2e/models", "e2e")
                )
            )
            args.e2e_model_dir = args.e2e_model_dir or str(
                choose("E2E ONNXモデル（metadata付き）", candidates)
            )
        if args.mode == "eval":
            args.bag = args.bag or str(
                choose("再生rosbag", discover(args.record_dir, "bag"))
            )
        elif args.run_name is None:
            args.run_name = input("run_name [run] > ").strip() or "run"
    if args.mode == "eval" and args.evs:
        raise ValueError("evalでは実EVSを起動しません")
    if args.evs:
        paths, overrides = pipeline_configuration(config / "sensors", args.evs_backend, args.evs_serial, args.evs_bias_file)
        if args.evs_bias_file and args.evs_bias_file != DEFAULT_BIAS:
            args.evs_bias_file = bias_selection(args.evs_bias_file)
        print(f"EVS: {overrides}; packet topic OFF; configs={paths}")
    sensors = sensor_parameters(
        config / "sensors/realsense.yaml", args.rgb_fps or "", args.infra_fps or ""
    )
    if args.mode in ("drive", "eval"):
        if not sensors["enable_infra1"]:
            raise ValueError("VSLAM/VGLにはInfraが必要です（なしは選択できません）")
        if not args.map_file or not args.map_dir or not args.model_dir:
            raise ValueError(
                "drive/evalには--map-file / --map-dir / --model-dirが必要です（TUIなら一覧選択）"
            )
        if (args.lane_id, args.line_type) not in hdmap_choices(args.map_file):
            raise ValueError("HDMapに存在する--lane-id / --line-typeを選んでください")
        from kart_bringup.localization import resolve

        _, _, _, profile = resolve(
            config / "localization", args.map_dir, args.model_dir
        )
        width, height, _ = sensors["depth_module.infra_profile"].split("x")
        if profile["input_shape"][2:] != [int(height), int(width)]:
            raise ValueError("VGLモデルとInfra解像度が一致しません")
    if args.mode != "eval" and not args.no_bridge and not args.device:
        raise ValueError("車両基板の--deviceが必要です。基板なしなら--no-bridge")
    if args.mode == "e2e":
        from kart_e2e.contract import model_contract, runtime_settings

        if not sensors["enable_color"]:
            raise ValueError("E2EにはRGBが必要です")
        _, mode = model_contract(args.e2e_model_dir)
        runtime = runtime_settings(args.e2e_model_dir)
        print(f"E2E metadata: {mode}; {runtime}")
    if args.mode != "eval":
        from kart_bag_manager.recorder import Settings

        Settings(recording_name=args.run_name or "run").validate()
    else:
        from kart_bringup.replay import playback

        playback(config, args.bag, args.rate)
    values = {
        "mode": args.mode,
        "rgb_fps": args.rgb_fps,
        "infra_fps": args.infra_fps,
        "enable_bridge": str(not args.no_bridge).lower(),
        "device": args.device,
        "enable_evs": str(args.evs).lower(),
        "evs_backend": args.evs_backend,
        "evs_serial": args.evs_serial,
        "evs_bias_file": args.evs_bias_file,
        "enable_foxglove": str(args.foxglove).lower(),
        "record_dir": args.record_dir,
        "run_name": args.run_name or "run",
    }
    if args.mode in ("drive", "eval"):
        values.update(
            {
                k: getattr(args, k)
                for k in ("map_file", "map_dir", "model_dir", "lane_id", "line_type")
            }
        )
    if args.mode == "e2e":
        values["e2e_model_dir"] = args.e2e_model_dir
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
            )
        }
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
    print(shlex.join(command))
    if args.mode == "eval":
        print(
            "実センサ・車両・制御は起動しません。RVizで確認。入力購読準備後に指定速度で再生（既定1倍）。終了はCtrl-C。"
        )
    else:
        print(
            f"保存先: {args.record_dir}/<日付>/<起動時刻>/{args.run_name or 'run'}（最初のSTARTで生成）"
        )
        print("録画はL1開始/R1停止。起動ではAUTOに切り替えません。終了はCtrl-C。")
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
            "ros2がありません。scripts/dev.shでROSコンテナへ入り、scripts/workspace/build.shでビルドしてください"
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
