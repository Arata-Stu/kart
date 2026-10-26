# bringupパラメータ伝達監査（2026-10-26）

対象は`bash scripts/bringup.sh` → `scripts/lib/bringup.py`から到達する
collect / environment / drive / RGB E2E / EVS E2E / offline eval / E2E latency。
CLIの全42 argument destination（help・検索root・dry-runを含む）を分類した。
実機には接続せず、ソース・設定・CLI dry-run・launch callback代替テストで調査した。

## 確認した不具合と対応

| 問題 | 調査結果・対応 |
|---|---|
| RGB/Infra「なし」 | 既修正。Isaac `ArgumentContainer`で文字列`0`が整数0となり、未指定扱いでYAMLのtrueが残っていた。sensor helperでも入力を文字列化。 |
| EVS画像・bias・serialのoverride | 既修正。`Node(name="tensor_pipeline")`のparameter辞書は親ノード専用になり、内部driver/preprocessorへ届かなかった。プロセスのname指定を外し、overrideは`/**`へ適用。内部ノード名・namespaceはプログラムが保持。 |
| 数字・Pythonリテラルに見える識別子 | 今回修正。数値serialはdriverの文字列parameterへ数値として渡り得た。例：`run_name:=0000`は整数0→文字列`0`へ変わる。全19箇所のIsaac argument評価を`LaunchArguments`へ変更し、元の文字列を保持。数値・bool・JSONは設定helperで明示変換。 |
| 使用しないモードのオプション | 今回修正。offlineのカメラ設定・録画名・device・Foxglove、実機モードのbag/rate、通常収集の地図/モデル指定等がコマンド組立て時に消えていた。CLI段階でエラーにする。sectionなしE2Eの地図指定、VSLAM単独評価のVGLモデル指定も拒否。 |
| `0`／`false`の他経路 | 固定AUTO/E2Eスロットル、multicam/tracking mode、動的調整OFFは明示的な空文字判定で保持される。親子callback経由の回帰テストを追加。JSONの自動変換済み値への対応は既存実装にあり、さらに文字列保持へ統一。 |

`LaunchArguments`は標準ROSの`DeclareLaunchArgument`・`LaunchConfiguration`・
`OpaqueFunction`を用い、値をPython式として評価しない。静的YAMLの既定値を複製しない。

## オプション別の伝達先

以下の「確認」はソース・代替実行での伝達確認。ドライバ・GPU・実機での反映確認とは区別する。

| CLI入力 | 親launch／変換 | 最終設定または責務 |
|---|---|---|
| `--mode` | mission / evaluation / sim_vslam、latencyは専用CLIへ分岐 | 起動するノード群の選択 |
| `--rgb-profile`, `--infra-profile` | mission → realsense、`sensor_parameters` | color/infra profileとenable flags。`0`はOFF。Depth有効時はInfra profileをDepthへも適用 |
| `--rgb-fps`, `--infra-fps` | 同上（同一streamのprofileと併用禁止） | 静的解像度を保持してFPSだけ変更。`0`はOFF |
| `--imu-profile` | mission → realsense | accel/gyroのenable、rate、unite method |
| `--evs`, `--no-evs` | mission `enable_evs` | OpenEB processと対応録画YAMLの選択 |
| `--evs-backend` | mission → openeb → `tensor_backend` | selectorがCPU/CUDA/非同期Tensorノードを選択 |
| `--evs-aggregation` | mission → openeb → `event_aggregation` | selector／event_tensorの集計方式。モデル由来Tensor YAMLを保持 |
| `--evs-image`, `--no-evs-image` | mission → openeb → `event_image_enabled` | event_preprocessor。OFFなら画像publisherを生成せず、購読者がいても画像を配信しない |
| `--evs-serial` | mission → openeb → `serial` | event_camera_driver。文字列を保持 |
| `--evs-bias-file` | mission → openeb → `bias_file` | driverがカメラ開始前に読込み。`@default`は明示空パス、未指定はYAML保持 |
| `--evs-tensor-config` | mission → openeb、およびEVS E2E契約検証 | 全Tensor設定ファイル。input shape・正規化・topic契約を検証 |
| `--device` | mission → vehicle → `configuration.load` | bridge.device |
| `--no-bridge` | mission → vehicle `enable_bridge=false` | 実機bridgeを起動しない。offlineでは元から起動しないため互換的に許容 |
| `--record-dir` | mission → vehicle `record_dir` | bag_manager.output_dir。offline TUIではbag探索root |
| `--run-name` | mission → vehicle `run_name` | bag_manager.recording_name。先頭ゼロを保持 |
| `--auto-fixed-throttle` | mission → vehicle | joy_manager.auto_fixed_throttle、command_mux.auto_use_teleop。値0も明示適用 |
| `--e2e-model-dir` | mission → RGB/EVS E2E `model_dir` → inference | TensorRT model/engine path、decoder runtime、section routing。latencyでは専用CLI `--model-dir` |
| `--e2e-fixed-throttle` | mission `e2e_fixed_throttle` → E2E → inference `fixed_throttle` | decoder.fixed_throttle。値0も適用 |
| `--recovery` | mission → tracking/speed_controller、またはE2E/inference | recoveryノード・入力remap・sensor profile選択。観測専用E2Eはproducerを作らない |
| `--map-file` | drive → tracking → hdmap、eval → hdmap | hdmap_server.map_file。section E2Eでは地図一致検証・section selectorの参照に使用 |
| `--map-dir`, `--model-dir` | mission/evaluation → localization → vslam/vgl | 保存地図とVGLモデル。section E2Eはmap_dirだけを使用 |
| `--lane-id`, `--line-type` | drive/eval → hdmap | 初期追従lane／ライン |
| `--tuning`, `--no-tuning` | mission → tracking → hdmap | tuning_enabled。ON時は対応地図からtuning_map_dirを導出 |
| `--multicam-mode` | drive/eval → localization → vslam、fresh → sim_vslam | cuVSLAM.multicam_mode。0/1/2を明示変換 |
| `--tracking-mode` | eval → evaluation/localization/replay、fresh → sim_vslam | cuVSLAM.tracking_modeとIMU再生要否。0はVO、1はVIO |
| `--base-frame`, `--imu-frame` | eval → localization / sim_vslam | cuVSLAM frame設定、base_frameはVGLにも反映 |
| `--bag`, `--rate` | eval → replay、fresh → sim_vslam、latency →専用CLI | bag player入力・再生速度。rateを有限値に変換・検証 |
| `--eval-localization` | CLI内でlaunch選択／`enable_vgl`へ変換 | VSLAM / VGL / fresh / E2E latency |
| `--latency-output` | E2E latency専用CLI `--output` | CSV/JSON/HTMLの保存先 |
| `--foxglove` | mission `enable_foxglove` → foxglove | Bridge processの起動選択 |
| `--map-root`, `--model-root`, `--bias-root` | CLI/TUI内だけで使用 | 選択候補の探索root。ROS parameterではない |
| `--dry-run`, `--help` | CLI内だけで使用 | 起動せず検証・表示、またはヘルプ表示 |

## 明示的な制約・例外

- driveは左右Infraが必須、RGB E2EはRGBが必須。OFFを黙ってONに戻さず起動前エラーにする。
- EVS E2EでRGB・左右Infraが全てOFFなら、既存方針に従ってRealSense/IMU自体を起動しない。
  collectでは画像OFFでもIMUを維持し、environmentではDepthも独立して維持する。
- 引数未指定のカメラ・IMU・画像・bias等はYAMLを保持する。EVS・Foxglove・bridgeの起動選択はCLIの明示的なboolから組み立てる。
- offline評価ではカメラ・実EVS・車両・録画を起動しない。録画STARTやAUTOへの遷移はどの起動操作でも自動実行しない。

## 検証と残る実機確認

- `scripts/tests/test_bringup_propagation.py`: 全CLI destinationの分類、collectでの一括指定、モード不適合の拒否。
- `kart_bringup/test/test_launch_arguments.py`: ROSのsubstitution/actionを代替し、実launch callbackを実行。
  mission→RealSense/EVS/vehicle、RGB/EVS E2E→decoder、evaluation→localization→VSLAMの最終parameterを確認。
  tracking→HDMap/追従controllerの動的調整OFF・clock OFFも確認。
- 既存のTUI、mission、EVS、recovery、localization、replay、tuning、E2E等のportable testも実行する。
- ROS/Isaac launchライブラリがある環境向けの`test_localization_launch.py`はこのMacではskip。
  代替テストはROS parameter server、SDK、GPU、cameraへの実適用を証明しない。

最終実行結果は136 passed、1 skipped、72 subtests passed。
対象は`ros2_ws/src/kart_bringup/test`全体と`scripts/tests/test_bringup_tui.py`、
`scripts/tests/test_bringup_propagation.py`。一時Python環境に不足していたPlotlyは
一時ディレクトリへ導入し、ソース・システムPythonの依存設定は変更していない。
Python構文確認と`git diff --check`も通過。

実機では変更反映後、STOP・bringup終了状態で`kart_bringup`を再build/sourceして再起動する。
センサの実効parameterとdriverログ、画像OFF時のpublisher数、録画名・保存先を確認する。
OpenEB biasは`bias_file`のパスだけでなく`Loaded camera biases from ...`ログも確認する。

## 参照した上流実装

- [Isaac launch helper](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_launch_utils/isaac_ros_launch_utils/core.py): 引数のPythonリテラル評価とinclude時の文字列化。
- [ROS launch Node](https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/actions/node.py): 名前指定時のparameter辞書のscope。
- [ROS parameter正規化](https://github.com/ros2/launch_ros/blob/rolling/launch_ros/launch_ros/utilities/normalize_parameters.py): Python文字列をYAML dumpして型を保持。kartでは未評価substitutionをparameter辞書へ直接入れず、設定helperの確定値を渡す。
