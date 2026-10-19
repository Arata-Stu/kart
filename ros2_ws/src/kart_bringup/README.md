# kart_bringup

## 車両特性の手動記録

`fixed_command:=true throttle:=0.10 steering:=0.0`を追加すると、
STOP→AUTOで指定した固定操作を保持する`kart_control/fixed_command`を単独processで起動する。
`fixed_command`未指定は`tuning/launch.json`のfalse。`steering/throttle/brake/duration_s`の既定は
空文字で`tuning/fixed_command.yaml`を保持する。duration_s=0は無制限、正値は保持秒数。
`/control/fixed_status`（std_msgs/msg/String）を最大10Hzで出版し、標準bagへ含める。
固定nodeの終了はlaunch全体を終了させる。走行値の動的変更・自動AUTO要求は行わない。
MANUALは手動のまま。解析は`--mode auto`を指定する。
[全ノード仕様](../kart_control/README.md#固定指令による車両特性試験)／
[試験手順](../../../tools/vehicle_tuning/README.md#固定スロットルステアでの記録)。

`tuning_collect.launch.py`は通常のcollect構成に地図なしcuVSLAMを追加する。
RealSenseとcuVSLAMはmission所有の同じmultithreaded containerへloadする。
地図・HDMap・VGL・追従制御は不要。録画STARTと走行操作は独立し、AUTO要求を送らない。
保存地図、mapping、動的TF出版を無効にして局所odometryを記録する。

```bash
bash /workspaces/scripts/vehicle_tuning.sh collect run_name:=straight_01
```

独自ノードは追加しない。既存`visual_slam`（`isaac_ros_cuvslam`、
`nvidia::isaac_ros::visual_slam::VisualSlamNode`）をloadする。
入力は左右`/realsense/infra{1,2}/image_rect_raw`（sensor_msgs/msg/Image）、
`/realsense/infra{1,2}/camera_info`（sensor_msgs/msg/CameraInfo）、
VIO時の`/realsense/imu`（sensor_msgs/msg/Imu）、`/tf_static`（tf2_msgs/msg/TFMessage）。
画像/IMUはSENSOR_DATA depth 5。主な出力は`/visual_slam/tracking/odometry`
（nav_msgs/msg/Odometry）と`/localization/vslam/diagnostics`
（diagnostic_msgs/msg/DiagnosticArray）。外部ノードの全仕様は既存localization説明を参照。

全launch引数の既定は空文字でYAML保持。`device`、`record_dir`、`run_name`はvehicleへ、
`rgb_fps`、`infra_fps`はカメラへ渡す。`tracking_mode`は0=VO/1=VIO（YAML既定0）、
`multicam_mode`は0=Moderate/1=Performance/2=Precision（YAML既定1）。
stereo無効・camera mount未設定は起動前に拒否する。base_frameはbase_link。
静的値の正本は[config/tuning](config/tuning/README.md)。

記録要件、解析フィルタ、全CLI設定、候補のレビュー手順は
[車両チューニング](../../../tools/vehicle_tuning/README.md)。
構成のportableテストは`test/test_tuning.py`。ROS/CUDAロード・実車記録は未検証。

`vehicle.launch.py`でJoy入力・操作解釈・モード管理・mux・任意のUSB bridgeを起動する。
このpackage自身の処理ノード、入力topic、出力topic、独自ROSパラメータはない。
以下はlaunchが起動するノードと、構成全体の既定インターフェース。

## 起動ノードの既定名

| ノード名（namespaceなし） | package | 条件 |
| --- | --- | --- |
| `/kart_joy_node` | [kart_joy](../kart_joy/README.md) | `enable_joy=true`（既定） |
| `/kart_joy_manager` | [kart_system](../kart_system/README.md) | `enable_joy=true`（既定） |
| `/operation_mode_manager` | kart_system | `enable_control=true`（既定） |
| `/command_mux` | kart_system | `enable_control=true`（既定） |
| `/kart_bag_manager` | [kart_bag_manager](../kart_bag_manager/README.md) | `enable_bag_manager:=true`（既定）。独立Pythonプロセス |
| `/kart_bridge` | [kart_vehicle](../kart_vehicle/README.md) | `enable_bridge:=true` |
| `/vehicle_container` | rclcpp_components | `composed:=true`時のcontainer |

C++ノードは既定で単一container＋intra-process communication有効。bag managerは常に別プロセス。`composed:=false`で独立プロセスへ分離する。
すべて`output="screen"`なので、INFOレベルのトリム更新ログは起動端末に表示される。

## 入出力topicの既定名

namespaceなし・remapなしの既定値。ROS標準の`/rosout`、`/parameter_events`等は省略。
`namespace:=kart`なら各topicの先頭に`/kart`が付く。型・QoS・詳細は上記各packageのREADMEを参照。

| Topic既定名 | 型 | 出力元 → 入力先 |
| --- | --- | --- |
| `/bag/request` | `kart_interfaces/msg/BagRequest` | joy manager・外部操作 → bag manager |
| `/bag/status` | `kart_interfaces/msg/BagStatus` | bag manager → 観測用 |
| `/joy` | `sensor_msgs/msg/Joy` | joy → joy manager |
| `/joy/raw` | `sensor_msgs/msg/Joy` | joy → 外部GUI/CUI |
| `/joy/connected` | `std_msgs/msg/Bool` | joy → 観測用 |
| `/joy/ready` | `std_msgs/msg/Bool` | joy → joy manager |
| `/joy/status` | `std_msgs/msg/String` | joy → 外部GUI/CUI |
| `/teleop/control_cmd` | `kart_interfaces/msg/ControlCommand` | joy manager → mux |
| `/auto/control_cmd` | `kart_interfaces/msg/ControlCommand` | 外部の自律制御ノード → mux |
| `/vehicle/control_cmd` | `kart_interfaces/msg/ControlCommand` | mux → bridge |
| `/operation_mode/request` | `kart_interfaces/msg/OperationModeRequest` | joy manager・mux・bridge・外部操作 → mode manager |
| `/operation_mode/state` | `kart_interfaces/msg/OperationModeState` | mode manager → joy manager・mux・bridge |
| `/vehicle/trim/request` | `kart_interfaces/msg/ControlTrim` | joy manager → bridge |
| `/vehicle/trim/state` | `kart_interfaces/msg/ControlTrim` | bridge → 観測用 |
| `/vehicle/rc_channels` | `std_msgs/msg/Int32MultiArray` | bridge → 観測用 |
| `/vehicle/output_channels` | `std_msgs/msg/Int32MultiArray` | bridge → 観測用 |
| `/vehicle/vbec` | `std_msgs/msg/Float32` | bridge → 観測用 |
| `/vehicle/active_path` | `std_msgs/msg/UInt8` | bridge → 観測用 |
| `/propo/control_cmd` | `kart_interfaces/msg/ControlCommand` | bridge → 記録・観測用 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | joy・bridge → 観測用 |

bridge無効時はbridge由来の配信・購読は存在しない。自律指令源・GUI/CUIは本launchでは起動しない。bag managerは待機状態で起動し、標準bringup設定ではR1で録画を開始、L1で停止する。
子recorderの入力topicと任意RAW出力は[kart_bag_manager](../kart_bag_manager/README.md)参照。
Joyサービスの既定名は`/kart_joy_node/configure`、`/kart_joy_node/reload_profile`。

## 設定の正本と優先順位

実運用はこのpackageの[config/bringup.yaml](config/bringup.yaml)を入口にする。
個別ROS pkgのconfigは単体用の基準値・全parameter一覧であり、ここでは暗黙に読み込まない。

| ファイル | 用途 |
| --- | --- |
| [config/bringup.yaml](config/bringup.yaml) | container名・namespace・機能選択・終了猶予・下記ファイルへの対応 |
| [config/input/joy.yaml](config/input/joy.yaml) | 入力取得とprofile_path |
| [config/input/joy_manager.yaml](config/input/joy_manager.yaml) | 操作割り当て・スケール・トリム要求 |
| [config/system/operation_mode_manager.yaml](config/system/operation_mode_manager.yaml) | mode managerの共通parameter |
| [config/system/command_mux.yaml](config/system/command_mux.yaml) | muxの鮮度設定 |
| [config/vehicle/bridge.yaml](config/vehicle/bridge.yaml) | USBデバイス・タイムアウト・出力補正 |
| [config/recording/bag_manager.yaml](config/recording/bag_manager.yaml) | 保存先・録画対象・storage等 |

manifest内のファイルパスはmanifestのあるディレクトリから解決する。独自configを作る場合は
関連ファイルを含むツリーをコピーするか、manifestの対応先に絶対パスを指定する。
全設定ファイルを起動前に検証するため、無効な機能についてもファイルは必要。

優先順位は**運用YAML → 明示された実行時引数**。launch内にROS parameterの静的な上書き辞書は持たない。
起動端末に各ノードの設定ファイルと有効値、明示的な上書きを表示する。
bridge有効時はcontrolも有効、device指定あり、構成全体でuse_sim_time=falseを必要とする。

Isaac ROS等のapt導入packageを追加する場合も、使用ノードの設定一式をこのpackageの
モジュール別configに置く。差分だけのYAMLやapt側設定の暗黙読込は避け、出典・確認したバージョン・
kartでの変更理由・省略項目をYAMLコメントに残す。apt更新時は上流との差分を確認する。
詳細は[外部packageの設定ルール](../../../docs/policies/ros_packages.md#aptで導入する外部ros-package)を参照。
現在のvehicle.launch.pyにはIsaac ROSノードの起動・設定接続はまだ追加していない。

## Launch引数（ROSパラメータとは別）

`config`以外の引数の既定値は空文字（未指定）。空ならYAML値をそのまま使う。
以下の「運用既定値」はlaunchの固定値ではなく、同梱configの現在の値。
root namespaceへの明示変更は`namespace:=/`とする。

| 引数 | 運用既定値 | 明示指定時の用途 |
| --- | --- | --- |
| `config` | package shareのconfig/bringup.yaml | 運用manifestの差替え |
| `container_name` | `vehicle_container` | container名 |
| `namespace` | 空 | 全ノードのnamespace |
| `composed` | `true` | C++ノードのcomponent／独立プロセス選択 |
| `enable_joy` | `true` | joyとjoy managerの起動 |
| `enable_control` | `true` | mode managerとmuxの起動 |
| `enable_bridge` | `false` | USB bridgeの起動 |
| `enable_bag_manager` | `true` | bag managerの待機起動。自動録画はしない |
| `profile` | `/workspaces/config/joy.yaml` | joy.profile_pathの上書き |
| `device` | 空 | bridge.deviceの上書き |
| `record_dir` | `/workspaces/record` | bag manager.output_dirの上書き |
| `bag_config` | recording/bag_manager.yaml | 録画設定ファイルの差替え。CLI相対パスは実行ディレクトリ基準 |
| `bag_shutdown_timeout` | `20` s | bag managerへSIGTERMを送るまでの猶予 |

launchに実行時引数を作っていない静的設定は、対応する運用YAMLを変更する。
例: bag_sigkill_timeout（5 s）、speed_scale（0.3）、command_timeout（0.15 s）。
ノードごとのparameter説明は各package READMEに記載する。

旧kart_system/config/vehicle.yamlは廃止し、各pkgと運用configへ分離した。
旧`config:=<ノードparameter YAML>`は使用せず、運用manifestを指定する。

## ビルド・起動

`BUILD_TESTING`有効時は`find_package(ament_cmake_pytest REQUIRED)`を読み込んでから
`ament_add_pytest_test`で各テストを登録する。依存はpackage.xmlのtest_dependに記載。

```bash
cd /workspaces/ros2_ws
colcon build --symlink-install --packages-up-to kart_bringup
source install/setup.bash
# 初回のみJoy profileを作成
ros2 run kart_joy kart_joy_config init --profile /workspaces/config/joy.yaml
# USB bridgeなし
ros2 launch kart_bringup vehicle.launch.py
# USB bridgeあり（デバイス名を実際の値に置き換える）
ros2 launch kart_bringup vehicle.launch.py enable_bridge:=true device:=/dev/serial/by-id/実際の基板ID
```

動作は常にSTOPから開始する。操作・再arm条件は[全体仕様](../../../docs/vehicle.md)を参照。
設定解決テスト（空引数で静的値を保持・明示上書き・依存検証・隠れたfallbackなし）とPython構文確認は実施済み。ROS上のlaunch実行・結合ビルド・実機動作は未検証。

## Notebook側のオフライン地図作成

`mapping.launch.py`を追加。`vehicle.launch.py`とは独立し、
`kart_mapping_container`へ`isaac_ros_cuvslam`の`VisualSlamNode`だけをロードする。
launch自身のノード・topicはなく、起動する外部ノード名は`visual_slam`。
静的設定は`config/mapping/cuvslam.yaml`、bag/topic/待ち時間は`config/mapping/workflow.json`。
58個の上流独自parameterとROS共通`use_sim_time`を明示している。

必須launch引数`job_file`（JSONの絶対パス）以外に既定値の重複定義はない。
ジョブで明示されたframe・tracking_modeだけを上書きし、設定ファイルと上書き内容を起動時に表示する。
通常は[Kart Map Studio](../../../tools/app/README.md)から[kart_mapping](../kart_mapping/README.md)を実行する。

利用ノードの入出力topic・型・QoS、全parameterの既定値・意味・出典・上流との差異は
[config/mapping/README.md](config/mapping/README.md)を参照。
ローカルの構文・設定キー整合は確認し、ROS結合／GPU実処理は未確認。

### オフライン地図生成の変更

Map Studioの生成は公式 `isaac_mapping_ros/create_map_offline.py` へ委譲する。
`config/mapping/workflow.json`がtopic、基準frame、光学frame、domainの正本。
設定の全既定値と入力型は[kart_mapping](../kart_mapping/README.md)参照。
この生成経路では`mapping.launch.py`・`cuvslam.yaml`・replay QoSは使用しない。
点群取得は下記のcapture専用工程で実行する。

### HDMap参照・可視化

`ros2 launch kart_bringup hdmap.launch.py map_file:=/absolute/path/map.json lane_id:=lane_001`
で独立した`kart_hdmap/hdmap_server`を起動する。車両ノード・TF・Bridgeは起動しない。
実運用設定は`config/hdmap/hdmap.yaml`。全parameter、topic型、QoS、サービスとTF契約は
[kart_hdmap README](../kart_hdmap/README.md)参照。
既定はmap_file空（必須指定）、frame_id=map、lane_id空（全lane表示のみ）、publish_rate_hz=1.0、
line_width_m=0.04、label_height_m=0.25、use_sim_time=false。
launch引数map_file/lane_id/use_sim_timeは空ならYAMLを保持し、明示値だけ上書きする。

### 保存済みVSLAM地図の点群取得

`mapping.launch.py`はcapture job専用に接続した。`config/mapping/capture_cuvslam.yaml`と
元地図作成時のframe/topicを使い、可視化ありで保存地図の作業コピーを読み込む。
`config/mapping/capture.json`はreplay_rate=1.0、ready_timeout_s=120、drain_timeout_s=30、
settle_s=3、tail_tolerance_s=0.5。詳細は[kart_mapping README](../kart_mapping/README.md)。
起動順・bag再生・localize成功検査は`kart_mapping/capture_snapshot`が管理する。

### 走行時VSLAM + VGL

`localization.launch.py`で`visual_slam`と`visual_global_localization`を独立containerへ起動する。
`config/localization/{cuvslam,vgl}.yaml`が全設定。Jetsonではpose/pathのみを可視化し、
画像・特徴点の可視化を無効化する。VSLAMがmap→odom→base_linkのTFを専有する。
`prepare_vgl_map` CLIで公式offlineの保存画像/posesから対応VGL地図を別bundleへ生成できる。
モデルはALIKED/LightGlueを明示し、既定424×240。モデル自体・GPU別engineは別途必要。
必須引数、全parameter、topic型、service、TFとモデルの制約、検証範囲は
[localization README](config/localization/README.md)を参照。
Joyの再要求service接続先は用意したが、ボタン割当は未実装。

localization launchの記述はIsaac ROS公式mapping系と同じ`isaac_ros_launch_utils`形式。
引数宣言、設定検証、Component定義、container起動、Component読込みを分離する。
`ros2 launch kart_bringup localization.launch.py --show-args`で8引数を一覧表示できる。
launch APIのテストはROS/Isaac環境で実行し、Macではskipする。

localizationは`launch/modules/localization/{vslam,vgl}.launch.py`へComponent定義を分割。
最上位だけがcontainerを作成し、moduleは既存containerへのloadのみを担当する。
`create_vslam_container/create_vgl_container:=false`で外部所有containerへ接続できる。
container配置・所有権・executorは`config/localization/containers.json`が正本。
共有containerの生成は1回に集約。外部container内のComponentは終了時に自動unloadしない。

### オフラインライン追従

`tracking.launch.py`がHDMap serverとPure Pursuit Componentを起動する。
引数map_file必須、lane_id空（未選択）、container_name/create_container空（launch.json維持）、
use_sim_time空（YAML維持）。config/control/launch.jsonはcontainer_name=kart_control_container、
create_container=true、container_type=multithreaded。
module `modules/control/pure_pursuit.launch.py`は既存containerへloadするだけ。
全topic・parameter・TF・速度と正規化スロットルの境界は[kart_control README](../kart_control/README.md)。

tracking.launch.pyは同じcontainerへ`kart_control::SpeedControllerNode`もloadする。
設定は`config/control/speed_controller.yaml`、moduleは`modules/control/speed_controller.launch.py`。
速度PIDが`/auto/control_cmd`へ出力する。既定gain=0/舵角未校正、AUTO要求は行わない。
PID係数はROS parameterで動的調整でき、全値とtopic型はkart_control README参照。

### Jetsonハードウェア監視と録画

`vehicle.launch.py`は`enable_jetson_stats=true`を既定とし、monitoring moduleでJetsonを検出した場合に
公式`isaac_ros_jetson_stats/jtop`を別processとして起動する。Python実行ファイルのためComponentではない。
localization/tracking launchでは重複起動しない。`enable_jetson_stats:=false`で除外できる。
設定・topic・前提条件は[monitoring README](config/monitoring/README.md)参照。

tracking/hdmap launchの`line_type:=centerline|raceline|customline`で起動時選択できる。
空ならhdmap.yamlを保持。実行中は`ros2 param set /hdmap_server line_type raceline`で変更し、
`/hdmap/selected_line`で確認できる。詳細はkart_hdmap README参照。

## DINOv3 E2E

`e2e.launch.py`は公式GPU image encoder、公式TensorRT、kartのC++ decoderを
同一のmultithreaded containerへloadする。PyTorch ROS推論ノードは使用しない。
moduleは`modules/e2e/{image_encoder,inference}.launch.py`に分離しload専用。
親だけがcontainerの所有／外部container利用を決める。

```bash
ros2 launch kart_bringup e2e.launch.py model_dir:=/workspaces/models/e2e_v1
# 外部containerへのload
ros2 launch kart_bringup e2e.launch.py model_dir:=/workspaces/models/e2e_v1 \
  container_name:=/shared_container create_container:=false
```

全launch引数・設定・公式ソースは[config/e2e/README.md](config/e2e/README.md)。
ノードの入出力・全parameter・学習／ONNX exportは[kart_e2e](../kart_e2e/README.md)。
既定drive_enabled=false。trueはAUTO muxへ接続するため車両出力を伴う。
rule-based制御と同時起動しない。外部containerのComponentはlaunch終了時に自動unloadされない。
カメラ・車両・jtopは既存launchが所有する。ROS/Jetson結合は未確認。


### OpenEB RAW連携profile

recording/bag_manager_openeb.yamlは全bag manager設定を含む運用profile。
raw_recording_driver_node=/event_camera/event_camera_driverを設定し、
RAWをrosbagと同じsession directoryに保存する。利用時はbag_configでこのファイルを指定し、
bag_shutdown_timeout:=26等で終了猶予を確保する。missionのenable_evsでOpenEB direct pipelineを起動する。
RAW記録はnative OpenEB writer、rosbagはMCAP writerとして独立する。
イベントpacketとGPU tensor payloadは除外し、可視化Imageと診断を対象に加える。
通常profileはRAW連携無効のまま。実カメラ・サービス結合は未確認。
詳細は[kart_bag_manager README](../kart_bag_manager/README.md)を参照。

## Foxglove Bridge

`ros2 launch kart_bringup foxglove.launch.py`で独立プロセス`/foxglove_bridge`を一度起動する。
vehicle/localization/e2eはBridgeを自動起動しない。コンテナの生成・loadは行わない。
Docker依存へ`ros-lyrical-foxglove-bridge`を追加したためimage再buildが必要。
接続は`ws://<Jetson IP>:8765`。地図・TF・軌跡・走行／録画状態・jtopのみ公開する。
Foxgloveからの操作は`/localization/pose_hint`とVGL検索要求だけに限定する。
ノードの全parameter既定値、入出力topic名・型、Foxglove操作、対応版・検証範囲は
[visualization設定](config/visualization/README.md)と[全parameter YAML](config/visualization/foxglove.yaml)参照。

## 用途別起動（TUI）

ROS開発コンテナ内で、リポジトリルートから`bash scripts/bringup.sh`を実行する。
補助実装は`scripts/lib/bringup.py`。通常はshの入口を使う。
fzfの上下キー・文字検索・Enterで用途、RGB/InfraのHz、EVS、Foxgloveを選ぶ。車両基板は既定で/dev/ttyACM0を使用し、選択を省略する。
地図走行ではmap/を深さ6まで探索し、HDMap、lane＋line、対応するVSLAM/VGL bundleを選択する。
モデルはmodels/およびmap/からALIKED/LightGlue資産を検出する。ごみ箱・隠しdirectory・
symlink directoryは探索しない。HDMapとbundleが同じ座標系であることは操作者が確認する。
HDMapを選んだだけで無関係なbundleを自動対応させない。

| mode | 起動内容 |
|---|---|
| collect | RealSense、Joy、mode manager、command mux、車両bridge、bag manager、Jetson jtop |
| drive | collect一式＋VSLAM/VGL、HDMap/reference line、Pure Pursuit、速度PID |
| e2e | collect一式＋公式image encoder/TensorRT＋control decoder。VSLAM/VGLなし |
| eval | bag＋保存map localization、またはmapなしVSLAM診断＋RViz2。実センサ・車両・追従・録画なし |

EVSとFoxgloveは選択時だけ追加。collectでいうvehicle controlは手動操作の制御権・指令muxであり、
自律追従は起動しない。jtopは既存vehicle launchが所有し、非Jetsonではskipする。
mode/構成の正本は`config/mission.yaml`。センサの詳細、topic・parameterは
[sensors設定](config/sensors/README.md)参照。mission自身のnode/topicはない。

起動しても録画は開始せず、AUTOへも切り替えない。録画はR1開始/L1停止。
停止完了を確認してからCtrl-Cでlaunchを終了する。Hz変更は再起動時の選択であり、録画中には変更しない。
データ収集後はMap Studioのbagからのoffline mappingを使う。このlaunchでmap生成は実行しない。

```bash
bash scripts/bringup.sh
# 選択内容とコマンドだけ表示（ノード起動なし）
bash scripts/bringup.sh --dry-run
# 非対話のデータ収集例
bash scripts/bringup.sh --mode collect --device /dev/ttyACM0 --rgb-fps 30 --infra-fps 60
# 車両基板を接続しない確認
bash scripts/bringup.sh --mode collect --no-bridge --dry-run
# 外部の地図・モデル置場を一覧探索
bash scripts/bringup.sh --map-root /workspaces/map --model-root /workspaces/models
```

TUIは標準入力の端末を使う。Esc/Ctrl-Cで中止。`--mode`指定時は非対話となり、
driveには`--map-file`、`--map-dir`、`--model-dir`、`--lane-id`、`--line-type`を指定する。
`--evs`、`--foxglove`で追加機能、`--no-bridge`で車両基板なし。
`--record-dir`の既定はリポジトリのrecord/。`KART_BRINGUP_PYTHON`でPython実行系を指定できる。
必要なpython3-yamlはDockerの既存依存。`exec`でros2 launchへ移行し、終了signalを直接届ける。

### mission.launch.py引数

| 引数 | 既定 | 意味 |
|---|---|---|
| mode | collect | collect / drive / e2e。evalはevaluation.launch.pyへ分岐 |
| rgb_fps / infra_fps | 空 | YAML保持。0=なし、30/60/90=候補Hz |
| device | 空 | vehicle/bridge.yaml保持（既定/dev/ttyACM0）。必要時のみ明示上書き |
| record_dir | 空 | bag YAMLの/workspaces/record保持 |
| run_name | 空 | bagのrecording_name上書き。TUI既定run |
| e2e_model_dir | 空 | e2e用model.onnx＋metadata.jsonのdirectory |
| enable_bridge | 空 | mission YAMLのtrue保持。falseでUSB基板を起動しない |
| enable_evs / enable_foxglove | 空 | mission YAMLのfalse保持 |
| sensor_container | 空 | mission YAMLのkart_sensor_container保持 |
| create_sensor_container | 空 | mission YAMLのtrue保持。falseは外部所有containerへのload |
| map_file | 空 | drive用のmap.jsonまたはhd_map.yaml |
| map_dir | 空 | drive用の完成したkart.vgl.v1 bundle |
| model_dir | 空 | 対象GPU用ALIKED/LightGlueモデルdirectory |
| lane_id | 空 | drive用HDMap内のlane ID |
| line_type | 空 | drive用centerline / raceline / customline |

`config/mission.yaml`の`sensor_container_type=multithreaded`を使用。
各modeのlocalization/tracking/e2eはcollect=false/false/false、drive=true/true/false、
e2e=false/false/true。e2e_drive_enabled=trueでdecoderを有効化するが、AUTO操作までは出力しない。
missionはsensor containerを一度だけ作成し、RealSense/OpenEBとVSLAMまたはE2Eの
image encoder・TensorRT・decoderを同じcontainerへloadする。VGL、vehicle、trackingは別所有。
下位moduleはcontainerを作らない。evalは実センサを使わず、localizationがcontainerを所有する。
二重起動しないこと。追加で独立launchを起動する場合はcontainer所有権とnode重複を確認する。

センサ・localization・controlの状態と選択ラインはbringupのbag設定に追加。
`realsense2_camera`はDockerfile.realsenseで既にビルドするためDockerfile.kartのAPTへ重複追加しない。
`openeb_ros2`は任意機能であり、kart branchを別途導入・ビルドしてからEVSを有効にする。

現時点で車体→カメラの取付TFは仮候補のため未配信。後輪軸TF、舵角校正・PID調整は別途必要。
地図走行の起動構成が揃っても、既定の舵角0・PID gain 0のままで走行可能とは扱わない。
VGL engineは実行GPU上で既存localization launchが検査する。
検証: portable選択・引数・探索テスト、shell構文、dry-run。
ROS/GPU/componentロード、RealSense実機Hz、車両・RAW記録連携は未確認。

### 選択・保存・E2E metadata

RGB/Infraは30/60/90Hz/なし。カメラ機種の実profile対応は別途確認する。
driveでInfraなし、e2eでRGBなしは起動前エラー。evalではbagの左右画像・CameraInfo・
/tf_staticに記録があることを確認し、不足をエラーにする。
centerline/raceline/customlineはHDMap内で生成済みのものだけ一覧に出す。

run_nameは起動時に入力、または `--run-name trial`。
保存先は `record/YYYY-MM-DD/HH-MM/trial`。日時はbag manager起動時に固定。
同じセッションの追加録画はtrial_01等。START前にdirectoryを作らないため、
起動直後のCtrl-Cでは空directoryは残らない。自動削除はしない。

E2Eはmodels/およびe2e/models/からONNX exportを探索する。checkpoint .ptではなく、
model.onnxとmetadata.jsonが必要。output_mode、入力サイズ・前処理、固定throttle・上限を
metadataで検証/適用する。steer_onlyは固定throttle、steer_throttleは予測throttleを使用。
runtime metadataのない旧exportは選択候補外とし、学習checkpointから値を明示して再exportする。
[モデル契約](../kart_e2e/README.md#起動時のモデル設定)参照。

### Notebookでbagのlocalization確認

TUIのevalでHDMap、lane/line、対応bundle、Notebook GPU用VGLモデル、bagを選ぶ。
`--record-dir`がbag探索ルート。既定1倍速、CLI `--rate`で変更可能。
RVizはNVIDIA公式cuVSLAM default.cfg.rvizを基に、画像displayを除き、
HDMapとVGL poseを追加。Fixed Frameはmap。2D Pose Estimateは/localization/pose_hintへ送る。
VGL→VSLAM再localizeの経路はliveと共通。VGLはbootstrap/再要求時に動作する。

`evaluation.launch.py`はbag/map_dir/model_dir/map_file/lane_id/line_typeが必須。
rate/rvizの既定は空でevaluation/replay.yamlを保持（1.0/true）。
`localization.launch.py visualize:=true`はSLAM/landmarks/observations可視化を有効化する。
Jetsonのlive YAMLはpose/path中心のまま。moduleと全parameter/topicは
[evaluation設定](config/evaluation/README.md)参照。

過去の動的TF（/tf）・odom・制御topicは再生しない。
カメラ入力と/tf_staticを再生し、新しいVSLAMだけがmap→odom→base_linkを配信する。
bagに校正済み車体→カメラTFが必要。仮のidentity TFで補完しない。
RVizで点群/HDMap/軌跡の整合性を確認できるが、正解軌跡なしのATE/RPE評価は実装していない。
portable検証とROS/GPU/実bag動作は別であり、実機統合は未検証。

vehicle.launch.pyの追加実行時引数run_name（既定空）はrecording_name、session_layout（既定空）は
bagのsession_layoutを明示上書きする。空の場合は運用YAMLを保持する。

## EVS direct pipeline

`mission.launch.py enable_evs:=true`は専用プロセスの`openeb_tensor_pipeline`を所有する。
RealSense用sensor containerとは別。driver/event_tensor/event_preprocessorと
`/event_camera/tensor_container`は同一EVSプロセスにあり、packet topicを経由しない。
全設定とtopic/parameterの説明は[センサ設定](config/sensors/README.md)。
RAWはbag START後に同一session dirへ保存し、起動では録画しない。
EVS有効時のbag終了猶予は26秒。現在のE2E missionはRGBモデル専用であり、
EVSモデルのTensorRT接続・制御decoder統合は別途必要。

### EVS起動時選択

`bash scripts/bringup.sh`のTUIでEVS有効/無効、decoder（cpu/cuda/cuda_async）、
`bias/evs/`の.bias一覧またはカメラ既定を選択する。packet topicは既定OFF。
CLIは`--evs`、`--no-evs`、`--evs-backend`、`--evs-serial`、`--evs-bias-file`、
`--bias-root`。decoder/serial/biasの明示指定はEVSを有効にする。evalではEVS指定を拒否する。

| mission/module launch引数 | 既定値 | 意味 |
|---|---|---|
| evs_backend | 空 | 運用YAMLのcuda_asyncを保持。cpu/cuda/cuda_async選択 |
| evs_serial | 空 | 運用YAMLのserialを保持 |
| evs_bias_file | 空 | 運用YAMLのbias_fileを保持。.bias絶対パス、@defaultでカメラ既定 |

`bias_file`は起動時のみ適用し、変更後はdriverを再起動する。
モジュールは運用YAML全体に明示overrideを最後に適用し、実効選択をログ表示する。
`evs_bias_file:=@default`は空bias_fileへ変換する。biasパスは起動前に存在・拡張子・非空を確認。
SDKによる内容/機種互換性確認は実機起動時。
チューナーは[tools/evs_bias_tuner](../../../tools/evs_bias_tuner/README.md)、
`bash scripts/sensors/evs-bias.sh --build`でビルド/起動する。実行時にカメラを占有するのでbringupと併用しない。
検証はtest/test_evs.pyとtest/test_mission.py。ROS結合・実機bias適用は未確認。

OpenEB学習収集では/event_camera/tensor_timing（std_msgs/msg/String、JSON v1、
reliable/volatile/depth 512、既定250 Hz）をOpenEB用bag profileへ追加。
GPU Tensorは記録せず、窓のセンサ時刻とsensor_to_ros_offset_nsを保存する。
RAWの相対時刻には各RAWのSDK timestamp shiftを足し、そのoffsetでROS時刻へ変換する。
録画開始ROS時刻をRAWの0へ直接対応させない。
RAW shift取得ツールはtools/evs_raw_timing（scripts/sensors/evs-raw-timing.sh）。
USB遅延を含む推定なので、実機で同期精度を確認する。

fzfによる選択は上下キーで移動、文字入力で絞り込み、Enterで確定、Esc/Ctrl-Cで中止。
既定候補を先頭に表示します。run_nameは自由入力です。
fzfはDockerfile.kartで導入します。既存コンテナでは一時的に
`sudo apt-get install -y fzf`で追加可能。非対話の`--mode`指定ではfzfは不要です。

## E2E TensorRT事前build

Jetsonのkartコンテナ内で`bash /workspaces/scripts/e2e_trt.sh`を実行する。`/workspaces/models`以下の転送済みONNX bundleをfzfで選択する。直接指定は`bash /workspaces/scripts/e2e_trt.sh /workspaces/models/<モデル名>`、探索先変更は`--models-root <dir>`。`KART_TRT_PYTHON`既定は`/opt/inference/bin/python`。ROS起動・車両指令publishは行わない。

FP32が既定。TensorRT 10では`--fp16`も指定可能、TensorRT 11以降ではこのフラグを拒否する。`--force`で再buildする。モデル配下の`model_<ONNX SHA先頭12桁>.plan`と同名`.json`にハッシュ・GPU UUID/名前・アーキテクチャ・CUDAドライバ・TensorRT版を保存する。`.build.log`と`.timings.json`には詳細ログと合成入力による速度計測を保存。warmup 500ms、計測3秒で、GPU compute/latencyの平均・中央値・P95が得られる場合に表示する。FP16の精度一致やrosbag走行性能はこの計測では検証しない。稼働中の推論を止めてから実行する。

E2E launchは有効なmanifestとハッシュ・ハードウェア一致、およびTensorRT deserializeとFP32入出力binding/形状検証が成功したengineだけを使用する。`force_engine_update=false`が既定で、true指定は拒否する。engineが未build・不適合なら案内を出して起動を停止し、自動buildしない。事前に`scripts/e2e_trt.sh`でbuildする。Notebookで作ったengineをJetsonへ流用しない。build失敗時は以前の成功engineを保持する。

## VSLAM地図だけでbag評価

bringup.shのevalでは「保存mapでVO localization」（既定）、「保存mapでVIO localization」、「VSLAM＋VGL」を選択する。VSLAMのみはMap Studio出力の`cuvslam_map/*.mdb`が非空の地図を探索し、VGLモデル・vgl_profile.jsonを要求しない。VGL併用は従来通りprepare_vgl_map済みbundleと実行GPU用モデルが必要。非対話CLIは`--eval-localization vslam|vgl|fresh`。

4番目の用途（offline bag評価）では「mapなしVO評価」と「mapなしVIO評価」を直接選べる。地図・HDMap・VGLモデルを選ばずbagだけ選ぶ。実機/simのbagを同じ入口で再実行し、VIOは`/realsense/imu`も必須入力として検査・再生する。両方ともVGLは起動せず、`sim_vslam.launch.py`で可視化・RVizを有効にする。
`sim_vslam.launch.py`で既存mapを読まず新規SLAMを開始し、RVizに左右画像・観測点群・ランドマーク・軌跡を表示する。
左右のmono8表示は入力画像そのもの。追加の左右CameraビューではCameraInfoとTFを使い、
3Dランドマークを画像へ再投影する。抽出器が検出した全2D pixelを表示するものではなく、
追跡・三角測量された点の投影。観測点群は別のPointCloud2表示で確認する。
最初はstereo VOで確認し、IMUを含むbagでVIOと比較できる。

```bash
bash scripts/bringup.sh --record-dir /workspaces/record/sim
# 非対話でmapなし診断（0=VO、1=VIO）
bash scripts/bringup.sh --mode eval --eval-localization fresh \
  --bag /workspaces/record/sim/<bag-directory> --tracking-mode 0 --rate 0.5
```

bagは左右のrectified画像・CameraInfo・/tf_staticが必要。VIOでは/realsense/imuも必須。
入力購読準備後に再生し、録画済み/clock・動的/tf・真値・制御指令は再生しない。
他のsim/localizationやbag再生を終了してから実行する。再評価はCtrl-C後に再起動してSLAM状態を初期化する。

evaluation.launch.pyの`enable_vgl`既定はfalse、localization.launch.pyでは互換性のためtrue。false時は`model_dir`不要、VGL node/専用containerを起動せず、VSLAMのみ`localize_on_startup=true`・`enable_request_hint=false`で起動する。map_dirは地図ルートまたはcuvslam_map自体。TFのpublisherはVSLAMのみ、bagの古いTFは再生しない。HDMapと同じ地図座標系を選ぶ。初期探索範囲内に位置がない場合はlocalizationが成立しないことがあり、実際の一致をRVizで確認する。

## Web UIからVGL地図を追加生成

既存地図を開き「VGL地図を追加生成」を選択する。ALIKED/LightGlueモデルはmodelsとmap以下の候補から選ぶかパスを入力し、画像サイズ（既定424×240）と保存名（地図名-vgl-vN）を指定する。実行GPUでbuild済みのengineが必要で、download/buildは自動実行しない。公式offline処理の保存画像・posesがない場合は生成できない理由を表示する。

`POST /api/vgl`はinstalled `ros2 run kart_bringup prepare_vgl_map`をジョブ実行し、検証済みbundleを`map/vgl/<保存名>`へ公開する。既存VSLAM地図・HDMapは変更しない。ログ・中止は既存ジョブ画面を使い、失敗・中止時の一時成果物は公開しない。成功後はbringupで「VSLAM＋VGL」を選び生成bundleを指定する。実走行GPUがNotebookと異なる場合は実走行GPU用モデルengineを別途準備する。UIサーバー再起動とkart_bringupのbuild/sourceが必要。

### オフライン再生の操作

再生は一時停止で開始し、選択したlocalizationの画像購読が揃うと自動再開する。VSLAM単独時はvisual_slamのみ、VGL併用時は両nodeを待つ。キーボード操作はlaunch下では無効。別ターミナルから以下を呼ぶ（同じROS環境/domain）。

```bash
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause '{}'
```

終了は起動ターミナルでCtrl-C。先頭から再評価するときは終了後bringupを再起動し、localizationの内部状態も初期化する。

オフラインevaluationの基準frameはevaluation/replay.yamlの`base_frame: camera_link`。点群取得と揃え、bagに未収録の車体取付TFを要求しない。evaluation.launch.pyの`base_frame`引数（既定空文字）で明示変更できる。実車localizationのbase_link設定は変更しない。HDMapのmap座標は変更せず、odomから推定対象camera_linkへのTFを配信する。車体poseの評価には校正済み取付TFとbase_frame:=base_linkが必要。

## 車体取付TF

`vehicle.launch.py`が`modules/vehicle/transforms.launch.py`を一度includeする。
missionのcollect/drive/e2eはいずれもvehicleを経由する。bag評価launchでは起動せず、
既存のcamera_link基準・bag内のカメラTFを維持する。
画像を扱わない静的TF publisherは独立processとし、sensor/VSLAM/E2Eの同一container構成を変更しない。

正本は`config/vehicle/transforms.yaml`。ROS parameter YAMLではなくlaunch用の取付設定。
各項目は`parent`、`child`、`xyz`（m）、`rpy`（rad、roll/pitch/yaw）の全キーを持つ。

| 設定 / ノード名 | parent → child | xyz / rpy既定 |
|---|---|---|
| rear_axle / kart_tf_rear_axle | base_link → rear_axle | 両方[0,0,0]。後輪軸中央という定義上同一 |
| camera_mount / kart_tf_camera_mount | base_link → camera_link | xyz=[0.23385,0.04750,0.10000] m、rpy=[0,0,0] rad。ユーザー指定の暫定値 |
| evs_mount / kart_tf_evs_mount | base_link → event_camera | xyz=[0.2045,0,0.0705] m、rpy=[0,0,0] rad。ユーザー指定の暫定値 |

base_linkは後輪軸の高さの中央で、x前方・y左・z上。xyzは親座標系における子原点、
event_cameraの仮原点はEVS本体前面のレンズ軸中心で、x前方・y左・z上。校正済み光学中心とは区別する。
rpyは子座標系の取付姿勢。光学座標への回転をcamera_mountへ重ねない。
CADの穴座標をcamera_link座標として使わない。D455は2026-10-10のユーザー指定値を暫定採用し、水平・前向きを仮定する。実測校正済みではない。取付値をnullへ戻すとdriveは起動前にエラーになる。
collect/e2eは未設定mountをログ表示して省略する。設定済みmountは録画時にも配信する。

使用実行ファイルは`tf2_ros/static_transform_publisher`（依存`ros-lyrical-tf2-ros`、Docker導入済み）。
出力は`/tf_static` (`tf2_msgs/msg/TFMessage`, reliable/transient_local、静的配信)。入力topicなし。
標準`/rosout`等は省略。独自ROSパラメータなし。取付値は標準CLIの
`--frame-id --child-frame-id --x --y --z --roll --pitch --yaw`へ変換して渡す。
モジュールlaunch引数`transforms_config`の既定はインストール済み上記YAML。
取付値は起動時固定で、変更後は再起動する。RealSense内部TFはドライバだけが所有する。

出典: geometry2の`tf2_ros/static_transform_broadcaster_program.cpp`
（https://github.com/ros2/geometry2/tree/rolling/tf2_ros）。対象はROS Lyrical、aptの厳密な版は実機未確認。
外部nodeのROSパラメータ設定ではなく、全取付入力をこのlaunch設定で管理する。
検証: `test/test_vehicle_tf.py`は未設定ガード・数値検査・軸順序を確認する。
ROS結合と実機の取付座標・TF接続は別途確認が必要。

```bash
bash scripts/build.sh --with-sim --packages-up-to kart_sim kart_bringup
source ros2_ws/install/setup.bash
# bringup実行中、別ターミナルで確認
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link camera_infra1_optical_frame
```

## 室内シミュレーション

`sim.launch.py`はkart_simだけを起動し、実USB bridgeや実センサを起動しない。
本launch自身のノード・入出力はなし。kart_simのtopic/parameterは[package README](../kart_sim/README.md)。
map（既定minicar_2026）、map_dir（既定空、installed map）、
camera_enabled/viewer_enabled/require_mode/imu_enabled/publish_truth_tf、
stereo_hz/rgb_hz/imu_hz、camera_rig_file（各既定空、YAML保持）を指定できる。
運用正本はconfig/sim/sim.yaml。map_fileだけ選択mapから解決する。
`ros2 launch kart_bringup sim.launch.py map:=indoor_empty`で切替。
`camera_enabled:=true monitor_enabled:=true record_dir:=/workspaces/record/sim`で
localhostの画像monitorから真値waypoint周回とrosbag開始/停止を操作できる。
monitorの操作時のみsim内部が制御を所有する。起動だけでは走行・録画を開始しない。
既存vehicle launchとの同時起動はTFとハードウェア所有が重複するため使わない。

`sim_vslam.launch.py`は/visual_slamを専用kart_sim_vslam_containerへloadする。
`bag`既定空はlive入力、指定時は選択bagを再生。`rate`既定空はevaluation/replay.yamlの1倍。
`visualize`既定空はYAMLのfalseを保持、trueで観測点・ランドマーク可視化を有効化。
`rviz`既定空は可視化設定に追従、明示true/falseでRVizの起動だけ上書きする。
RViz設定はconfig/sim/vslam.rviz。bag指定時のみ/replay_readyを起動し、
入力購読と再生resume serviceの準備完了後に再生する（既存evaluation設定を使用）。
運用正本config/sim/vslam.yaml、tracking_mode引数は既定空（YAMLの0=VO）、1=VIO。
入力は/realsense/infra{1,2}/image_rect_raw・camera_info・/realsense/imu。
nodeの出力型・全parameterは[localization config README](config/localization/README.md)、
シミュレーションの変更点・起動例は[sim config README](config/sim/README.md)と
[kart_sim](../kart_sim/README.md)を参照。simのpublish_truth_tf=falseが必要。
本入口はsimそのものや実車、操作ノードを起動しない。GPU環境での結合は未検証。

## MuJoCoを2つのterminalで運転する

`sim.launch.py`がセンサ・固定TF・clock・monitorを担当し、
`sim_bringup.launch.py`がmonitorからの要求で保存地図localizationとrule base制御を起動する。
手順は[simulation.md](../../../docs/simulation.md#monitorからの地図生成とkart制御)を参照。
`sim_drive.launch.py`はHDMap server、既存localization、mode manager、command mux、
pure pursuit、speed controllerだけを起動する。実機のcamera/vehicle/joyは起動しない。
実行時引数は`map_file`、`map_dir`、`model_dir`、`lane_id`（既定空）、
`line_type`（centerline）、`enable_vgl`（false）。保存済みlocalization地図とHDMapは必須。
VGL利用時は対応bundleとGPUに合ったモデルを指定する。

追加ノード`sim_operator`はPythonのプロセス監督で、component化せず別processとして動かす。
起動直後は未設定・STOP。`sim_bringup.launch.py`の`map_root`/`model_root`引数は、
空ならYAML値を使い、指定時だけ上書きする。

| 入出力 | 既定topic/service | 型 |
|---|---|---|
| 入力 | `/sim/operator/request`, `/sim/operator/heartbeat` | std_msgs/String（JSON） |
| 入力 | `/operation_mode/state` | kart_interfaces/OperationModeState |
| 入力 | `/visual_slam/tracking/odometry` | nav_msgs/Odometry |
| 入力 | `/control/tracking_status` | std_msgs/String |
| 入力 | `/localization/vslam/diagnostics` | diagnostic_msgs/DiagnosticArray |
| 出力 | `/sim/operator/status` | std_msgs/String（JSON） |
| 出力 | `/operation_mode/request` | kart_interfaces/OperationModeRequest |
| 出力 | `/hdmap/select_lane` | std_msgs/String |
| client | `/hdmap_server/set_parameters` | rcl_interfaces/SetParameters |

要求JSONは`action`と`value`。actionは`kart_configure`（map/line選択）、`kart_line`
（lane/line変更）、`kart_speed`（動的試験速度）、`kart_auto`、`kart_stop`、`kart_unload`、`kart_refresh`。
configure値は`map_file/map_dir/model_dir/enable_vgl/lane_id/line_type`の文字列dictionary。
line値は`[lane_id,line_type]`。探索された資産だけを許可し、固定argvで子launchを起動する。
地図変更・ライン変更はSTOPで実施する。終了時は所有する子launchとStudioを終了する。

`kart_speed`のvalueは0.05..1.5 m/sの数値、またはnull（地図速度へ復帰）。
実行中のsim pipelineだけを対象に、`/hdmap_server/set_parameters`
（`rcl_interfaces/srv/SetParameters`）で`speed_override_mps`を更新する。
通常走行中も適用でき、走行開始や記録開始は行わない。パラメータ応答成功後だけ
status JSONの`speed_override_mps`に反映（-1は地図速度、nullは未確認）。
`speed_pending`は応答待ち。2秒の応答期限超過・拒否時はエラーとSTOP要求。
restart/unloadで試験速度を破棄する。地図/YAMLは書き換えない。

`config/sim/operator.yaml`が全parameterの運用基準。

| parameter | 既定値 | 意味 |
|---|---|---|
| use_sim_time | true | ROS header時刻はsim clock |
| map_root | /workspaces/map | 保存地図・HDMap・VGL bundle探索root |
| model_root | /workspaces/models | VGLモデル探索root |
| heartbeat_timeout_s | 5.0 | sim heartbeat・推定・診断・状態の受信期限（steady秒） |
| status_period_s | 0.2 | 状態送信周期（steady秒） |
| studio_enabled | true | Map Studioを起動／既存localhost serverを利用 |
| studio_script | /workspaces/tools/app/start.sh | 信頼済みStudio起動script |
| studio_port | 8766 | Studioのlocalhost port |

各`config/sim/drive_*.yaml`は対応するcontrol/system/hdmap設定の全parameterを保持する。
既存package READMEの意味・既定値を引き継ぎ、全ノードの`use_sim_time=true`、
HDMap centerline速度0.35m/s、通常の参照速度上限0.45m/s、lookahead 0.35m、操舵上限0.45radへ変更する。
試験速度のためpure_pursuit / speed_controllerの速度上限は1.5m/sとする。
HDMapの`nominal_speed_limit_mps=0.45`により、試験未指定時のRace/Custom速度上限も維持する。
速度制御はkp=0.8、ki=0.05、kd=0、kff=0.49、throttle_limit=0.65、brake_limit=0.3。
これらはsim用の仮調整。全値は上記YAMLを参照。
Pure Pursuitと速度PIDはuse_sim_time=true時にROS timerで実行し、PID dtと
AUTO開始の中立100msもsim秒で計算する。各stamp期限は従来のsim秒を保持し、
control/mux/sim actuatorの`wall_timeout_s=5.0`とoperator heartbeat 5秒で
実時間の受信途絶を監視する。実機用wall_timeout_s=0.0は従来の短い期限を維持する。
/clock停止中はPIDを積分せず、受信watchdogだけ動作する。5秒以上停止した場合は
AUTOが解除され得るため、再開後に明示的にAUTOを要求する。
静的HDMapの再配信は実時間1Hz、headerはsim時刻。
これは非同期ROS構成であり、physics・推定・制御の完全なlockstepや倍速動作の保証ではない。
AUTO許可には保存地図へのlocalization、freshなodom/診断/tracking/mode、
sim camera有効・真値TF無効・操舵上限一致が必要。接続や推定が途切れればSTOPを要求する。

`sim_operator`の状態JSONには`last_tracking_issue`（最後のtracking異常の状態とsim時刻）を
含める。地図restart時に履歴を消す。起動時の`not_localized`等も記録するため、
異常履歴だけで走行中の停止原因を断定しない。topic・parameterの変更はない。

Foxgloveの内部launchは`launch/modules/visualization/foxglove_bridge.launch.py`。
公開入口とbasenameを分け、ROS 2のshare内検索で同名衝突を起こさない。
旧版から更新するとinstallに古い内部`foxglove.launch.py`が残る場合があるため、
`install/kart_bringup/share/kart_bringup/launch/modules/visualization/foxglove.launch.py`
だけを取り除いて再buildする。ノード・topic・parameterは変更しない。

実機bagのVIO評価を非対話で起動する例:

```bash
bash scripts/bringup.sh --mode eval --eval-localization fresh \
  --tracking-mode 1 --bag /workspaces/record/e522 --rate 1.0
```

既存の`--eval-localization fresh --tracking-mode 0|1`は維持する。
VIOでIMUが記録されていないbagは起動前に拒否する。Map StudioのVIO地図生成を
追加する変更ではなく、bagの再実行・可視化用。node/topic/parameterの変更はない。

mapなしVO/VIO評価のRVizは大きな通常window（1600×900）で起動する。
`--fullscreen`はRVizが全dockを非表示にするため使用しない。画面いっぱいにするには
windowの最大化ボタンを使う。以前の全画面で起動中ならEscでpanelを復帰できる。
`config/sim/vslam.rviz`にQt5のdock配置を保存し、左にDisplays、中央に3D、
右に左右画像を上下配置する。再投影は各画像の切替tabにまとめ、Viewsパネルは置かない。
起動時の注視点は原点。画像受信・dock復元のROS実行はLinuxで確認する。
Qt配置の再生成手順は[tools/rviz](../../../tools/rviz/README.md)参照。
node/topic/ROS parameterは変更しない。

mapなし評価は`base_frame`/`imu_frame`を明示指定できる（空が既定でYAML保持）。
simの既定は`base_link`/`camera_gyro_optical_frame`。実機bagに`base_link`の固定TFが
記録されていない場合は、camera/IMUへ接続される`camera_link`を基準に評価する。
この場合の出力poseは車体基準ではなくcamera_link基準で、車体軌跡との比較には
実測のcamera mount TFが必要。TFを仮に追加して補わない。

```bash
ros2 launch kart_bringup sim_vslam.launch.py \
  bag:=/workspaces/record/e522 tracking_mode:=1 visualize:=true rviz:=true \
  base_frame:=camera_link imu_frame:=camera_imu_optical_frame

bash scripts/bringup.sh --mode eval --eval-localization fresh --tracking-mode 1 \
  --bag /workspaces/record/e522 --base-frame camera_link --imu-frame camera_imu_optical_frame
```

IMU frame名はbagのheaderと記録TFを確認して指定する。上のframe間TFの存在は
対象bagで別途確認する。既定configの変更、ROS topicの変更はない。

保存mapでのoffline VIO評価をTUIから直接選べる。保存地図・HDMap・lane/line・bagを
選択し、evaluation.launch.pyへtracking_mode=1を渡す。VGLを選択しなければ
モデル探索やVGL起動は行わない。VIOはIMUが必須で、bag不足を起動前に拒否する。
frame overrideはmapなし/保存mapのどちらのevalでも利用できる。

```bash
# TUIで「保存mapでVIO localization」を選ぶ（今回の旧実機bagのframe）
bash scripts/bringup.sh --base-frame camera_link --imu-frame camera_imu_optical_frame

# 非対話: 既存の保存map評価へ--tracking-mode 1を追加
bash scripts/bringup.sh --mode eval --eval-localization vslam --tracking-mode 1 \
  --bag /workspaces/record/e522 --map-dir /workspaces/map/e522 \
  --map-file /workspaces/map/e522/exports/revision-12-v2/hd_map.yaml \
  --lane-id lane_001 --line-type centerline \
  --base-frame camera_link --imu-frame camera_imu_optical_frame
```

localization/evaluation launchへ直接tracking_mode:=1とimu_frame:=...を指定することもできる。
VIO追跡と保存地図への再局在は別で、localization成功までは既存のAUTO許可条件を緩めない。
ROS/GPUでの実地図を用いたVIO実行は未検証。Map StudioのVIO地図生成はこの変更に含めない。

## VSLAMの速度・精度モード

bringup.shでVSLAMを使うdrive/offline evalには「VSLAM速度・精度」の選択を追加。
mapなしVO/VIO、保存mapのVO/VIO、VGL併用のいずれでも設定できる。
これは入力方式のtracking_mode（VO/VIO）とは独立したmulticam_mode。

| multicam_mode | 選択画面の説明 |
|---|---|
| 0 Moderate | 速度と精度のバランス |
| 1 Performance | 速度重視・処理負荷を抑える |
| 2 Precision | 精度重視・処理負荷は増える |

既定候補は対応する運用YAMLの値を読む。現在はlocalization/simとも1。
`--multicam-mode 0|1|2`で明示した場合は選択を省略する。collect/E2EはVSLAMを
起動しないためこの指定を拒否する。選択値はcuVSLAMだけへ渡し、VGLには渡さない。

```bash
bash scripts/bringup.sh --multicam-mode 2
# VIOとPrecisionを組み合わせたoffline evalをTUIで選ぶ
bash scripts/bringup.sh --tracking-mode 1 --multicam-mode 2 \
  --base-frame camera_link --imu-frame camera_imu_optical_frame
```

mission/localization/evaluation/sim_vslam.launch.pyはmulticam_mode引数を受け付ける。
launchの既定は空（YAML保持）で、明示指定のみ上書き。値は整数0/1/2を検証する。
入力topicやTF、bag再生内容、VO/VIO選択は変えない。
精度・負荷の説明はモードの狙いであり、実bag/GPUでの改善や処理時間を保証しない。
公式enum対応: [NVIDIA固定版実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/e4dc8f59816dfca387bcb92364ecaf9207216475/isaac_ros_cuvslam/src/impl/visual_slam_impl.cpp#L424)。

EVS encoder専用設定: `modules/sensors/openeb.launch.py`に`evs_tensor_config`引数を追加。
既定空文字は従来YAML、指定時は生成済みfull tensor YAMLへ置換。時間補間はnoneのみ。
これはセンサmoduleの明示指定であり、missionへの引数転送は未追加。
`/event_camera/tensor` (isaac_ros_tensor_msgs/msg/TensorList)のcontent解像度をモデルに合わせる。
CPU/CUDA/cuda_asyncそれぞれの既存full YAMLをbaseに生成すること。driverコード/既定設定は変更しない。

### 固定軸の対話切替

`tuning_collect.launch.py fixed_command:=true fixed_axes:=throttle`でスロットルのみ固定できる。
`fixed_axes`の既定は`both`、他に`steering`。非固定軸はJoy操作。
別端末で`bash scripts/vehicle_tuning.sh control`を実行し、STOP中に固定対象・
steering/throttle/brake/duration_sを一括変更する。再起動や録画停止は不要。
AUTOへの移行と録画はJoyで操作する。変更はYAMLへ保存しない。
`teleop_timeout_s`の既定は0.15秒、片軸固定の入力途絶でSTOP要求。
条件は`/control/fixed_config`（std_msgs/msg/String、JSON、1Hzと変更時）へ記録する。
詳細と全parameterは[kart_control README](../kart_control/README.md)。

### Map Studioの公式dense生成オプション

`config/mapping/workflow.json`に`dense_reconstruction`（既定false）と
`fs_model_res`（既定low_res、high_resも可）を追加。kart_mappingが公式offline CLIへ渡す。
ROSノード・topic・launch引数の追加はない。詳細はconfig/mapping/README.md参照。


## EVS E2E policy（2026-10-18）

[kart_e2eのEVS手順](../kart_e2e/README.md#evs-rawから制御policyまで2026-10-18)に
RAW dataset・制御head学習・ONNX・TensorRT buildを記載。
missionのe2eはONNX metadataのmodalityでRGB/EVSを選ぶ。
EVSにはenable_evs=trueと、モデルから生成した全OpenEB YAMLのevs_tensor_configが必須。
CLI追加`--evs-tensor-config`、mission追加`evs_tensor_config`は既定空（通常のsensor YAML保持）。
collectでも指定でき、EVSを有効化してsensor moduleへ渡す。

```bash
bash scripts/bringup.sh --mode e2e --evs \
  --e2e-model-dir /workspaces/models/evs_control_v1 \
  --evs-tensor-config /workspaces/models/evs_control_v1/openeb_tensor.yaml \
  --rgb-fps 0 --infra-fps 0
```

EVSのTensorRTNode（e2e_tensor_rt）＋kart_e2e::ControlDecoderNode（e2e_control_decoder）は
既存/event_camera/tensor_containerへloadし、driverやRGB encoderを重複起動しない。
RGB/Infraが両方0のEVS E2EではRealSense/IMUと空のsensor containerも起動しない。
片方が有効ならRealSenseを併用記録できる。localizationは起動しない。
missionは既存e2e_drive_enabled=trueでAUTO muxへ接続するが、AUTO／録画は自動開始しない。

単体evs_e2e.launch.pyは既存EVS processへのload専用でdrive_enabled=false。
全引数（すべて既定空＝config保持）はmodel_dir、evs_tensor_config、container_name、input_topic、
fixed_throttle、drive_enabled、use_sim_time。前2者は必須。
container_name=/event_camera/tensor_container、input_topic=/event_camera/tensorは構成正本
config/e2e/evs_launch.jsonで固定。その他の意味・既定実効値は既存decoderと共通。
全ROS parameterはconfig/e2e/evs_tensor_rt.yamlとcontrol_decoder.yaml。
TensorRT入力/topic型は/event_camera/tensor / isaac_ros_tensor_msgs/msg/TensorList、
出力/e2e/tensor_outputは同型、reliable/volatile/depth1。decoderの入出力・QoS・既定値は既存E2Eと同じ。
独自service/actionは追加なし。ROS標準自動topicは省略。
既存e2e.launch.pyはRGB専用としてEVSモデルの誤接続を拒否する。
構成とbinding照合を代替fixtureで検証。実SDK/ROS/Jetson/車両は未確認。

Jetsonの通常buildは`kart_sim`を探索対象から除外する。simを使う場合は上記の
`--with-sim`で明示的に含める。実機用bringupの起動ではsim packageは不要。


## AUTO固定スロットル・手動ステア

`auto_fixed_throttle=-1.0`（既定）は通常動作。0..speed_scale（標準上限0.3）を
指定すると、□AUTOで固定throttle＋手動ステア、△MANUALで通常R2/L2＋手動ステアになる。
AUTO中もステアを操作でき、△でSTOPを挟まずMANUALへ直接引き継げる。
正常なMANUAL→AUTOも直接切替可能。STOPからの初回進入は全入力中立が必要で、
AUTO初回進入は0.3秒中立を配信してmux/bridgeのHOST handshakeを待つ。
直接切替時は中立待ちを挟まない。×はSTOP、固定AUTO中の○はbrake優先＋STOP、L2は中立＋STOP。
入力途絶・異常・再起動後はSTOPから再armし、モード変更だけで異常解除しない。
固定値は速度ではなく補正前の正規化要求で、起動時固定。変更はSTOP後に再起動する。

JoyManagerNodeが`/teleop/control_cmd`（kart_interfaces/msg/ControlCommand、reliable/depth1）へ
元Joy stampで合成指令を出版する。bringupは固定設定からcommand_muxの
`auto_use_teleop`（bool、ノード既定false）を導出し、有効時はAUTOでもこの入力を選ぶ。
この間はE2Eなどの`/auto/control_cmd`を車両へ流さない。
`/teleop/fixed_config`（std_msgs/msg/String、reliable/volatile/depth1、10Hz）は
`kart.auto_fixed_throttle.v1`、enabled、throttle、speed_scale、entry_neutral_s、mode、armedを記録する。
armedは論理的許可で、実車速度や基板ackを示さない。独自service/actionは追加なし。
MANUALは通常のthrottle trim、固定AUTOは既存bridgeのAUTO補正規則を使用する。
録画はR1開始／L1停止（bringup標準）で、走行モード操作とは独立。

```bash
bash scripts/bringup.sh --mode collect --auto-fixed-throttle 0.10 --run-name auto_fixed_010
# EVSの場合は --evs を追加
```

CLI/launchの旧`manual_fixed_throttle`は`auto_fixed_throttle`へ変更した。
実運用設定はkart_bringup/config/input/joy_manager.yaml、未指定ならYAMLを保持する。
TUIの「AUTOスロットル」で通常／固定を選ぶ。collect/driveで利用可能。E2Eでは収集用の固定設定を無効化し、モデルのステア＋E2E専用固定スロットルを使用する。
RGB/EVS教師抽出はbag内の固定AUTO有効設定を自動検出する。固定収集bagでは固定AUTOだけを採用し、MANUALの復帰・調整区間を全て除外する。通常bagは従来どおりMANUALだけを採用する。
ROS結合・実車動作は未確認。portable testで固定出力、直接切替、異常ラッチ、HOST handshakeを検証する。

## 収集用固定スロットルとE2E走行の分離

- collect/driveの`--auto-fixed-throttle`（Joy parameter `auto_fixed_throttle`）は手動ステア収集用。
- `--mode e2e`はAUTOでモデルのステア＋固定スロットル。収集用Joy固定は必ず-1へ上書きし、
  muxの`auto_use_teleop=false`で`/auto/control_cmd`を車両へ通す。MANUALは通常の手動操作。
- E2E専用CLI `--e2e-fixed-throttle`／mission引数 `e2e_fixed_throttle` は未指定なら
  metadataのruntime.fixed_throttleを使い、明示時は0..runtime.max_throttleの有限値に限る。
  TUIもE2E専用の入力欄を使い、収集用の固定選択を表示しない。
- missionはRGB/EVSの双方に`use_fixed_throttle=true`を渡す。モデルがsteer_throttleでも
  ステアだけを操舵に用い、駆動量は固定値。モデルの出力数・形式は書き換えない。
- decoderの新しい起動時parameter `use_fixed_throttle` はbool、ノード/YAML既定false。
  単独launchにも同名の任意引数を提供する。既存topic・QoS・watchdog・STOP処理は同じ。

```bash
bash scripts/bringup.sh --mode e2e --e2e-model-dir /workspaces/models/policy \
  --e2e-fixed-throttle 0.10
```

0.10は車速ではなく正規化した固定要求。起動だけでは走行せず、□でAUTO、△でMANUAL、×でSTOP。
C++ gateとlaunch転送・CLIをportable testで確認する。ROS結合・GPU推論・実車操舵は未確認。
