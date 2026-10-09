# kart_bringup

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

bridge無効時はbridge由来の配信・購読は存在しない。自律指令源・GUI/CUIは本launchでは起動しない。bag managerは待機状態で起動し、L1で録画を開始する。
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
bag_shutdown_timeout:=26等で終了猶予を確保する。カメラは別途openeb_ros2のkart branchで起動する。
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
