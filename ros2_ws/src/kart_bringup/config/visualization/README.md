# Foxglove Bridge

`ros2 launch kart_bringup foxglove.launch.py`で一台につき一度起動する。
NotebookのFoxgloveから `ws://<JetsonのLAN IP>:8765` へ接続する。
Dockerは既存のhost network構成を使用する。依存を追加したためDockerの再buildが必要。
launchはカメラ・localization・車両を起動しない。それらは既存launchを別途起動する。
Bridgeは専用プロセスを所有し、既存component containerへloadしない。
vehicle/localization/e2e launchへ重複includeしない。ROS namespaceはroot固定。

## 公開範囲

| 方向 | topic/service | ROS型 | 用途 |
|---|---|---|---|
| ROS→表示 | /tf, /tf_static | tf2_msgs/msg/TFMessage | 地図・車体座標 |
| ROS→表示 | /clock | rosgraph_msgs/msg/Clock | replay時計 |
| ROS→表示 | /hdmap/markers | visualization_msgs/msg/MarkerArray | 境界・禁止領域・line |
| ROS→表示 | /hdmap/{centerline,raceline,customline} | nav_msgs/msg/Path | 生成ライン |
| ROS→表示 | /hdmap/selected_lane | std_msgs/msg/String | 選択lane ID |
| ROS→表示 | /planning/reference_line | kart_interfaces/msg/ReferenceLine | 選択走行ライン |
| ROS→表示 | /visual_slam/tracking/odometry | nav_msgs/msg/Odometry | VO |
| ROS→表示 | /visual_slam/tracking/vo_pose | geometry_msgs/msg/PoseStamped | VO姿勢 |
| ROS→表示 | /visual_slam/tracking/vo_pose_covariance | geometry_msgs/msg/PoseWithCovarianceStamped | VO姿勢と共分散 |
| ROS→表示 | /visual_slam/tracking/{vo_path,slam_path} | nav_msgs/msg/Path | 軌跡 |
| 双方向 | /localization/pose_hint | geometry_msgs/msg/PoseWithCovarianceStamped | VGL出力／手動初期姿勢→cuVSLAM |
| 双方向 | /visual_localization/trigger_localization (topic) | geometry_msgs/msg/PoseWithCovarianceStamped | VGL要求、値は使用しない |
| Client→service | /visual_localization/trigger_localization | std_srvs/srv/Trigger | VGL要求、responseは受付であり成功位置の保証ではない |
| ROS→表示 | /localization/{vslam,vgl}/diagnostics, /system/jetson/diagnostics, /control/speed_diagnostics | diagnostic_msgs/msg/DiagnosticArray | localization／温度・負荷／PID |
| ROS→表示 | /operation_mode/state | kart_interfaces/msg/OperationModeState | 走行モード |
| ROS→表示 | /bag/status | kart_interfaces/msg/BagStatus | 録画状態 |
| ROS→表示 | /control/tracking_cmd | ackermann_msgs/msg/AckermannDriveStamped | 追従指令 |
| ROS→表示 | /control/tracking_status, /e2e/status | std_msgs/msg/String | 制御状態 |
| ROS→表示 | /control/lookahead | geometry_msgs/msg/PointStamped | 注視点 |
| ROS→表示 | /e2e/control_cmd, /auto/control_cmd, /vehicle/control_cmd | kart_interfaces/msg/ControlCommand | 指令の観測のみ |
| ROS→表示 | /vehicle/trim/state | kart_interfaces/msg/ControlTrim | トリム状態 |
| ROS→表示 | /vehicle/{rc_channels,output_channels} | std_msgs/msg/Int32MultiArray | PWM値 |
| ROS→表示 | /vehicle/vbec | std_msgs/msg/Float32 | BEC電圧 |
| ROS→表示 | /vehicle/active_path | std_msgs/msg/UInt8 | 基板の経路 |

画像・特徴点・tensor・HDMap全文JSON・rosout・parameter_eventsなどは非公開。
`connectionGraph`も無効にして、除外topicをグラフ経由で一覧化しない。
Clientからの書込みは上記2topicと1serviceだけ。TF、AUTO指令、モード変更、録画操作、
SetSlamPose、パラメータ変更、ファイル取得は公開しない。
制限はFoxglove経由のアクセス対象であり、ROS DDS全体のアクセス制御ではない。

## Foxgloveでの操作

- 3D表示を`map`にする。Publish → **2D pose estimate**のtopicを
  `/localization/pose_hint`へ設定。`map`座標における`base_link`の位置・向きを送る。
  `/initialpose`はこの構成では受信しない。保存済みcuVSLAM地図と動作中カメラが必要。
- cuVSLAMは姿勢を探索ヒントとして使用する。共分散は探索半径に反映されず、
  `config/localization/cuvslam.yaml`のlocalizer設定を使用する。
  現行設定では失敗時にVGLへ別のヒントを要求する。
- VGLはService Callパネルで上記Trigger serviceを呼ぶか、Publishパネルで
  `/visual_localization/trigger_localization`へPoseWithCovarianceStampedを送信する。
  後者はテンプレート値のままでよい。メッセージ内容は位置指定にならない。
- VGL結果は`/localization/pose_hint`へ入り、cuVSLAMで再localizationする。
  最終結果はVSLAM diagnosticsと`map→odom`／slam_pathで確認する。
  再localizationで地図上の姿勢は変わり得るため、手動指定はSTOP中に行う。
- topic名をworkflow.jsonで変更した場合、このallowlistとFoxglove送信先も合わせる。

## 設定・出典

正本は[foxglove.yaml](foxglove.yaml)。ノード名`/foxglove_bridge`、
package/executableは`foxglove_bridge`。独自launch引数なし。
全34個の独自parameterと`use_sim_time`を明示し、既定値・意味はYAML内コメントに記載。
QoSはpublisherに合わせる。depthは1〜10、transient localのstatic TFを維持する。
外部ノードのROS標準parameter/serviceは一覧省略。Bridgeからの公開は制限する。

対応版はLyrical rosdistroの`3.6.0-1`。APTの実インストール版は未確認。
[配布版ソース](https://github.com/ros2-gbp/foxglove_bridge-release/tree/release/lyrical/foxglove_bridge/3.6.0-1)
の`src/param_utils.cpp`と`include/foxglove_bridge/param_utils.hpp`を確認。
全独自parameterを記載。上流launchの値を重ねない。`num_threads`はこの版で宣言されていない。
`disable_load_message`は上流キーの綴りのまま。
`sysinfo=false`でjtopへ集約。remote_access=falseでクラウドgatewayを使用しない。
capabilitiesはclientPublish/servicesのみ、asset/paramはnever-match、queueを縮小。
待受0.0.0.0:8765・TLS無効はロボットLANでの接続用。インターネットへの公開は想定しない。
APT更新時には対応パラメータを再確認する。

検証: allowlistの許可／拒否とlaunchのPython構文をローカルで検査。
ROS Bridge起動・WebSocket通信・実Jetson localizationは未確認。

## NotebookからFoxgloveを開く

Notebookのホスト側（Docker外）で、プロジェクトルートから実行する。

```bash
./scripts/open-foxglove.sh jetson.local
# IPやportの指定も可能
./scripts/open-foxglove.sh 192.168.1.20:8765
# ブラウザ版／URL表示だけ
./scripts/open-foxglove.sh --web ws://localhost:8765
./scripts/open-foxglove.sh --print-url jetson.local
```

引数なしは共通`config/jetson_hosts.json`の既定値`ws://10.42.0.1:8765`。
`--preset notebook|lan|usb`で選択、`--list`で一覧表示。環境変数`KART_FOXGLOVE_URL`でも既定先を指定可能。
macOSの`open`、Linuxの`xdg-open`とPython 3標準ライブラリを使用する。
[公式deep link](https://docs.foxglove.dev/docs/visualization/shareable-links)の
`openIn=desktop`でアプリを開く。未導入なら公式の案内ページが表示される。
Bridgeの起動・SSH接続・localization要求はこのスクリプトでは行わない。
