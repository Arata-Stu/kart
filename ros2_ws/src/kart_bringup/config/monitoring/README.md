# Jetson監視

公式`isaac_ros_jetson_stats`の実行ファイル`jtop`、node名`/jtop`を使う。
CPU/GPU負荷・周波数、RAM/SWAP/EMC、温度、電力、fan、board/disk情報を
`/system/jetson/diagnostics`へ配信する。項目は搭載機種/JetPack/jtopで変わる。
温度等はDiagnosticStatus.valuesのkey/valueに格納され、独立Float topicではない。

出典: [公式node実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_jetson/blob/0f311a1dcd1164aec7b75648df336e45fb53ffa2/isaac_ros_jetson_stats/isaac_ros_jetson_stats/ros2_jtop_node.py)、
release-5.0、2026-10-08確認。APT実版はMac環境では未確認。

## 起動と所有者

- 通常は`vehicle.launch.py`が所有する。bringup.yamlのenable_jetson_stats=trueが既定。
- moduleは`launch/modules/monitoring/jetson_stats.launch.py`。container作成/loadはせずPython processを起動する。
- localization/trackingは監視を起動しない。vehicleと併用してもこの構成からの二重起動を避けられる。
- 他launchから手動で同じmoduleを重ねて起動する場合の重複検知はない。1台につき所有者を1つにする。
- node名/出力topicはホスト共通のroot名。vehicle namespaceには追従しない。
  同一ROS domainの複数Jetsonでは、ホスト別topic/node namespaceへ明示的に変更する。

単独の監視（vehicleを起動しない試験用）:

```bash
ros2 launch kart_bringup jetson_stats.launch.py mode:=on
ros2 topic hz /system/jetson/diagnostics
ros2 topic echo /system/jetson/diagnostics
```

独自launch設定はmonitoring/launch.jsonの`mode=auto`。
module引数modeは既定空文字でconfigを保持し、auto/on/offのみを明示上書きできる。
autoはARM64かつL4T識別ファイル/device-treeのJetson名/jtop socketで判定する。
x86_64や通常のARM notebookではpackageを解決せずskipする。
autoでJetsonなのにpackage/socketがない場合は「記録されない」旨をログへ出してskipする。
onは明示要求として不足時にlaunchエラー。offは起動しない。
起動後のnode異常終了は自動再起動/車両停止へ連動しない。録画にtopicがあることを実行時に確認する。

## 全parameter・入出力

ROS parameter正本はjetson_stats.yaml。上流独自parameterはintervalのみ、全て列挙済み。

| 名前 | kart値 | 意味 |
|---|---|---|
| interval | 0.5秒 | node timerとjtopクライアントの要求周期。共有jtop serverの実周期と一致する保証はない |
| use_sim_time | false | 実ハードウェアの時刻。bag replayのclockに合わせない |

intervalは起動時に使用され、実行中のsetだけではtimer再作成を行わない上流実装なので変更時は再起動する。

| 方向 | topic | 型 | QoS |
|---|---|---|---|
| 出力 | /system/jetson/diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth1、既定2Hz |

上流の相対`diagnostics`をremap。ROS標準rosout/parameter_events等は省略。
公式launchが起動するdiagnostic_aggregatorは使用しない。生のDiagnosticArrayだけを録画するため不要。
上流はハードウェア対応時に以下のserviceも公開するが、kartのlaunchから呼出さない。

| service | 型 | 上流機能 |
|---|---|---|
| /jtop/fan | isaac_ros_jetson_stats_services/srv/Fan | fan設定 |
| /jtop/jetson_clocks | isaac_ros_jetson_stats_services/srv/JetsonClocks | clocks設定 |
| /jtop/nvpmodel | isaac_ros_jetson_stats_services/srv/NVPModel | 電力mode設定 |

## Docker/host前提

Dockerfile.kartにはARM64限定でROS packageとjetson-statsを導入済み。
notepcへこのJetson専用依存を必須追加しないためpackage.xmlには無条件のexec_dependを追加しない。
プラットフォーム限定依存はDockerのARM64分岐で供給し、moduleが実行時に存在確認する。

ホスト上のjtop.serviceが必要。既存の[ホスト準備手順](../../../../../README.md#jtop-setup)を参照。コンテナのPythonクライアントとホストserviceの版を揃える。
このrepoはdocker/jetson-stats.envに7.2.0/commitを固定しており、無条件の最新版upgradeを混ぜない。
既存CLIはjtop groupがある場合に/run/jtop.sockをmountしgroupを追加する。
service停止、socket未mount、group権限不足、host/client版不一致を切り分ける。
launchはhostへのinstall、systemctl操作、fan/clock/power設定変更を行わない。

## rosbag

既存bag managerの既定topicリスト（package/bringup/コードfallback）へ
`/system/jetson/diagnostics`を追加済み。通常の録画START/STOPに含まれる。
監視起動だけでは録画を開始しない。独自bag_configでtopics/excludeを変更している場合は別途追加する。
no_discovery=falseなので、録画開始後に現れたtopicも発見対象となる。

```bash
ros2 bag info /path/to/bag
# /system/jetson/diagnosticsとmessage countを確認
```

bagに保存するのは構造化された測定topicであり、端末に出るjtopの文字表示ではない。
FoxgloveのRaw Messagesで値を確認し、温度/負荷/電力を走行・推論遅延と同時系列で解析できる。
node未起動/異常終了時は測定が欠落する。bag全体の成功だけで監視データの存在を判断しない。

## 検証

portable testsはJetson/ARM notebook/x86の判定、mode切替、録画リストを検査する。
ROS node起動、実センサ項目・温度、socketアクセス、実bagへの記録はJetsonで未確認。
