# kart_interfaces

車両指令・動作モード・トリムのROSメッセージ定義。JetPilotの制御用3msgと同じフィールドを維持し、
`ControlTrim`と録画用`BagRequest`/`BagStatus`を追加している。パッケージ名が異なるためROS上ではJetPilot型と別の型になる。

## ノード・入出力・パラメータ

このpackage自身はノードを提供しない。**入力topic・出力topic・既定ノード名・パラメータはなし**。
下表は他packageがこれらの型を使用する既定topic名（namespaceなし・remapなし）。
メッセージ型そのものがtopic名やQoSを決めるわけではない。

| 型 | 既定topic名 | 送信元 → 受信先 |
| --- | --- | --- |
| `kart_interfaces/msg/ControlCommand` | `/teleop/control_cmd` | joy manager → mux |
| 同上 | `/auto/control_cmd` | 外部の自律制御ノード → mux |
| 同上 | `/vehicle/control_cmd` | mux → bridge |
| 同上 | `/propo/control_cmd` | bridge → 記録・観測用consumer |
| `kart_interfaces/msg/OperationModeRequest` | `/operation_mode/request` | joy manager・mux・bridge等 → mode manager |
| `kart_interfaces/msg/OperationModeState` | `/operation_mode/state` | mode manager → joy manager・mux・bridge |
| `kart_interfaces/msg/ControlTrim` | `/vehicle/trim/request` | joy manager → bridge |
| 同上 | `/vehicle/trim/state` | bridge → 記録・観測用consumer |

録画用の既定topic:

| 型 | 既定topic名 | 送信元 → 受信先 |
| --- | --- | --- |
| `kart_interfaces/msg/BagRequest` | `/bag/request` | joy manager等 → bag manager |
| `kart_interfaces/msg/BagStatus` | `/bag/status` | bag manager → 観測・記録用consumer |
| `kart_interfaces/msg/BagRequest` | なし（既定無効） | bag manager → 任意設定のRAW録画要求topic |

config/には[パラメータがない旨の説明](config/README.md)を置く。架空のparameter YAMLは作成しない。

## フィールド定義

### ControlCommand

| フィールド | 型 | 意味 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 元指令の時刻とframe。muxでstampを更新しない |
| `steering` | `float32` | −1..1、左正 |
| `throttle` | `float32` | 0..1、前進出力 |
| `brake` | `float32` | 0..1、専用ブレーキ出力 |
| `reverse` | `float32` | 0..1、負スロットル側の出力 |

速度・舵角などの物理単位ではなく正規化指令。駆動3フィールドの同時正値、非有限値、範囲外は
consumerで拒否する。全ゼロは中立で、制動指令ではない。L2側の実際のブレーキ／後退はESCに依存する。

### OperationModeRequest / OperationModeState

両者は同じフィールド構成。

| フィールド／定数 | 型・値 | 意味 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 要求／状態の時刻 |
| `mode` | `uint8` | 以下のモード番号 |
| `source` | `string` | 操作・異常などの要求元情報 |
| `AUTO` | `1` | 自律指令をHOST経路へ |
| `MANUAL` | `2` | Joy指令をHOST経路へ |
| `STOP` | `3` | HOSTを解除し中立へ |
| `PROPO` | `4` | 物理RC経路 |

AUTO/MANUALはいずれも基板HOST経路。STOPは物理RC経路を停止させる機能ではない。

### ControlTrim

| フィールド | 型 | 意味 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 要求／状態の時刻 |
| `steering` | `float32` | 左正の補正 |
| `throttle` | `float32` | 前進正の補正 |

requestでは**増分**、stateでは**現在の累積値**。固定`steering_offset`はstateに含まない。
適用範囲・制限は[kart_vehicle](../kart_vehicle/README.md)を参照。

### BagRequest / BagStatus

`BagRequest`: `std_msgs/Header header`、`uint8 command`、`string label`。
定数はSTART=1、STOP=2、SPLIT=3、MARK=4。labelは開始時の命名やマークに使用する。

`BagStatus`: `std_msgs/Header header`、`bool recording`、`string current_uri`、
`string last_event`、`string message`。current_uriは最後の試行先、messageは状態。
recordingは子の生存＋出力ディレクトリ生成確認であり、記録内容の完全性を保証しない。
詳細は[kart_bag_manager](../kart_bag_manager/README.md)を参照。

## ビルド・確認

```bash
cd /workspaces/ros2_ws
colcon build --symlink-install --packages-select kart_interfaces
source install/setup.bash
ros2 interface show kart_interfaces/msg/ControlCommand
ros2 interface show kart_interfaces/msg/ControlTrim
```

`ament_cmake_auto`と`rosidl_default_generators`を使用する。
ROS上での生成・結合ビルドは未検証。各consumerの単体ロジックテストは別packageで管理する。

### ReferenceLine

`std_msgs/Header header`、`string lane_id/line_type`、`bool closed`、
`geometry_msgs/Point[] points`、`float64[] speeds`。
同一map座標のオフライン参照。点列順が前進方向、速度はm/s・非負で各点1値。
閉路は始点を末尾へ重複追加しない。空点列は以前の参照を無効化する。
header stampは配信時刻であり、地図座標のTF検索時刻ではない。

### TrackingReference

Pure Pursuitが実際に選んだ追従点の観測用メッセージ。
既定topicは`/control/tracking_reference`、送信元は`kart_control/pure_pursuit`、
受信先はMap Studio調整bridge・Foxglove等。reliable/volatile/depth1、既定30 Hz。

| フィールド | 型 | 意味 |
| --- | --- | --- |
| header | std_msgs/Header | 有効時は制御に使ったposeの計測時刻・map座標。無効時は通知時刻 |
| valid | bool | 制御の参照点計算が成功したか。falseなら位置・距離を表示しない |
| vehicle_position | geometry_msgs/Point | 制御に使用した車両基準位置のXY、z=0 |
| target | geometry_msgs/Point | 実際の追従点XY、z=0 |
| lookahead_m | float64 | 設定した前方経路長m。最近傍経路投影点が起点 |
| target_distance_m | float64 | vehicle_positionからtargetへのXY直線距離m |

表示用の再計算で代用しない。AUTOモードや車両への出力許可を表すメッセージではない。
ノード・parameterの追加なし。型生成・ROS結合ビルドはROS環境で別途確認する。

`E2EHeadSelection`は`/e2e/head_selection`（reliable / volatile / depth 10）で
section selector → E2E decoderへ送信する。headerは判定対象画像と同じ時刻、
`valid`、`section_id`、`head_id`、0始まり`head_index`、`routing_sha256`、`reason`を持つ。
未局在・曖昧・時刻不整合はvalid=false、head_index=-1。位置そのものや操作指令ではない。

`ReferenceLine.step_speed`（bool、既定false）を追加。trueはCustomlineの各点から次点まで
始点の目標速度を保持し、境界で即時切替する指定。通常の速度補間・境界先行減速を省くが、
走行側の上限と開路終点停止は維持する。publisherはkart_hdmap、consumerはkart_control。
独自parameter追加なし。型変更時は関連workspaceを再ビルド・source・ノード再起動する。
