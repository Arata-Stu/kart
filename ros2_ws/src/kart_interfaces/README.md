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

### HeadToHeadObservation / HeadToHeadPlan

相手1台の実験用Frenetインターフェース。`/racing/observation`をkart_racingが購読し、
`/racing/shadow_plan`へ出力する。各fieldの単位と契約はmsgコメントと
[kart_racing README](../kart_racing/README.md)を参照。
元stamp、reference_id（地図/ラインrevision）、同一周回原点のs、左正d、相手IDと分散を保持。
observed=falseと空いたコースを混同しない。出力は空間スプラインと速度目標のshadow情報。
有効期限は元観測時刻から計算し、時刻付き動的安全軌道や車両指令とは扱わない。

`InfraStampResult` はInfra時刻検証の受信1件分。streamは左画像=0、右画像=1、
左CameraInfo=2、右CameraInfo=3。stamp_nsは元の撮影時刻、sequenceは入力別の受信連番、
interval_nsは前回との符号付き時間差、expected_frames_missingは公称周期換算の推定欠け数、
seen_maskは同じstampでここまで観測したstreamのビット集合。

`OpponentGroundPoints`: 撮影時刻と車体frameのheader、および
`OpponentGroundPoint[] tracks`（string track_id、geometry_msgs/Point point、単位m）。
平坦路仮定の可視接地点。空配列は推定点なし。

`GroundDetections`はRCカー・障害物の推定接地点。既定は表示用で、単体既定topicは`/map_points`、
bringupでは`/racing/map_points`、publisherは`kart_opponent_tracker/perception_ground`、
consumerはFoxgloveと`kart_hdmap/hdmap_server`のStudio bridge、明示したlive behaviorの
`kart_racing/racing_behavior`。best effort / volatile / depth 1。
`header`は有効結果では元画像の撮影時刻と変換済みframe（既定`map`）。watchdogの無効な消去通知は
通知時刻を使う。`GroundDetection[] objects`の各要素は
`string detection_id`（画像内識別子であり永続trackではない）、元の`string class_id`、
profileで確定した`string class_name`（`rc_car` / `obstacle`）、`float32 score`（0..1）、
`geometry_msgs/Point point`（単位m）を持つ。bbox下端中央と平坦路からの概算であり、物体中心・
3D寸法・占有領域・安全な自由空間を表さない。配列全体の`bool valid`と`string reason`で、
正常な空結果（`zero_detections` / `zero_selected`）と失敗・timeout消去を同じメッセージ内で区別する。
`valid=false`の空配列を0件と表示しない。詳細は投影診断で確認する。
既存の`OpponentGroundPoints`とは確認状態・IDの意味を分ける。通常の認識起動だけで制御へ接続せず、
明示的な`overtake_mode=active`ではbehaviorの観測入力となる。接地点を3D寸法・自由空間の証明とせず、
behavior側の鮮度・map/frame・占有仮定・停止条件の検証を省略しない。新型を使うpublisher/consumerは
同じworkspaceで再ビルド・source・再起動する。型生成とROS結合確認はROS環境で別途必要。

PassingCorridor は直線区間の実測/外部認定。地図frame・時刻・参照revision・対象ID、unwrapped sの
開始/終了[m]、区間全体の左右空き幅[m]、速度上限[m/s]、clearを持つ。側方・合流まで観測可能な場合だけclear=true。

HeadToHeadObservation.measurement_stampは位置速度の撮影時刻。header.stampは通路情報も含む最古時刻。measurement_stamp=0はheader.stampを用いる。
