# kart_system

Joyの車両操作への変換、動作モード管理、手動／自動指令の選択を担当する。
デバイスからの入力取得は別packageの[kart_joy](../kart_joy/README.md)。

## ノードの既定名

| ROSノード名 | 実行ファイル | Component |
| --- | --- | --- |
| `/kart_joy_manager` | `kart_joy_manager_node` | `kart_system::JoyManagerNode` |
| `/operation_mode_manager` | `kart_operation_mode_manager` | `kart_system::ModeManagerNode` |
| `/command_mux` | `kart_command_mux` | `kart_system::CommandMuxNode` |

以下はnamespaceなし・remapなしの既定名。実装は相対topic名を使うため、namespaceが
`kart`なら`/joy`は`/kart/joy`になる。ROS標準の`/rosout`、`/parameter_events`等は表から省略。
QoSはすべてvolatile、keep-last。R=reliable、BE=best effort、数字=depth。

## 入力・出力topic

### kart_joy_manager

| 方向 | 既定topic名 | 型 | QoS | 内容 |
| --- | --- | --- | --- | --- |
| 入力 | `/joy` | `sensor_msgs/msg/Joy` | R/1 | 正規化した軸・ボタン |
| 入力 | `/joy/ready` | `std_msgs/msg/Bool` | R/1 | 入力・profileが有効かつ設定中でないこと |
| 入力 | `/operation_mode/state` | `kart_interfaces/msg/OperationModeState` | R/1 | 現在モード |
| 出力 | `/teleop/control_cmd` | `kart_interfaces/msg/ControlCommand` | R/1 | 有効なJoy受信時に手動指令を配信。元stampを保持 |
| 出力 | `/operation_mode/request` | `kart_interfaces/msg/OperationModeRequest` | R/10 | モード切替・入力喪失時のSTOP要求 |
| 出力 | `/bag/request` | `kart_interfaces/msg/BagRequest` | R/10 | L1 START / R1 STOP。走行モードと独立 |
| 出力 | `/vehicle/trim/request` | `kart_interfaces/msg/ControlTrim` | R/10 | ボタン押下時の補正増分 |

`TeleopInput`は連続入力、`JoyButtons`は押下edgeを処理する内部クラス。
接続・鮮度・deadman・操作許可は一つのJoyManagerNodeが管理する。
初回／再接続時に押されていたボタンからはedgeを生成しない。

### operation_mode_manager

| 方向 | 既定topic名 | 型 | QoS | 内容 |
| --- | --- | --- | --- | --- |
| 入力 | `/operation_mode/request` | `kart_interfaces/msg/OperationModeRequest` | R/10 | モード要求 |
| 出力 | `/operation_mode/state` | `kart_interfaces/msg/OperationModeState` | R/1 | 10 Hz＋要求受理時の状態 |

初期モードはSTOP。AUTO=1、MANUAL=2、STOP=3、PROPO=4。
HOSTモードへの切替はSTOPを経由する。AUTO/MANUALの直接切替は拒否する。
STOP以外の要求stampは500 ms以内が必要。STOPのみstamp省略可。

### command_mux

| 方向 | 既定topic名 | 型 | QoS | 内容 |
| --- | --- | --- | --- | --- |
| 入力 | `/auto/control_cmd` | `kart_interfaces/msg/ControlCommand` | BE/1 | 自律制御指令 |
| 入力 | `/teleop/control_cmd` | `kart_interfaces/msg/ControlCommand` | BE/1 | 手動指令 |
| 入力 | `/operation_mode/state` | `kart_interfaces/msg/OperationModeState` | R/1 | 選択するモード |
| 出力 | `/vehicle/control_cmd` | `kart_interfaces/msg/ControlCommand` | R/1 | 有効なHOSTモード中のみ100 Hzで元stampを保持して配信 |
| 出力 | `/operation_mode/request` | `kart_interfaces/msg/OperationModeRequest` | R/10 | 指令・モード喪失などによるSTOP要求 |

STOP/PROPOでは指令を配信しない。起動・異常後はSTOPを観測してから新しいHOST遷移を要求する。
指令の受信時刻と元stampを両方確認し、異常から回復しただけでは再armしない。

## パラメータ

独自パラメータは起動時固定（read-only）。単体起動用の全parameterは
[joy_manager.yaml](config/joy_manager.yaml)、[operation_mode_manager.yaml](config/operation_mode_manager.yaml)、
[command_mux.yaml](config/command_mux.yaml)に記載する。mode managerは共通のuse_sim_timeのみ。
実運用は[kart_bringup/config](../kart_bringup/config/)のinput/・system/を使う。
個別pkgのconfig変更はbringupへ自動反映しない。共通の`use_sim_time`はYAMLで既定false。

### kart_joy_manager

軸・ボタンの値は`kart_joy`の正規化後配列index。通常のindexは0..63、`auto_button`・`deadman_button`・`bag_start_button`・`bag_stop_button`は-1で無効化可。

| 名前 | 既定値 | 説明 |
| --- | --- | --- |
| `steering_axis` | `0` | 左スティックX。右正入力をROS左正へ変換 |
| `throttle_axis` | `5` | R2、静止0／最大1 |
| `reverse_axis` | `4` | L2、静止0／最大1 |
| `bag_start_button` | `4` | L1。録画START要求 |
| `bag_stop_button` | `5` | R1。録画STOP要求。同時押しはSTOP優先 |
| `deadman_button` | `-1` | 保持条件なし。4にするとL1をMANUAL操作とモード・トリム要求時に保持 |
| `brake_button` | `1` | ○。専用brake=1を優先 |
| `stop_button` | `0` | ×。STOP要求、deadman不要 |
| `manual_button` | `2` | △。中立かつSTOP時にMANUAL要求 |
| `auto_button` | `3` | □。中立かつSTOP時にAUTO要求 |
| `steering_trim_left_button` | `15` | D-pad左、ステア補正を正へ |
| `steering_trim_right_button` | `16` | D-pad右、ステア補正を負へ |
| `throttle_trim_up_button` | `13` | D-pad上、スロットル補正を正へ |
| `throttle_trim_down_button` | `14` | D-pad下、スロットル補正を負へ |
| `steering_trim_step` | `0.001` | 1押下のステア補正増分。0.001..0.05 |
| `throttle_trim_step` | `0.001` | 1押下のスロットル補正増分。0.001..0.05 |
| `deadzone` | `0.08` | ステア入力の不感帯。0..0.5、外側を再正規化 |
| `steering_scale` | `1.0` | ステア倍率。0.01..1 |
| `speed_scale` | `0.3` | 正負スロットル倍率。0.01..1。車速[m/s]ではない |

スロットルは`(R2 − L2) × speed_scale`。各トリガー0.01以下を0とし、負側は`reverse`へ格納する。
両方同量なら相殺するが、モード要求時には両トリガーを離す必要がある。
Joyとreadyの有効期間は100 ms、監視は100 Hz（固定値）。MANUAL中の切断・設定開始・入力途絶はSTOP。deadmanを設定した場合はその解除でもSTOP。
既定は△だけでMANUALを要求し、△を離しても継続する。×でSTOP。□・D-padにもL1は不要。
AUTO中はJoy切断や、任意設定のdeadman解除だけではSTOPにしない。
L1/R1は録画操作に使用する。任意でdeadmanを設定する場合、録画ボタンとの重複を避ける。
録画処理は[kart_bag_manager](../kart_bag_manager/README.md)が担当する。Joy切断や車両STOPで録画を自動停止しない。
トリムの現在値・適用・ログの責務は[kart_vehicle](../kart_vehicle/README.md)にある。

### operation_mode_manager / command_mux

operation_mode_manager独自の設定パラメータはない（STOP起動、10 Hz固定）。

| ノード | 名前 | 既定値 | 説明 |
| --- | --- | --- | --- |
| command_mux | `command_timeout` | `0.15` s | 受信時刻・元stampの有効期間。0.02..0.2 s |

muxの配信周期100 Hz、mode監視300 msは固定。

## ビルド・起動・検証

```bash
cd /workspaces/ros2_ws
colcon build --symlink-install --packages-up-to kart_system
source install/setup.bash
ros2 run kart_system kart_joy_manager_node --ros-args --params-file /workspaces/ros2_ws/src/kart_system/config/joy_manager.yaml
# 別ターミナルで個別起動する場合
ros2 run kart_system kart_operation_mode_manager
ros2 run kart_system kart_command_mux
```

全体の同時起動は[kart_bringup](../kart_bringup/README.md)。
`colcon test --packages-select kart_system`でcontrol_core・joy_controlsを実行する。
ROSを使わないテストはプロジェクトrootの`./scripts/test-vehicle.sh`でも実行可能。
単体テストはmacOSで確認済み。ROS上の結合ビルド・実機動作は未検証。
