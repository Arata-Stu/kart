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
| 出力 | `/teleop/fixed_config` | `std_msgs/msg/String` | R/1 | 10 Hz、固定スロットル設定とAUTO gateの状態（JSON） |
| 出力 | `/teleop/control_cmd` | `kart_interfaces/msg/ControlCommand` | R/1 | 有効なJoy受信時に手動指令を配信。元stampを保持 |
| 出力 | `/operation_mode/request` | `kart_interfaces/msg/OperationModeRequest` | R/10 | モード切替・入力喪失時のSTOP要求 |
| 出力 | `/bag/request` | `kart_interfaces/msg/BagRequest` | R/10 | L1 START / R1 STOP。走行モードと独立 |
| 出力 | `/vehicle/trim/request` | `kart_interfaces/msg/ControlTrim` | R/10 | ボタン押下時の補正増分 |

`TeleopInput`は連続入力、`JoyButtons`は押下edgeと保持状態、`JoyModeHold`はモード保持許可を処理する内部クラス。
接続・鮮度・deadman・操作許可は一つのJoyManagerNodeが管理する。
初回／再接続時に押されていたボタンからはedgeを生成しない。

### operation_mode_manager

| 方向 | 既定topic名 | 型 | QoS | 内容 |
| --- | --- | --- | --- | --- |
| 入力 | `/operation_mode/request` | `kart_interfaces/msg/OperationModeRequest` | R/10 | モード要求 |
| 出力 | `/operation_mode/state` | `kart_interfaces/msg/OperationModeState` | R/1 | 10 Hz＋要求受理時の状態 |

初期モードはSTOP。AUTO=1、MANUAL=2、STOP=3、PROPO=4。
起動・異常後はSTOPからHOSTへ進入する。正常なAUTO/MANUALは直接切替できる。
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
modeの元stampもROS時刻で0.3秒以内をtick時に確認する。
mode managerの10Hz配信とmuxの100Hz転送・監視は実時間のまま。
simではwall_timeout_s=5.0とし、遅い描画中に0.15秒の実時間だけで失効させず、
sim時刻での入力鮮度と実時間での通信断を分離する。元stampは更新しない。

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
| `deadman_button` | `-1` | 追加deadmanなし。0以上なら△／□に加えて保持が必要。解除はAUTO/MANUALともSTOP |
| `brake_button` | `1` | ○。MANUALでは正パルス→brake=1、トリガーより優先 |
| `stop_button` | `0` | ×。STOP要求、deadman不要 |
| `manual_button` | `2` | △。保持中のみMANUAL、離すとSTOP。STOPからは中立で要求 |
| `auto_button` | `3` | □。保持中のみAUTO、離すとSTOP。STOPからは中立で要求 |
| `steering_trim_left_button` | `15` | D-pad左、ステア補正を正へ |
| `steering_trim_right_button` | `16` | D-pad右、ステア補正を負へ |
| `throttle_trim_up_button` | `13` | D-pad上、スロットル補正を正へ |
| `throttle_trim_down_button` | `14` | D-pad下、スロットル補正を負へ |
| `steering_trim_step` | `0.001` | 1押下のステア補正増分。0.001..0.05 |
| `throttle_trim_step` | `0.001` | 1押下のスロットル補正増分。0.001..0.05 |
| `deadzone` | `0.08` | ステア入力の不感帯。0..0.5、外側を再正規化 |
| `steering_scale` | `1.0` | ステア倍率。0.01..1 |
| `speed_scale` | `0.3` | 正負スロットル倍率。0.01..1。車速[m/s]ではない |
| `brake_pulse_throttle` | `0.08` | MANUALの○押下時の正throttle要求、0.002..1。起動時固定 |
| `brake_pulse_s` | `0.01` | 正パルス保持時間[秒]、steady時刻、0.01..0.5。起動時固定 |
| `auto_fixed_throttle` | `-1.0` | AUTO時の固定正規化指令（手動ステア）。-1で通常操作、0..speed_scaleで固定。起動時固定 |

スロットルは`(R2 − L2) × speed_scale`。各トリガー0.01以下を0とし、負側は`reverse`へ格納する。
両方同量なら相殺するが、モード要求時には両トリガーを離す必要がある。
Joyとreadyの有効期間は100 ms、監視は100 Hz（固定値）。AUTO/MANUALとも切断・設定開始・入力途絶はSTOP。
△保持中だけMANUAL、□保持中だけAUTO。離すと`joy_manager:mode_button_released`でSTOPを要求し、
teleopへ中立を配信する。×もSTOP。同時保持は`joy_manager:mode_buttons_conflict`でSTOPとし、
片方だけを離しても再開しない。異常停止・再接続・起動時の押下保持では再armしない。
両ボタンを離し、新鮮なSTOPを確認後に押し直す。STOPからの進入は従来どおり全入力中立が必要。
AUTO/MANUALを直接切り替える場合は、同一Joyサンプルで元ボタンを離して次のボタンを押した場合だけ可能。
追加deadmanを設定した場合はその解除でもAUTO/MANUALともSTOP。□・D-padにL1は不要。
STOPはHOST解除・中立化であり、自動ブレーキや実停止の確認ではない。
`test_joy_controls.cpp`で保持・解除・同時押し・押下保持での再接続・異常後の押し直し・
直接切替を検証する。portable C++テストは確認済み。ROS Component結合・実車での解除動作は未確認。
L1/R1は録画操作に使用する。任意でdeadmanを設定する場合、録画ボタンとの重複を避ける。
録画処理は[kart_bag_manager](../kart_bag_manager/README.md)が担当する。Joy切断や車両STOPで録画を自動停止しない。
トリムの現在値・適用・ログの責務は[kart_vehicle](../kart_vehicle/README.md)にある。

### operation_mode_manager / command_mux

operation_mode_manager独自の設定パラメータはない（STOP起動、10 Hz固定）。

| ノード | 名前 | 既定値 | 説明 |
| --- | --- | --- | --- |
| command_mux | `command_timeout` | `0.15` s | ROS時刻での元stampの有効期間。0.02..0.2 s。wall_timeout_s=0なら受信期限も兼ねる |
| command_mux | `wall_timeout_s` | `0.0` s | 0は従来の受信期限（指令0.15、mode 0.3秒）。正値は別の実時間受信監視、最大60秒。sim設定は5.0 |

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
ROSを使わないテストはプロジェクトrootの`./scripts/tests/test-vehicle.sh`でも実行可能。
単体テストはmacOSで確認済み。ROS上の結合ビルド・実機動作は未検証。


## AUTO固定スロットル・手動ステア

`auto_fixed_throttle=-1.0`（既定）は通常動作。0..speed_scale（標準上限0.3）を
指定すると、□AUTOで固定throttle＋手動ステア、△MANUALで通常R2/L2＋手動ステアになる。
AUTO中もステアを操作でき、△でSTOPを挟まずMANUALへ直接引き継げる。
正常なMANUAL→AUTOも直接切替可能。STOPからの初回進入は全入力中立が必要で、
AUTO初回進入は0.3秒中立を配信してmux/bridgeのHOST handshakeを待つ。
直接切替時は中立待ちを挟まない。×はSTOP、固定AUTO中の○はbrake優先＋STOP、L2は中立＋STOP。
入力途絶・異常・再起動後はSTOPから再armし、モード変更だけで異常解除しない。
固定AUTOでも□保持が必要で、離すと固定出力を中断してSTOPにする。
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


### ESCへの手動要求と○による制動試験

R2/L2は通常の前進/負入力要求。L2の自動シーケンスはない。
MANUALの○はトリガーより優先し、押下からbrake_pulse_throttleをbrake_pulse_sだけ要求し、
その後は押している間brake=1.0を要求する。ステアは現在の手動入力を維持する。
正パルス中はbrake/reverse=0、制動中はthrottle/reverse=0。長押しでパルスを再開始しない。
○を離すとパルスを中断し、通常トリガー操作に戻る。短すぎる押下では制動段階に達しない。
MANUAL進入後に○を一度離した入力を観測してから、新しい押下でのみ正パルスを開始する。

パルス時間はsteady時刻で判断し、出力はJoy到着周期に量子化する（標準100 Hz）。
Joy/ready途絶、不正入力、deadman解除、×STOP、モード変更、mode鮮度喪失で中断し、
復帰後に○が押されたままでも正パルスを再開しない。入力間隔100 ms超や時計逆行でも中断する。
元Joy stampを保持し、timerから新しいstampで駆動を延命しない。
AUTOでは正パルスを生成せず、固定AUTOでの○＋STOP、L2＋STOPも従来どおり。
AUTOから△への引継ぎ時に○を押していてもパルスは始まらない。

説明されたESC規則では正入力でforward履歴を作り、次の負入力が制動となる。
ただし正パルスは実際の駆動要求であり、停止中にも一瞬前進する可能性がある。
出力はbridgeのMANUAL throttle trimを受ける。負trimで正パルスが0になる場合もあるため、
最初の確認ではthrottle trimを0にし、実際の出力量とESC応答を確認する。
時間満了やモデルのbrake表示は制動成立のフィードバックではない。実ESCで値は未校正。

実運用値はkart_bringup/config/input/joy_manager.yamlを編集して再ビルド・再起動する。
新規topic/serviceはなく、既存/teleop/control_cmd（kart_interfaces/msg/ControlCommand）へ出す。
ログのJoy brake signalは段階変化時だけ出力する（長押し中の毎周期出力はしない）。
positive_pulseでは正throttle要求値・設定時間requested_s・その後のbrake=1.0保持を表示する。
brakeへの切替時には設定時間とsteady時刻の段階経過時間elapsed_sを表示する。
idleでは直前段階・経過時間・解除/STOP/入力喪失/モード変更等の終了理由を表示する。
再接続時の押下保持などで正パルスを省略した場合も明記する。
これらは補正前の指令生成とcallback間の経過時間であり、PWM実測値・ESC受信時間・物理制動の証明ではない。
/vehicle/control_cmdはbridge補正前の要求なので、trim反映後のPWM値そのものではない。

車輪を浮かせてMANUALへ入り、後退を確認してから○を押し続けると、同じ信号列を試験できる。
ROSなしのscripts/tests/test-vehicle.shで一度だけの生成、中断、再接続時の抑制、および
HostSessionへ接続した全ての既知ESC履歴からの正パルス→brake=1.0を検証する。
ROS結合・実PWM・実制動は未確認。

○の正パルスは既定0.01秒。Joy・mux・bridgeの配信周期で量子化され、基板の20 ms PWM更新で捕捉されない可能性がある。実ESCで要確認。復帰moduleの正パルス時間は0.05秒のまま。
