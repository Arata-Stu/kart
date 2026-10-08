# Vehicle基盤

JetPilotの`ControlCommand`・`OperationModeRequest`・`OperationModeState`のフィールドと
モード番号を`kart_interfaces`へ移植した。`kart_system`の`kart_joy_manager`がJoy操作の解釈を担当し、同パッケージがモード管理・mux、
`kart_vehicle`がJPBBのUSB通信を担当する。自律走行アルゴリズムは今回の範囲に含まない。

```text
kart_joy -> joy + joy/ready -> kart_system::JoyManagerNode -> teleop/control_cmd --+
自律制御ノード -------------------------------------> auto/control_cmd ----+-> CommandMux
                                                                           |
                                                                   vehicle/control_cmd
                                                                           |
                                                               kart_vehicle::BridgeNode
                                                                           |
                                                                   USB JPB1 -> STM32
プロポ -----------------------------------------------------------> STM32 RC入力
```

Joy関連は次の2ノードで構成する。

| ノード | 役割 |
| --- | --- |
| `kart_joy_node`（package: `kart_joy`） | evdev接続管理・入力取得・正規化・ボタン配置変換。車両の操作仕様は扱わない |
| `kart_joy_manager`（package: `kart_system`） | 指令変換・モード要求・トリム要求。Joyの鮮度・接続・操作許可を一括管理 |

manager内では`TeleopInput`が連続入力から指令を生成し、`JoyButtons`がボタンの押下edgeを検出する。
独立したteleopノードは起動しない。`teleop/control_cmd`というtopic名は手動指令として維持する。
単体実行は`ros2 run kart_system kart_joy_manager_node`、componentは`kart_system::JoyManagerNode`。
以前の`kart_teleop`向けカスタムYAMLは対象ノード名を`kart_joy_manager`へ変更する。

モード管理・joy manager・mux・bridgeは`ament_cmake_auto`のC++17 component。
`kart_bringup`は同一container・intra-process communication有効を既定とし、
`composed:=false`で単体ノードにも分離できる。指令の保存・再配信などにはコピーが残るため、
すべての経路のzero-copyを保証する構成ではない。

## モードと制御権

| ROSモード | 値 | 指令・権限 |
| --- | --- | --- |
| AUTO | 1 | `auto/control_cmd`を選択しSTM32のHOST経路を要求 |
| MANUAL | 2 | Joy由来の`teleop/control_cmd`を選択しHOST経路を要求 |
| STOP | 3 | HOSTをdisarmしニュートラルを送信 |
| PROPO | 4 | STM32がプロポを直接選択。JetsonはRC指令を記録用に配信 |

**ROSのSTOPはHOST経路を解除する操作であり、物理RC経路を停止させるものではない。**
CH3でRCを選択するとプロポの操作が優先され、ROSのモードもPROPOへ更新される。
CH3をHOSTへ戻しても自動再armしない。受信機なしでの明示的HOST armも既存firmware仕様に従う。
`propo/control_cmd`は観測用で、muxからSTM32へ折り返さない。

`ControlCommand`はsteering=-1..1（左が正）、throttle/brake/reverse=0..1の正規化値。
速度[m/s]や加速度ではない。3つの駆動入力の同時正値、NaN、範囲外を拒否する。
全ゼロは中立であり、ブレーキではない。ブレーキと後退は基板側では同じESCパルス方向になるため、
ESCの実際のブレーキ・後退動作は設定と履歴に依存する。

### ESC状態管理と遅延時制動

bridgeにunknown/neutral/forward/brake/reverseの出力指令モデルを実装した。
遷移表・専用brakeの拒否条件・診断項目は[kart_vehicle README](../ros2_ws/src/kart_vehicle/README.md#esc指令の状態管理)を参照。
ユーザー申告（2026-10-07）では、前進から負入力へ切り替えるとブレーキになり、
そのまま保持しても後退しない。一方、後退から正入力への切替にはブレーキ段階がない。
これは指令履歴のモデルであり、実機検証済みの遷移表や停止判定ではない。

遅延時制動について合意した方向性と、実装前に残る事項:

- 接続有無だけでなく入力情報の鮮度を判定し、異常時は通常指令を遮断する。
  現Joyの再配信stampではBluetooth区間の古さを判別できないため、入力レポート監視が別途必要。
- forward/brake状態で制動可能な場合はブレーキを保持し、中立待機へ移る。
  回復だけでは走行再開せず、明示操作を必要とする。
- 後退・中立・unknownを前進中と仮定して負のPWMを送らない。中立でも惰性走行はあり得る。
  この場合の制動方法は未解決であり、中立化を「停止成功」と報告しない。
- 通常のSTOPによって制動指令が消されないよう、走行モードと制動実行状態を分離する必要がある。
  ただし物理RCへの制御権移行やUSB断を越えてHOST制動を保証することはできない。
- 閾値・ブレーキ量・保持時間・中立保持時間は実機確認後にconfigへ定義する。
  時間経過を停止完了と同一視しない。現時点ではこれらの制動設定は未実装。

現在実装済みなのは通常指令のESC状態管理。遅延検出と自動制動への接続は未実装であり、
既存STOP・timeoutは引き続き中立化する。

## 起動・ビルド

コンテナ内で実行する。パッケージはプロジェクトroot mount内にあるので、通常のC++変更は
colcon再ビルドとノード再起動、YAML変更はノード再起動で反映できる。Isaac ROS/RealSenseの再ビルドは不要。
今回Dockerfile最終APT層に`rosidl-default-generators`と`diagnostic-msgs`を明示追加した。
既存イメージで依存が不足する場合のみDocker再ビルド、または開発中のコンテナでrosdepを使用する。

```bash
cd /workspaces/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to kart_bringup
source install/setup.bash
colcon test --packages-select kart_system kart_vehicle kart_joy
colcon test-result --verbose

# 初回のJoy profile作成・GUI/CUIはkart_joy READMEを参照
# まずUSB bridgeを起動せず、Joy・変換・モード・muxを確認
ros2 launch kart_bringup vehicle.launch.py

# 実際に選定したデバイスパスでbridgeも起動
ros2 launch kart_bringup vehicle.launch.py \
  enable_bridge:=true device:=/dev/serial/by-id/実際の基板ID
```

`device`は空が既定で、bridge有効時には必須。自動で最初のttyACMを選ばない。
コンテナ内で見えている安定パスを使う。ホストにだけby-idがある場合は対応するttyACMを明示する。
現在のDockerデバイス設定でCDCアクセス・抜き差し後の再オープンができるかはJetson実機で確認する。
`/dev`全体の追加マウントは行っていない。

起動時は常にSTOP。`namespace:=kart`にも対応し、全topic名はnamespace相対。
`config`の既定は`kart_bringup/config/bringup.yaml`。ここからinput/・system/・vehicle/・recording/の
運用YAMLを読み込む。個別pkgのconfigは基準値と単体起動用で、全体起動では読み込まない。
`profile:=...`・`device:=...`・`record_dir:=...`等を明示した場合だけ対応する値を上書きする。
起動時に採用設定と明示上書きを表示する。設定パラメータは起動時固定。
実車bridgeでは`use_sim_time=false`を必要とする。

CLIからもSTOPを要求できる（STOPだけは要求stampを省略可能）。

```bash
ros2 topic pub --once /operation_mode/request kart_interfaces/msg/OperationModeRequest \
  "{mode: 3, source: operator}"
```

AUTO/MANUALなどの要求には現在時刻の`header.stamp`を付ける。要求の有効期間は500 ms。

## DualSenseの既定操作

| 操作 | 用途 |
| --- | --- |
| スティック・トリガーを中立にして△ | STOPからMANUALを要求 |
| スティック・トリガーを中立にして□ | STOPからAUTOを要求 |
| 左スティックX | ステアリング |
| R2 / L2 | 正 / 負のスロットル。L2側はESCの状態に応じてブレーキ・後退 |
| ○ | ブレーキ。前進・後退より優先 |
| × | STOP要求。AUTOでも使用できる |
| L1 / R1 | rosbag録画開始 / 停止。走行とは独立、同時押しは録画停止優先 |
| D-pad左 / 右 | ステアトリムを左 / 右へ1段階 |
| D-pad上 / 下 | スロットルトリムを正 / 負へ1段階 |

既定は`deadman_button: -1`で保持条件なし。△を押してMANUALに入った後、△を離しても継続する。
停止は×。必要な場合だけ`deadman_button`を設定できるが、L1/R1の録画割り当てとの重複を避ける。

スロットル入力は`(R2 − L2) × speed_scale`（既定0.3）。両方を同量引くと相殺する。
ROSメッセージの`throttle`は非負を維持し、負側は`reverse`へ格納する。
○の専用ブレーキは従来どおり優先する。モード要求時は両トリガーを離す必要がある。
Joy GUI/CUIは物理ボタン→標準ボタンの割り当て、`kart_bringup/config/input/joy_manager.yaml`は標準ボタン→車両操作の割り当て。
□でAUTOを要求するには、起動済みAUTO指令源が新しい中立指令を配信し、STOPである必要がある。
MANUAL⇔AUTOの直接遷移は受け付けない。
MANUALではJoyが必須。AUTOではJoyを必須にしていないため、Joy切断だけではAUTOを停止しない。

### D-padのトリム

1回の押下で既定0.001ずつ加算する。長押しの連続加算は行わない。
STOPまたはMANUAL中に調整できる。L1は不要。AUTO中とHOST接続確認中は調整を受け付けない。

- ステアトリム: 左で正、右で負。上限は既定±0.25。MANUAL/AUTOの舵に共通で加算する。
- スロットルトリム: 上で正、下で負。上限は既定±0.1。**MANUALの操作中だけ**加算する。
  例: +0.03なら、入力+0.20→+0.23、−0.20→−0.17。両トリガーを離した入力0は必ず0。
  補正で入力と逆向きに駆動しないよう0で制限する。○ブレーキとAUTOの駆動量には加算しない。
- トリムは基板がHOSTを確認した後に適用する。STOP・接続確認中は補正なしの中立を送る。
- `vehicle/control_cmd`は補正前、`vehicle/output_channels`は補正後の基板PWMを観測する。

補正は`speed_scale`適用後に加算するため、既定ではR2最大0.3＋トリム上限0.1＝0.4になり得る。
ESCの中立PWMを変更する機能ではない。
調整要求をbridgeが受け付けるたびに、起動端末へINFOログで変更前→変更後を小数点以下3桁で表示する。
例: `Trim updated: steering=+0.000 -> +0.001, throttle=+0.000 -> +0.000`。
上限に達して変化しない場合は前後が同じ値になる。bridgeを有効にして起動する必要がある。
現在値は`ros2 topic echo /vehicle/trim/state`およびdiagnosticsでも確認できる。
値はbridgeのメモリに保持し、STOPやUSB再接続では維持するが、ノード再起動時には
`initial_steering_trim`/`initial_throttle_trim`（既定0）へ戻る。YAMLへの自動保存は行わない。
運用のステップは`kart_bringup/config/input/joy_manager.yaml`、上限・初期値は`kart_bringup/config/vehicle/bridge.yaml`で変更できる。

## rosbag録画

`kart_bag_manager`はvehicle.launch.pyから別プロセスで待機起動する。
L1で開始、R1で停止。×やJoy切断で録画は停止しない。保存先は既定`/workspaces/record`。
`/bag/status`で状態とcurrent_uriを確認できる。設定・停止完了の判定は
[kart_bag_manager README](../ros2_ws/src/kart_bag_manager/README.md)を参照。

## 鮮度と再arm

- Joyと`joy/ready`は受信後100 msまで有効。Joyの元stampも確認する。
  設定開始・切断・無効profile・入力途絶はMANUAL中のSTOP要求になる。
  任意でdeadmanを有効にした場合は、その解除でもSTOPを要求する。
- muxは元の`header.stamp`を保存し、受信時刻と元stampの両方を既定150 msで確認する。
  AUTO指令源も毎回、現在時刻のstampを設定する。古いstamp、ゼロstamp、50 msを超える未来stampは拒否する。
- mode stateは10 Hz、volatile/reliable/depth 1。muxとbridgeは300 msで監視する。
  latched HOST状態からの再起動armを避け、STOPの観測と新しいHOST遷移を必要とする。
- bridgeは指令を同じく150 ms、基板statusを200 msで監視し、100 Hzで通信する。
  watchdogはsteady clockを使う。基板statusのCRC・連番を検証し、重複や逆行は鮮度を更新しない。
- arm時は健全かつdisarm済みの基板と中立の指令を必要とする。ニュートラルHOST要求を送り、
  新しいstatusでHOST経路を確認してから操作を通す。確認中の非中立指令はarm失敗。
  指令配送の初期待機は最大150 ms、全handshakeは最大500 ms。
- 指令/モード途絶、基板fault、USB断ではローカルでもdisarmしSTOPを要求する。
  通信が復旧しただけでは復帰しない。STOP観測後に改めてMANUAL/AUTOを要求する。
  プロセスごと停止した場合はSTM32側の既存200 ms command watchdogが最後の保護を担う。

STOP中はmuxが指令を送らず、bridge自身がHOST_REQUESTなしの中立フレームを送る。
`vehicle/control_cmd`のpublisherはmux一つに限定する。ROS topicは認証機構ではないため、
別publisherからの直接指令注入を防ぐセキュリティ機構はこの実装に含まれない。

## 基板通信・観測

既存`JPB1`プロトコルとCRC16-CCITTを維持。baud=115200、排他的open、非同期read/write。
read量・行長を制限し、部分write・通信エラー時には切断して再接続する。
steeringはROS左正から、既存基板の左1000 µsへ既定`steering_scale=-1`で変換する。
`steering_offset`は通常指令のみに適用し、disarm/handshakeでは正確な中立を送る。

| Topic | 内容 |
| --- | --- |
| `operation_mode/request`, `operation_mode/state` | モード要求と10 Hz状態 |
| `vehicle/trim/request`, `vehicle/trim/state` | `ControlTrim`。増分要求と10 Hzの現在値。ROS正規化単位、steering左正／throttle前進正 |
| `vehicle/rc_channels` | RX1/2/3のPWM µs |
| `vehicle/output_channels` | servo/ESCの基板生成PWM µs。実際の舵角・車速feedbackではない |
| `vehicle/vbec` | BEC電圧 V |
| `vehicle/active_path` | 0=disabled、1=RC、2=HOST、3=failsafe |
| `propo/control_cmd` | RC経路中の入力換算値。パルス端点は既定1000/1500/2000 µs |
| `diagnostics` | USB・status・fault bits・HOST phase・指令鮮度 |

## 検証状況

2026-10-05、macOS上で以下を確認した。

```bash
./scripts/test-vehicle.sh /Users/at/project/tmp/kart_bridge_board
```

- Joy managerの連続入力、ボタンedge・長押し・再接続時の誤発火防止: pass
- モード起動・STOP経由arm・タイムアウト・復旧時の再arm禁止・入力範囲・Joy変換: pass
- 正負トリガー差分、トリム上限・中立保持・方向反転防止・AUTO適用範囲・handshake中の補正抑止: pass
- CRC・status解析・連番・HOST handshake・fault時disarm: pass
- POSIX擬似端末による分割受信・送信: pass（実USBは使用していない）
- 今回のencoderの45フレームを独立したfirmwareのCパーサーへ入力し、値とPWM極性を照合: pass

比較元はローカルJetPilotの3msgと`jetpilot_bridge_interface`のprotocol実装、
`kart_bridge_board/firmware/common`と`step8_final`。firmwareファイルは変更していない。
パーサー照合は通信形式の互換性の確認であり、firmware全体や実機の安全性検証ではない。

**未検証**: ROS 2 Lyricalでのcolconビルドとノード結合、Dockerイメージビルド、DualSense実機、
JetsonのCDC権限・抜き差し、実基板でのCH3切替・ESCブレーキ・watchdog動作。
このMacにはROSがなくDocker daemonも稼働していないため、上記のコンテナ手順で続けて確認する。
