# kart_vehicle

JPBB基板のUSB CDC通信、HOST arm、指令の最終鮮度確認、トリム適用、ESC指令の状態遷移を担当する。
既存firmwareの`JPB1`・CRC16-CCITT、115200 baudを使用する。

## ノードの既定名

| ROSノード名 | 実行ファイル | Component |
| --- | --- | --- |
| `/kart_bridge` | `kart_bridge_node` | `kart_vehicle::BridgeNode` |

以下はnamespaceなし・remapなしの既定名。実装は相対topic名なので、namespace=`kart`なら
`/vehicle/control_cmd`は`/kart/vehicle/control_cmd`になる。
ROS標準の`/rosout`、`/parameter_events`等は省略。QoSはすべてreliable/volatile/keep-last。

## 入力topic

| 既定topic名 | 型 | Depth | 内容 |
| --- | --- | --- | --- |
| `/vehicle/control_cmd` | `kart_interfaces/msg/ControlCommand` | 1 | muxが選択した補正前の指令 |
| `/operation_mode/state` | `kart_interfaces/msg/OperationModeState` | 1 | モードheartbeat |
| `/vehicle/trim/request` | `kart_interfaces/msg/ControlTrim` | 10 | ステア・スロットルの補正増分。stamp有効期間100 ms |

## 出力topic

| 既定topic名 | 型 | Depth | 内容・配信タイミング |
| --- | --- | --- | --- |
| `/operation_mode/request` | `kart_interfaces/msg/OperationModeRequest` | 10 | 異常時STOP、実RC経路選択時PROPOの要求 |
| `/vehicle/trim/state` | `kart_interfaces/msg/ControlTrim` | 1 | 10 Hz。累積トリム、`steering_offset`は含まない |
| `/vehicle/rc_channels` | `std_msgs/msg/Int32MultiArray` | 1 | 新しい有効statusごと。`[RX1,RX2,RX3]`、µs |
| `/vehicle/output_channels` | `std_msgs/msg/Int32MultiArray` | 1 | statusごと。`[servo,ESC]`、µs。基板生成PWMであり実測舵角／車速ではない |
| `/vehicle/vbec` | `std_msgs/msg/Float32` | 1 | statusごと。BEC電圧、V |
| `/vehicle/active_path` | `std_msgs/msg/UInt8` | 1 | statusごと。0=disabled、1=RC、2=HOST、3=failsafe |
| `/propo/control_cmd` | `kart_interfaces/msg/ControlCommand` | 1 | 健全なRC経路中の入力換算値。記録用でmuxに折り返さない |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1 | 1 Hz。USB接続・fault・鮮度・HOST phase・トリム・ESC指令モデル |

基板statusは既存firmwareで20 Hz。bridgeのread/write処理は100 Hz。
基板statusの連番は独立した基板カウンタであり、host指令のACK番号ではない。

## パラメータ

`steering_offset`だけ動的変更可能。他の独自パラメータは起動時固定。単体起動用の全parameterは[config/bridge.yaml](config/bridge.yaml)。
実運用は[kart_bringup/config/vehicle/bridge.yaml](../kart_bringup/config/vehicle/bridge.yaml)を使用する。
個別pkgのconfigはbringupへ自動で重ねない。

| 名前 | 既定値 | 説明・範囲 |
| --- | --- | --- |
| `device` | `""` | 必須。空では起動を拒否。実際の`/dev/serial/by-id/...`または選定したttyACMパス |
| `steering_scale` | `-1.0` | ROS左正→基板PWM方向への倍率。−1..1、絶対値0.01以上 |
| `steering_offset` | `0.0` | 倍率適用後のステア補正。fresh STOP中のみ動的変更可能。−0.5..0.5。D-padトリムとは別 |
| `command_timeout` | `0.15` s | 指令の受信時刻・元stampの有効期間。0.02..0.2 s |
| `status_timeout` | `0.2` s | 基板statusの有効期間。0.05..0.5 s |
| `steering_trim_limit` | `0.25` | 累積ステアトリムの絶対値上限。0..0.5 |
| `throttle_trim_limit` | `0.1` | 累積スロットルトリムの絶対値上限。0..0.5 |
| `initial_steering_trim` | `0.0` | 起動時トリム。±steering_trim_limit以内。ROS左正 |
| `initial_throttle_trim` | `0.0` | 起動時トリム。±throttle_trim_limit以内。前進正 |
| `use_sim_time`（ROS共通） | `false` | 実車bridgeではfalse必須。trueでの起動を拒否し、動作中の変更も通信解除 |

固定値: mode監視300 ms、再接続待ち1 s、arm時の初期指令待ち150 ms、handshake全体500 ms。
RC換算の端点は1000/1500/2000 µs。独自サービスはない。

## トリム・ログ

STOP/MANUALで増分要求を受け付け、HOST接続確認中・AUTO中の調整要求は無視する。
同じ／古いstampや範囲外増分も拒否する。1要求の各増分は絶対値約0.05まで、累積値は上限で制限する。

- ステアトリムはMANUAL/AUTO共通。ROS指令に加算・±1へ制限してからsteering_scaleとsteering_offsetを適用する。
- スロットルトリムはMANUALだけ。操作中の正負指令へ加算し、0を越える方向反転を防ぐ。
  入力0は常に0。専用brake入力とAUTO駆動量には加算しない。
- STOP・handshake中は固定補正もトリムも加えず、ニュートラルを送る。
- 累積値はSTOP・USB再接続を越えて維持する。ノード再起動でinitial値へ戻る。自動保存はしない。

受理時に起動端末へINFOログを出す（上限で変化しない場合も表示）。

```text
Trim updated: steering=+0.000 -> +0.001, throttle=+0.000 -> +0.000
```

## ESC指令の状態管理

[EscState](include/kart_vehicle/esc_state.hpp)をHostSessionの出力直前、トリムと整数化の後に適用する。
以下はユーザー申告のTamiya ESC動作をモデル化したもの（2026-10-07、2026-10-21に中立中の履歴を補足）。
**出力指令の履歴によるモデルであり、実測車速・実ESC状態のフィードバックではない。**
`neutral`は中立指令であり、停止確認ではない。生成した指令の個別ACKも現protocolにはない。

| 現在の指令モデル | 0入力 | ＋入力（throttle） | −入力（reverse） | 専用brake要求 |
| --- | --- | --- | --- | --- |
| unknown | neutral | 中立に抑制、unknown維持 | 中立に抑制、unknown維持 | 中立に抑制、unknown維持 |
| neutral・前進後の初回負入力待ち | neutral（待ちを維持） | forward | brake | brake |
| neutral・初回負入力待ちでない | neutral | forward | reverse | 拒否してneutral |
| forward | neutral | forward | brake | brake |
| brake | neutral | forward | brake保持 | brake保持 |
| reverse | neutral | forward（制動段階なし） | reverse | 拒否してneutral |

- HOST中立handshakeでモデルを初期化する。停止・USB異常・基板fault・RCへの制御権移行でunknownへ戻す。
  RC中の入力履歴をHOSTへ引き継がない。unknownに戻ることは車両停止の確認ではない。
- forward/brakeからの負入力は、同じ大きさの`brake`へ分類し直し`reverse=0`にする。
  現firmwareでは両者が同じPWMのため、この分類変更だけでは物理出力波形は変わらない。
- brake保持だけではreverseへ遷移しない。0入力を挟み、再び負入力するとreverseとなるモデル。
  中立の必要保持時間は実機未検証であり、現モデルは出力1回を中立として扱う。
- 初回負入力待ちでないneutral/reverseからの専用brakeは、後退を指示してしまうため中立化して警告する。
  **この場合、制動できたとは扱わない。** ステア指令は維持する。
- 判定は送信値の0..1000整数化後。firmwareの整数`milli/2`変換では0と1が中立PWMになるため、
  どちらも0へ正規化する。微小な正値でforward扱いしない（ESC固有の不感帯は未モデル化）。
  新規パラメータ・topicはない。遷移時はINFO、brake拒否時は最大3秒に1回WARNを出す。
- `/diagnostics`の`esc_command_state`にunknown/neutral/forward/brake/reverse、
  `esc_brake_rejected`に直近出力の拒否有無を載せる。連続故障のラッチではない。

**未実装:** Bluetoothレポートの鮮度監視、遅延異常時のブレーキ優先出力・保持・解除の状態機械。
現在のSTOPと指令timeoutは引き続きHOST解除・中立化であり、自動ブレーキを行わない。
これらは[停止制御の設計メモ](../../../docs/vehicle.md#esc状態管理と遅延時制動)を参照。

## 動作と起動

テストで使用する`kart_system/control_core.hpp`はヘッダーのみで実装されている。
`kart_system_INCLUDE_DIRS`を両テストの`target_include_directories`へ明示的に渡し、
ROS Componentライブラリはリンクしない。`test_bridge_core`には`Threads::Threads`をリンクする。
非推奨の`ament_target_dependencies`は使わない。

起動・異常後はSTOPの観測、新しいHOST要求、健全なdisarm済み基板、中立指令が必要。
ニュートラルHOST要求の後、新しいstatusでHOST経路を確認して操作を通す。
USB復旧や指令復帰だけでは再armしない。STOPはHOST解除と中立送信であり、物理RC経路の停止ではない。

```bash
cd /workspaces/ros2_ws
colcon build --symlink-install --packages-up-to kart_vehicle
source install/setup.bash
ros2 run kart_vehicle kart_bridge_node --ros-args -p device:=/dev/serial/by-id/実際の基板ID
```

通常は[kart_bringup](../kart_bringup/README.md)で全体を起動する。
macOSでprotocol・arm制御・擬似端末の単体テスト、実firmwareパーサーとの45指令の照合を確認済み。
ESCモデルの全20遷移、負入力保持、専用brake拒否、制御権喪失、整数化境界を単体テスト済み。
再実行: `scripts/tests/test-vehicle.sh /path/to/kart_bridge_board`（project rootから、実機アクセスなし）。
ROS結合ビルド、実USB再接続、CH3切替・ESC動作は未検証。


portableなbridge_core testに、AUTO固定スロットルの0.3秒中立からHOST handshakeへの
組合せ確認を追加した。HOST未成立の非中立入力は引き続き拒否する。
bridge実装・parameter・topicは変更しない。このテストはROS/基板/実車動作の確認ではない。

正常なHOST稼働中のAUTO↔MANUALはAuthorityを保持して直接切替する。
この切替ではHostSession/ESC履歴を再初期化せず、操舵と通常手動操作への引継ぎを許可する。
異常・通信途絶・再接続後のSTOP再arm要件は維持する。


## UIでのステア補正

Map Studioの実車調整 →「ステア補正」で`kart_bridge.steering_offset`を取得・変更する。
UIは新しいSTOPとVSLAMによる1秒以上の停車確認を要求する。
bridge自身も元stampと受信時刻が300 ms以内のSTOPを要求し、有限値−0.5..0.5だけを受理する。
ROSパラメータのatomic commit後、次の10 ms周期で反映する。STOPの中立出力は変わらない。
再起動でYAML値に戻り、自動保存しない。調整後は既存操作系で走行を再開する。

加算・±1制限の方式は維持する。D-padの`steering_trim`は倍率適用前のROS左正、
`steering_offset`は倍率適用後であり、`steering_scale=-1`では符号の意味が異なる。
`/diagnostics`は両方の現在値と`steering_scale`を1 Hzで公開する。
UIのHOST有効表示はbridgeのauthority状態であり、物理的な動作・停止確認ではない。

## ESC要求シーケンサ（任意有効化）

`EscSequencer`は`HostSession`内、補正・整数化の後、USB送信の直前で動くROS非依存クラス。
controllerは既存の`ControlCommand`（前進=`throttle`、後退=`reverse`、制動=`brake`、全ゼロ=中立）を
周期送信する。特別な意図topic、パルス指令、新ノードは不要。`EscState`は実際に選択した出力指令の履歴モデルを担当する。

`esc_sequencer_enabled=true`が既定。falseを明示した場合は時間付き信号列の生成を止め、ESC履歴規則による指令分類だけを行う。
有効時の規則は以下。いずれもESC・車体の実測状態ではない。

| 現在の指令モデル | 要求 | 出力・段階 |
| --- | --- | --- |
| forward / brake | 後退 | `reverse_braking`で負入力制動保持 → `reverse_neutral`で中立保持 → 指定量の後退 |
| neutral・前進後の初回負入力待ち | 後退 | `reverse_braking`で負入力制動保持 → `reverse_neutral`で中立保持 → 後退 |
| neutral・初回負入力待ちではない | 後退 | `reverse_neutral`で中立保持 → 指定量の後退 |
| reverse | 後退 | 指定量の後退を継続 |
| reverse | 制動 | `brake_entry`で正パルス → 中立を挟まず指定量の負入力 → `braking`で保持 |
| forward / brake / 初回負入力待ちのneutral | 制動 | 正パルスを挟まず指定量の制動 |
| 初回負入力待ちでないneutral / unknown | 制動 | `rejected`。中立へ抑制し、未知の方向から正パルスを生成しない |
| 既知状態 | 前進 | 指定量の前進。後退からも直接切替 |
| 任意 | 中立（ステアだけの入力を含む） | 保留中の段階を即時中断して中立 |

- 同じ制動要求の再送や制動量変更で正パルスをやり直さない。正パルス中の前進要求は即時採用、
  後退要求は負入力制動保持→中立保持へ切替。中立後でも前進後の初回負入力待ちなら負入力制動を受け付け、履歴がなければ拒否する。
- 要求の鮮度・排他性・範囲チェックは`HostSession`が毎周期行う。入力不正、timeout、STOP、
  制御権喪失、USB切断ではHOSTを解除し、遷移履歴を破棄する。復旧で自動再開しない。
- シーケンサ有効時、送信周期の逆行・非有限時刻・過大な間隔は`failed`となりHOSTを解除する。
  各段階はsteady clockで進め、sleepせず、10 msのbridge周期で出力を選ぶ。
- 時間の満了は信号列の進行だけを意味する。中立保持は停車確認ではなく、`braking`も制動成功・停車完了ではない。
  復帰moduleが画像/IMU等で停車を判定し、中立へ切り替える責務は維持する。
- 正パルス量はMANUALのスロットルトリムを加算しない。ステアは各周期の要求を通常どおり補正して使用する。
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`, reliable/volatile/depth 1, 1 Hz) に
  `esc_sequence_phase`（idle/reverse_braking/reverse_neutral/brake_entry/braking/rejected/failed）と
  `esc_sequence_reason`を追加。失敗でbridgeがSTOP要求した後はidleへリセットし、理由は`last_interlock`に残る。
  既存の入力・出力topic名、型、ノード名に変更はない。

全て起動時固定。パルス量と時間は実機未校正の仮値。

| 名前 | 既定値 | 意味・許容範囲 |
| --- | --- | --- |
| `esc_sequencer_enabled` | `true` | 要求を信号列へ変換するシーケンサを有効化。falseは従来動作 |
| `esc_reverse_brake_command` | `1.0` | 前進/制動モデルから後退へ移る際の負入力制動量。0.002..1 |
| `esc_reverse_brake_s` | `0.2` s | 後退前の制動要求保持。0.001..2 |
| `esc_reverse_neutral_s` | `0.01` s | 後退要求時の中立保持。0.001..2 |
| `esc_brake_entry_throttle` | `0.08` | 後退から制動へ移る正パルス量。0.002..1 |
| `esc_brake_entry_s` | `0.05` s | 正パルスの保持時間。0.001..0.5 |
| `esc_max_step_gap_s` | `0.1` s | シーケンサ出力周期の許容間隔。0.001..0.2 |

実運用はbringupの`config/vehicle/bridge.yaml`で設定する。
実際の保持時間は送信周期分の量子化を受け、USB送信やESC内部受信の個別ACKはない。
実ESCが正パルスを認識する時間、後続の負入力による後退中の制動成立、停止距離は実機確認が必要。
復帰moduleは`reversing → post_reverse_braking → resume_neutral`で、
後退から中立を挟まず`brake`を要求する。上位の別controllerでも同じ要求を利用できる。
Joy managerのMANUAL時○ボタンも既存の`brake`要求経路をそのまま利用する。

検証: `bash scripts/tests/test-vehicle.sh`。正パルス/中立保持の時間、要求再送、量変更、
中立・前進・後退への切替、量子化、時計異常、freshness/STOP/基板制御権喪失による中断を
ROSなしのC++テストで確認する。ROS component結合と実ESCの制動性能は未確認。

前進中にL2（reverse要求）を押すと、シーケンサ有効時は後退前に負入力制動を保持する。
後退要求の再送・量変更で保持時間をリセットしない。制動中の中立・前進・専用brake要求は
即時採用し、中立/前進で後退への保留遷移を中断、専用brakeでは後退へ進まず制動を継続する。
ブレーキ量と時間は仮値であり、時間満了は停車確認ではない。
R2を離してneutralになっても、前進後の初回負入力待ちを保持する。
中立時間の長さでこの履歴を消さず、R2→中立→L2でも制動保持→中立保持→後退へ進む。


後退前のvehicle中立保持の既定値は0.01秒（bridge 100 Hzの約1周期）。
USBやESCが中立を認識する時間を保証する値ではなく、必要に応じて実機で調整する。
「負入力制動0.2秒」は、前進履歴のあるESCに正規化値−1.0相当
（`esc_reverse_brake_command=1.0`）のブレーキ信号を0.2秒要求し続ける意味。
後退信号と同じPWM方向だが、前進からの負入力なので制動として扱う指令モデル。
停止を検出して0.2秒待つ、あるいは0.2秒で必ず停止するという意味ではない。
復帰module自身の`reverse_neutral_s`（後退開始前の停車確認側設定）は別設定で変更しない。


### 前進後の初回負入力の履歴

ユーザー申告の実ESC規則（2026-10-21）を採用する。正入力を送ると「初回負入力は制動」の
履歴を立て、中立だけでは何秒経っても消さない。初回負入力はbrakeとして扱い、
負入力を保持している間もbrakeを維持する。その後中立を挟んで再び負入力を送るとreverseとなる。
この履歴はEscStateで管理し、出力モデルneutralと区別する。シーケンサ無効時の指令分類にも同じ規則を使う。

`/diagnostics`に`esc_negative_action`（brake/reverse/unknown）を追加する。
これは「次の負入力の指令モデル」であり、実ESC内部状態や車体の進行方向の測定ではない。
STOP、HOST喪失、再接続等のresetで履歴を破棄してunknownへ戻す。
中立handshake自体が実ESC内部の前進履歴を消したことを保証するわけではない。

portableテストは前進→0.01/0.03/0.37/10秒の中立→後退要求を再現し、
いずれも最初に制動を出すことと、制動→中立→後退が一巡して無限に制動を繰り返さないことを確認する。
