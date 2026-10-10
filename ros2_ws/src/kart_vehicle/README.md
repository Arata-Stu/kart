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
| `/vehicle/trim/state` | `kart_interfaces/msg/ControlTrim` | 1 | 10 Hz。累積トリム、固定`steering_offset`は含まない |
| `/vehicle/rc_channels` | `std_msgs/msg/Int32MultiArray` | 1 | 新しい有効statusごと。`[RX1,RX2,RX3]`、µs |
| `/vehicle/output_channels` | `std_msgs/msg/Int32MultiArray` | 1 | statusごと。`[servo,ESC]`、µs。基板生成PWMであり実測舵角／車速ではない |
| `/vehicle/vbec` | `std_msgs/msg/Float32` | 1 | statusごと。BEC電圧、V |
| `/vehicle/active_path` | `std_msgs/msg/UInt8` | 1 | statusごと。0=disabled、1=RC、2=HOST、3=failsafe |
| `/propo/control_cmd` | `kart_interfaces/msg/ControlCommand` | 1 | 健全なRC経路中の入力換算値。記録用でmuxに折り返さない |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1 | 1 Hz。USB接続・fault・鮮度・HOST phase・トリム・ESC指令モデル |

基板statusは既存firmwareで20 Hz。bridgeのread/write処理は100 Hz。
基板statusの連番は独立した基板カウンタであり、host指令のACK番号ではない。

## パラメータ

独自パラメータはすべて起動時固定。単体起動用の全parameterは[config/bridge.yaml](config/bridge.yaml)。
実運用は[kart_bringup/config/vehicle/bridge.yaml](../kart_bringup/config/vehicle/bridge.yaml)を使用する。
個別pkgのconfigはbringupへ自動で重ねない。

| 名前 | 既定値 | 説明・範囲 |
| --- | --- | --- |
| `device` | `""` | 必須。空では起動を拒否。実際の`/dev/serial/by-id/...`または選定したttyACMパス |
| `steering_scale` | `-1.0` | ROS左正→基板PWM方向への倍率。−1..1、絶対値0.01以上 |
| `steering_offset` | `0.0` | 倍率適用後の固定ステア補正。−0.5..0.5。D-padトリムとは別 |
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
以下はユーザー申告のTamiya ESC動作をモデル化したもの（2026-10-07）。
**出力指令の履歴によるモデルであり、実測車速・実ESC状態のフィードバックではない。**
`neutral`は中立指令であり、停止確認ではない。生成した指令の個別ACKも現protocolにはない。

| 現在の指令モデル | 0入力 | ＋入力（throttle） | −入力（reverse） | 専用brake要求 |
| --- | --- | --- | --- | --- |
| unknown | neutral | 中立に抑制、unknown維持 | 中立に抑制、unknown維持 | 中立に抑制、unknown維持 |
| neutral | neutral | forward | reverse | 拒否してneutral |
| forward | neutral | forward | brake | brake |
| brake | neutral | forward | brake保持 | brake保持 |
| reverse | neutral | forward（制動段階なし） | reverse | 拒否してneutral |

- HOST中立handshakeでモデルを初期化する。停止・USB異常・基板fault・RCへの制御権移行でunknownへ戻す。
  RC中の入力履歴をHOSTへ引き継がない。unknownに戻ることは車両停止の確認ではない。
- forward/brakeからの負入力は、同じ大きさの`brake`へ分類し直し`reverse=0`にする。
  現firmwareでは両者が同じPWMのため、この分類変更だけでは物理出力波形は変わらない。
- brake保持だけではreverseへ遷移しない。0入力を挟み、再び負入力するとreverseとなるモデル。
  中立の必要保持時間は実機未検証であり、現モデルは出力1回を中立として扱う。
- neutral/reverseからの専用brakeは、後退を指示してしまうため中立化して警告する。
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
