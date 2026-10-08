# kart_joy

Linux evdevを読むC++ Composable Nodeと、共通YAMLを編集するGUI/CUI。
最初の対応プロファイルはPS5 DualSense (`054c:0ce6`)。
`ament_cmake_auto`を使用し、`kart_joy::JoyNode`と単体実行ファイル`kart_joy_node`を提供する。
これは入力ノードであり、車両制御コマンドやAUTO/MANUAL切替は発行しない。

## ノード・パラメータの既定値

既定ノード名は`/kart_joy_node`、実行ファイルは`kart_joy_node`、componentは`kart_joy::JoyNode`。
以下はnamespaceなし・remapなしの名前。topicはnamespace相対で、namespace=`kart`なら`/joy`は`/kart/joy`になる。
ノード固有の入力topicはない。Linuxの`/dev/input/event*`を直接読み取る。
ROS標準の`/rosout`、`/parameter_events`等は本READMEの表から省略する。

| ROSパラメータ | ノードの既定値 | 説明・範囲 |
| --- | --- | --- |
| `profile_path` | 空文字 | 配置・デバイス識別YAML。空／無効では中立で待機。launchでは下記のprofileパスを渡す |
| `input_dir` | `/dev/input` | evdevデバイスを探索するディレクトリ |
| `publish_rate_hz` | `100.0` | Joy/raw/connected/readyの配信周期。1..1000 Hz |
| `scan_period_ms` | `1000` | 未接続時の探索間隔。100 ms以上 |

すべて起動時固定。単体用の全parameterは[config/joy.yaml](config/joy.yaml)。共通のuse_sim_timeはfalse。
`joy.launch.py`の引数は`config`（package shareのconfig/joy.yaml）、`profile`（空ならconfig値を維持）、
`namespace`（空）、`composed`（true）。profileの暗黙の環境変数展開は行わない。
実運用は[kart_bringup/config/input/joy.yaml](../kart_bringup/config/input/joy.yaml)を使用する。
composed起動時のcontainer既定名は`/joy_container`。
プロファイル内の軸・ボタン配置はROSパラメータではなく、GUI/CUIとreloadサービスで変更する。

## ビルド・起動

Dockerfileにlibevdev・yaml-cpp・amentの開発依存を追加済み。
イメージ再ビルド後、コンテナ内のシステムPython/ROS環境で実行する。

```bash
cd /workspaces/ros2_ws
colcon build --symlink-install --packages-select kart_joy
source install/setup.bash
colcon test --packages-select kart_joy
colcon test-result --verbose

# 初回のみ。既存ファイルがある場合は上書きしない。
ros2 run kart_joy kart_joy_config init --profile /workspaces/config/joy.yaml

# デフォルトはComposable Nodeコンテナ。コントローラ未接続でも起動可能。
ros2 launch kart_joy joy.launch.py profile:=/workspaces/config/joy.yaml
# 単体実行へ切り替える場合: composed:=false
```

直接実行する場合:

```bash
ros2 run kart_joy kart_joy_node --ros-args \
  -p profile_path:=/workspaces/config/joy.yaml \
  -p publish_rate_hz:=100.0 -p scan_period_ms:=1000
```

ノード起動パラメータは読み取り専用。プロファイルだけをサービスで再読込する。
単体用configのprofile_pathは空なので、設定するか`profile:=...`を明示する。
bringupの運用configでは`/workspaces/config/joy.yaml`を指定している。ファイルがない・壊れている場合もノードは中立出力で待機し、
有効なプロファイルを読み込むまで機器を選択しない。GUIからの初回保存も可能。

## GUI

ノードと同じROS環境をsourceした別ターミナルで:

```bash
ros2 run kart_joy kart_joy_config gui
```

同じホストのブラウザで `http://127.0.0.1:8766/` を開く。
Dockerでは既存のhost networkを使う。別PCから閲覧する場合は、そのPCで
`ssh -N -L 8766:127.0.0.1:8766 USER@JETSON` を実行してから同じURLを開く。
HTTPサーバーはlocalhostだけで待ち受ける。コントローラはブラウザPCではなくJetsonに接続する。

1. 「編集開始」で設定モードへ移行する。
2. 軸・ボタンの「検出」を押し、対象の入力だけを操作する。
3. 軸は静止位置と端を保持して「取得」。スティックはbipolar、トリガーはunipolar。
4. 必要に応じてdeadzoneを調整する。反転は負端と正端を交換する。
5. 「保存して反映」でYAMLへ保存し、ノードへ再読込を要求する。
6. 全操作を離し、再接続表示を確認して「出力再開」。

コード`-1`は割り当て無効。GUIでは生入力と接続状態も表示する。
設定値は`/workspaces/config/joy.yaml`に保存され、プロジェクトルートのbind mountで永続化される。
GUIとCUIは同時に編集せず、一つだけ起動して使う。

## CUI

```bash
ros2 run kart_joy kart_joy_config cui
```

- `axis`：出力名を選び、操作で軸を検出。静止位置・正端・負端・deadzoneを設定。
- `button`：出力名を選び、ボタンまたは十字キーの軸を操作して割り当て。
- `device`：接続中のデバイスのVendor/Product/名前/固有IDを設定へ取り込む。
- `show`：編集中プロファイルと現在の入力を表示。
- `save`：GUIと同じYAMLを検証・保存・ノードへ反映。
- `resume`：全操作を離して出力再開し、エディタを終了。
- `quit` / Ctrl-C：出力を自動再開せず終了。

`axis`/`button`選択後に`disable`で割り当てを無効化できる。
YAMLを直接編集した場合の構文・値の確認:

```bash
ros2 run kart_joy kart_joy_config check --profile /workspaces/config/joy.yaml
ros2 service call /kart_joy_node/configure std_srvs/srv/SetBool '{data: true}'
ros2 service call /kart_joy_node/reload_profile std_srvs/srv/Trigger '{}'
# 接続が戻り全入力を離してから:
ros2 service call /kart_joy_node/configure std_srvs/srv/SetBool '{data: false}'
```

`namespace:=teleop`で起動した場合、ツールにも`--namespace /teleop`を付ける。
設定ツールは`rclpy`を使うため、uvの推論・raceline仮想環境ではなくROSのシステムPythonで実行する。
追加のPyPI依存はない。

## 入出力と動作

以下はすべて出力topic。QoSはvolatile/keep-last、raw以外はreliable/depth 1、rawはbest effort/depth 1。

| 出力topic既定名 | 型 | 内容 |
| --- | --- | --- |
| `/joy` | `sensor_msgs/msg/Joy` | 正規化・割り当て後。既定100 Hz、reliable、depth 1 |
| `/joy/raw` | `sensor_msgs/msg/Joy` | ABS/KEYコードを配列indexにした生入力。best effort、depth 1 |
| `/joy/connected` | `std_msgs/msg/Bool` | 接続・入力同期・必要な入力コードの利用可否。設定モード中もtrueになり得る |
| `/joy/ready` | `std_msgs/msg/Bool` | 接続・同期・profileが有効かつ設定中でない。100 Hz。teleopの許可判定に使用 |
| `/joy/status` | `std_msgs/msg/String` | デバイス名・固有ID・使用可能コード・設定状態を含むYAML。1 Hzと状態変更時 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 接続待機・設定モード・異常状態 |

`joy/raw`のaxesは64要素、buttonsは768要素。軸はカーネルのmin/maxから[-1,1]へ正規化する。
生入力のトリガー静止値は-1。通常の`joy_node`の可変長配列とは異なるため、
生入力を既存teleopへ直接渡さない。`header.frame_id`の世代番号で機器の切替を検出する。

`joy.axes`は固定8要素:

```text
0 left_x, 1 left_y, 2 right_x, 3 right_y, 4 l2, 5 r2, 6 dpad_x, 7 dpad_y
```

スティック・十字キーは[-1,1]（初期設定では右/下が正）、トリガーは[0,1]（静止0）。
`joy.buttons`は固定17要素:

```text
0 cross, 1 circle, 2 triangle, 3 square, 4 l1, 5 r1, 6 l2, 7 r2,
8 create, 9 options, 10 ps, 11 l3, 12 r3,
13 dpad_up, 14 dpad_down, 15 dpad_left, 16 dpad_right
```

既定のVendor/Product一致に加えてゲームパッドのABS_X/ABS_Y/BTN_GAMEPAD能力で選別し、
DualSenseのタッチパッド・IMUを除外する。`js0`や`eventN`は指定不要。
同じ条件に一致する機器が複数ある場合は任意の一台を選ばず待機する。
`device.unique`へ機体固有IDを設定して区別する。必要なら
`udevadm info --attribute-walk --name=/dev/input/eventN`等で識別情報を調べる。

未接続時は既定1秒間隔で再探索し、接続時に現在値を取得する。readエラーで切断処理に入り、
押下中の値を保持せず全出力を中立にする。libevdevの同期モードで`SYN_DROPPED`から復帰し、
同期中も出力は中立。設定中は生入力の配信を続け、割り当て後の出力のみ中立にする。
無効なプロファイルの再読込は既存の有効な設定を維持する。保存は一時ファイル＋rename。
設定モードはノードのメモリ上の状態で、ノードを再起動すると解除される。

`joy/connected`だけで車両の走行許可を判断しない。`kart_system`の`kart_joy_manager`は
`joy/ready`とJoyの鮮度を確認する。既定ではL1保持は不要。切断・設定開始・入力途絶ではMANUALをSTOPへ戻す。
復旧後は中立で△を押し、再度MANUALを要求する。このノードは無操作と、ドライバが切断を通知しない
通信停止を区別できない。自動再接続時の走行再許可も車両側で決める。

## サービス・GUI/CUIの入出力

| サービス既定名 | 型 | 動作 |
| --- | --- | --- |
| `/kart_joy_node/configure` | `std_srvs/srv/SetBool` | trueで設定モード。falseで全操作を離したこと等を確認して出力再開 |
| `/kart_joy_node/reload_profile` | `std_srvs/srv/Trigger` | 設定モード中にprofileを再読込 |

サービスはprivate名`~/configure`・`~/reload_profile`なので、ノード名やnamespaceを変えると追従する。
GUI/CUI時のツールの既定ノード名は`/kart_joy_config`。入力は`/joy/raw`（sensor_msgs/msg/Joy、
SensorDataQoS）と`/joy/status`（std_msgs/msg/String、reliable/depth 1）。出力topicはなく、上記サービスを呼ぶ。
サービス宛先はstatus内のノード名から取得する。init/checkモードはROSノードを起動しない。
CLIオプションは`--namespace`（既定`/`）、`--port`（GUI、既定8766）、`--profile`（init/checkでは必須、既定値なし）。

## Dockerの入力アクセス

kart CLIの既存設定は`/dev/input`ディレクトリをマウントする。
固定`/dev/input/eventN`一個だけのマウントにすると、抜き差し後に作成されるデバイスを追えない。
既存のDocker設定を利用し、新たな`/dev`全体のマウントは追加していない。
USB/ Bluetoothの接続・ペアリングはJetsonホスト側で行う。
コンテナの実行ユーザーに対象デバイスの読み取り権限が必要。
`ls -ln /dev/input/event*` と `id`で確認し、必要なinputグループの数値GIDをDockerの
`--group-add`で渡す。権限不足はノードのstatusに表示する。

## 検証

macOSでC++の変換処理テスト、Pythonのプロファイル/検出テスト、Python/XML/JS構文を検証。
GUIはモック入力で表示・編集・保存・再読込を確認。
通常のC++テストは成功したが、ASan/UBSan付きの実行はこのMacで20秒タイムアウトした。
原因は未特定で、サニタイザー検証済みとは扱わない。
ROS 2/Lyricalのcolconビルド、実際のサービス通信、evdevの切断・再接続と
DualSenseのUSB/Bluetooth両経路はJetson上での確認が必要。

実機では「未接続起動→接続→ボタン保持→切断→全出力0→再接続」の順で確認する。
続いてGUI/CUIで軸・ボタンを再割り当てし、保存後とノード再起動後で同じ`joy`出力になること、
L2/R2静止時が0であること、入力権限不足時や複数台接続時に任意の機器を選ばないことを確認する。

参考: [Linux hid-playstation](https://github.com/torvalds/linux/blob/master/drivers/hid/hid-playstation.c)、
[libevdev event handling](https://github.com/whot/libevdev/blob/master/libevdev/libevdev.h)、
[ROS 2 component registration](https://github.com/ros2/rclcpp/blob/rolling/rclcpp_components/cmake/rclcpp_components_register_node.cmake)。
