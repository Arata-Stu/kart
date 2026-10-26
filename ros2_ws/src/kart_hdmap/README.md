# kart_hdmap

Map Studioの複数lane・左右境界・Centerline/Raceline/Customlineを読み込み、ROS 2 topicへ配信する。
既定ノード名・実行ファイルは`hdmap_server`。Python単独ノードで、geometry読込と可視化を担当する。
オンラインplanner・障害物回避・走行指令は実装しない。

## 入力ファイル

`kart.hdmap.v2`の編集正本`map.json`、または書き出した`hd_map.yaml`＋相対参照CSVに対応。
Jetsonへ転送したbundleは`export/hd_map.yaml`を指定できる。古いv1はStudioで保存してv2にする。
点群未取得のpending地図、frame不一致、重複lane ID、不正数値、CSVとラインの不整合を拒否する。
ファイルは起動時とreload要求時だけ読み込む。編集中の自動追従はしない。

## 起動

```bash
cd ros2_ws
colcon build --symlink-install --packages-up-to kart_hdmap kart_bringup
source install/setup.bash
ros2 launch kart_bringup hdmap.launch.py \
  map_file:=/absolute/path/to/map/course/map.json lane_id:=lane_001
```

単独起動は`ros2 run kart_hdmap hdmap_server --ros-args --params-file <package-config> -p map_file:=<file>`。
実運用はbringupの`config/hdmap/hdmap.yaml`。車両ノードやVSLAMを起動しない。

## TF・座標の契約

- ファイルの`frame`／`frame_id`をそのまま全メッセージへ設定する。既定は`map`。
  parameterは期待するframeの検証用で、座標を別frameとして貼り替える機能ではない。
- XYはメートル、元座標を変えず、2D HDMapのZは0。ラベルだけ表示高さを付ける。
  点群の高さや路面勾配を復元した3D走行ラインではない。
- TFの購読・配信・map→odom適用は行わない。HDMap作成時に既にmapへ揃えた座標を再変換しない。
- FoxgloveのDisplay frameを`map`にすると、HDMap単独はTFなしで表示できる。
  車両・odom点群との重ね合わせには、同じ保存地図へlocalizeした推定系の`map→odom→base_link`が必要。
  表示のために架空のidentity map→odomを配信しない。別のVSLAM地図は同じframe名でも同じ座標系とは限らない。
- stampは配信時のROS時刻。bag再生と重ねるときは`use_sim_time:=true`とbagの`/clock`を使う。
  生成時刻のstampをTF検索に流用しない。静的地図の座標自体は時刻で変化しない。

## Topic・service

すべて相対topic名。表はnamespaceなしでの既定名。namespace/remapで分離可能。
出力はreliable / transient_local / depth 1、起動・選択変更・reload時および既定1 Hz。
定期再送によりvolatile購読でも次の周期で受信できる。`/rosout`・parameter等ROS標準IFは省略。

| 方向 | 既定topic | 型 | 内容 |
|---|---|---|---|
| 出力 | `/hdmap/markers` | `visualization_msgs/msg/MarkerArray` | 全laneの境界・生成ライン・IDラベル。青=左、橙=右、緑=Center、紫=Race、黄=Custom。選択laneは太線 |
| 出力 | `/hdmap/centerline` | `nav_msgs/msg/Path` | 選択laneのCenterline |
| 出力 | `/hdmap/raceline` | `nav_msgs/msg/Path` | 選択laneのRaceline |
| 出力 | `/hdmap/customline` | `nav_msgs/msg/Path` | 選択laneのCustomline |
| 出力 | `/hdmap/document` | `std_msgs/msg/String` | 検証済み正規化JSON。全lane・frame・revision・offline_reference・速度profile。観測用でplanner向けの型付きAPIではない |
| 出力 | `/hdmap/selected_lane` | `std_msgs/msg/String` | 現在選択中のID。空なら選択なし |
| 入力 | `/hdmap/select_lane` | `std_msgs/msg/String` | lane ID選択要求。reliable / volatile / depth 1。空で解除、不明IDは拒否し現在の選択を維持 |

`Path`は形状と姿勢のみで速度を含まない。Race/Customの速度はdocumentのprofileに保持。
profile列は`[s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2]`。
閉路Pathは表示用に始点を末尾へ付加する。documentの点列・profileには付加しない。
未選択／未生成のPathは空配信して以前のラインを消す。markersは専用topicで毎回DELETEALL＋全データを送る。

| service | 型 | 用途 |
|---|---|---|
| `/hdmap/reload` | `std_srvs/srv/Trigger` | 同じmap_fileを再読込。不正ファイル／選択ID消失ならfailure、直前の有効地図を維持 |

```bash
ros2 topic pub --once /hdmap/select_lane std_msgs/msg/String '{data: shortcut}'
ros2 service call /hdmap/reload std_srvs/srv/Trigger '{}'
```

選択は可視化・参照データの選択のみで、走行モードや車両制御を変更しない。

## パラメータとlaunch

line_type・centerline_speed_mps・speed_override_mpsは動的変更可能。他の独自parameterは起動時固定（read_only）。全値をpackage configとbringup configに記載。

| 名前 | 既定値 | 意味 |
|---|---|---|
| map_file | `""` | 必須。正本またはexportへのパス |
| frame_id | `map` | 入力frameとの一致検査。TF変換先ではない |
| lane_id | `""` | 起動時の選択ID。空なら全lane表示のみ。複数laneから勝手に一つ選ばない |
| publish_rate_hz | 1.0 | 再送頻度、0より大きく30以下 |
| line_width_m | 0.04 | 境界／ライン表示幅、正値。選択laneは1.5倍 |
| label_height_m | 0.25 | ラベルの表示高さと文字サイズ、正値 |
| use_sim_time | false | ROS共通parameter。trueならclockに合わせる |

launch引数`map_file`・`lane_id`・`use_sim_time`は既定空文字。明示したものだけbringup YAMLを上書き。

## Foxglove

既存のFoxglove Bridgeへ接続して3D panelを追加。Display frame=`map`、`/hdmap/markers`を有効にする。
選択ライン単独の表示には各Path topicを有効にする。Raw Messagesの`/hdmap/document`でlane IDと速度profileを確認できる。
Bridge自体の導入・起動・外部公開はこのlaunchに含めない。
対応形式: https://docs.foxglove.dev/docs/visualization/panels/3d

## 検証

```bash
PYTHONPATH=ros2_ws/src/kart_hdmap python3 -m unittest discover -s ros2_ws/src/kart_hdmap/test -v
# ROS環境:
colcon test --packages-select kart_hdmap
colcon test-result --verbose
```

portableテストは座標保持・閉路・ID選択・frame拒否・不正profile・CSV参照を検査。
ROSメッセージテストはROS未導入ならskip。実ROSでの通信、Foxglove表示、VSLAMとのTF整合、実車は別途検証が必要。

## 走行不可能領域

正本／exportの`obstacles: [{id, polygon}]`を同じmap座標で読み込み、
`/hdmap/markers`へ赤い閉じた境界、`/hdmap/document`へ頂点とIDを配信する。
既存topic型とパラメータの変更はない。地図にない場合は空として扱う。
このノードは可視化・配信のみで、車両停止指令やオンライン衝突判定は行わない。
Studioが保存時に形状を検証し、生成・export時に禁止領域との接触を検査する。

## 速度付きオフライン参照ライン

選択lane内の`line_type`を`/planning/reference_line`（`kart_interfaces/msg/ReferenceLine`）へ配信する。
QoSはreliable/transient_local/depth1、既存のpublish_rate_hzと同周期。
静的地図の再配信timerはsteady clockを使い、headerはROS時刻とする。
遅いsimでも地図heartbeatの実時間間隔が伸びず、制御側の受信watchdogと整合する。
再配信は静的参照の状態通知であり、推定姿勢や制御指令のstampは更新しない。
閉路でも先頭を末尾に重複追加しない。XY/速度の要素数は同じ。
追加parameterは`line_type=centerline`（動的）と`centerline_speed_mps=-1.0`（動的）（-1は地図保存値、0以上は明示速度）。
Race/Customはprofileのvx_mpsを使用し、Centerlineは明示した一定速度を使う。

| 速度試験parameter | 既定値 | 動的 | 意味 |
|---|---|---|---|
| allow_speed_override | false | 不可 | 試験速度指定の許可。sim運用設定のみtrue |
| speed_override_mps | -1.0 | 可 | -1は通常速度、0.05..1.5 m/sは正の参照速度を一定値へ置換 |
| nominal_speed_limit_mps | 0.0 | 不可 | 通常速度の上限、0は追加制限なし。sim運用では0.45 |

`speed_override_mps`はparameter serviceから適用後すぐReferenceLineを再配信する。
停止点（速度0）、座標、地図ファイルは変更しない。空のlineに点を追加しない。
allow_speed_override=falseでは-1以外を拒否するため、実機設定の既定動作は変わらない。
試験速度はnominal上限を置換するが、下流controllerの独立した速度上限は有効。
速度変更は車両モードを変更しない。追従・速度制御の設定整合は呼び出し側の責務。
`/hdmap/select_line`（std_msgs/msg/String、reliable/volatile/depth1）で
centerline/raceline/customlineを選択できる。不明kindは拒否。
未生成のkindやlane未選択なら空のReferenceLineで以前の選択を無効化する。
これはオフライン参照であり、障害物回避済みのオンライン軌道ではない。
走行中の選択変更の連続性は保証しない。


### 起動時・実行中のline選択

`hdmap.launch.py`と`tracking.launch.py`の`line_type`引数で初期選択する。
未指定/空ならYAMLの既定centerlineを保持し、centerline/raceline/customline以外は拒否する。

```bash
ros2 launch kart_bringup tracking.launch.py \
  map_file:=/data/maps/course/map.json lane_id:=main line_type:=raceline
ros2 param set /hdmap_server line_type customline
ros2 param get /hdmap_server line_type
# 既存topicでも同じparameterを変更できる
ros2 topic pub --once /hdmap/select_line std_msgs/msg/String '{data: centerline}'
```

変更が確定すると選択ラインを即時再配信する。不正parameter変更は現在の選択を保持する。
選択laneに未生成のkindを指定した場合は、空ReferenceLineで以前の参照を無効化する。
`/hdmap/selected_line`（std_msgs/msg/String、reliable/transient_local/depth1）で現在の種別を配信する。
既存select_line topic、ROS parameter、selected_line出力は同じ状態を表す。
実行中変更はYAMLへ自動保存せず、再起動時はlaunch/YAMLの選択に戻る。
変更後のラインへ即時切替するため、走行中の軌道接続や操舵の連続性は保証しない。

## Notebookからの動的調整

`hdmap_server`に任意の調整窓口を追加した。通常は無効。
新しい実行ファイルは追加せず、既存`hdmap_server`（Python単体ノード）の中で扱う。
表示・編集・ライン生成はNotebookのMap Studio、JetsonはROS状態の取得・snapshot検証と
既存`/planning/reference_line`等の配信を担当する。TFの発行や車両モード要求は行わない。

| 起動時固定parameter | 既定値 | 意味 |
| --- | --- | --- |
| tuning_enabled | false | loopback TCP 8782の調整窓口を有効化 |
| tuning_map_dir | 空文字 | 自己位置推定に使用する転送bundleの絶対パス。map_fileはこの配下に限定 |
| tuning_tracking_frame | rear_axle | mapから取得する車両のTF frame |

有効時のみ追加される入力（namespace/remapは通常の相対topic規則に従う）:

| 方向 | 既定topic | 型 | QoS・用途 |
| --- | --- | --- | --- |
| 入力 | /operation_mode/state | kart_interfaces/msg/OperationModeState | reliable/volatile/depth1、走行モードとstamp/受信期限0.3秒 |
| 入力 | /visual_slam/tracking/odometry | nav_msgs/msg/Odometry | sensor data QoS、速度normとstamp/受信期限0.5秒 |
| 入力 | /localization/vslam/diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth10、表示用の位置確定状態、1.5秒期限 |
| 入力 | /tf、/tf_static | tf2_msgs/msg/TFMessage | tf2_ros標準QoS、map→tuning_tracking_frameの取得、動的TFは0.5秒期限 |

出力topic・既存serviceは上記の表と同じ。追加ROS service/actionなし。
HTTP `POST /status`・`/apply`・`/rollback`は127.0.0.1:8782固定、Originなし・JSONのみ、
3 MiB＋request envelope上限。Notebookから既存SSH接続設定で呼ぶ。Dockerはhost networkingが必要。
状態監視は10 Hz、HTTP取得は約1 Hz。状態変更はHTTPスレッドで行わずexecutorで一括処理する。

ライン適用・rollbackは最新AUTO/MANUALと有限の速度情報があれば走行中も可能。
STOPの場合は速度推定の絶対値0.05 m/s未満が1秒以上継続する従来条件を使う。参照ライン配信元が
このノードだけであること、地図資産hashと適用元revisionの一致が必要。
速度推定はVSLAM由来であり、物理的な停車の保証ではない。
有効なStudio v2 geometryを最大20,000生成点で受け、選択lane・lineと速度を一括交換する。
Center速度はsnapshotの値、Race/Customは生成済みprofileを使う。uniform speed overrideは適用時に-1へ戻す。
Center速度はROS parameterとHTTP active.centerline_speed_mpsの両方へ反映する。
旧版へ戻す際はgeometry・lane・line・Center速度・uniform overrideをまとめて戻す。
元ファイルは上書きせずメモリ上に適用するため、再起動時は起動地図へ戻る。
画面切断によるSTOP要求は出さず、既存Joy/モード管理・watchdogに従う。

検証: `test_tuning.py`で停車継続・鮮度・NaN・地図不一致・hash・形状不整合を検証。
実ROS環境の結合・Jetson負荷・実車は別途確認が必要。


実車調整が有効なときは追加で`/control/tracking_reference`
（`kart_interfaces/msg/TrackingReference`、reliable/volatile/depth1）を購読する。
有効かつmap座標・有限値で、計測stampと受信から0.5秒以内のものだけ
HTTP statusの`tracking_reference`として返す。origin/targetは制御計測時の座標、
lookahead_mは経路長、target_distance_mは直線距離。無効通知・timeout・地図適用時には消去する。
Pure Pursuitの速度連動時、ここで表示するlookahead_mは実効先読み距離となる。
「パラメータ調整」では`lookahead_mode`を「固定距離 / 速度に合わせる」から選ぶ。
固定時は`lookahead_m`、速度連動時は1 m/s用の`lookahead_low_speed_m`と
5 m/s用の`lookahead_high_speed_m`だけを表示する。初期値はfixed、固定0.8 m、低速0.8 m、高速2.0 m。
速度範囲内を線形補間し、範囲外は端の距離を維持する。従来の係数/上限指定は置き換えた。
既存のrevision・モード/速度鮮度確認・atomic set/readbackを使用し、新しい独自topic/serviceは追加しない。
この観測追加による新しいparameter、publisher、serviceはない。


## Jetsonパラメータ調整窓口

`tuning_enabled=true`時の同じloopback HTTP窓口へ`POST /parameters_get`・`/parameters_set`を追加。
map_id、target（tracking / speed / map / recovery / vehicle）、変更時はexpected_revisionとchangesを受ける。
許可した5ノードと項目だけに限定し、クライアントから任意ノード名・service名は受け付けない。
対象はHDMapノードと同じnamespace内のpure_pursuit、speed_controller、hdmap_server、recovery、kart_bridge。
現在値とdescriptorのread_only・型・説明を取得して返す。取得は閲覧用でSTOP不要、変更は
追従・速度・参照ラインは走行中も変更可能。車両補正・復帰は最新STOP・1秒停車を要求する。
全変更で地図一致・参照ライン非競合・モード受信0.3秒・速度受信0.5秒の鮮度条件を維持する。

既存ROS標準serviceのクライアント:

| 方向 | 既定service名（nodeを上記5ノードで展開） | 型 | 用途 |
| --- | --- | --- | --- |
| 要求 | /node/get_parameters | rcl_interfaces/srv/GetParameters | 許可項目の現在値取得 |
| 要求 | /node/describe_parameters | rcl_interfaces/srv/DescribeParameters | 実ノードの型・read_only属性取得 |
| 要求 | /node/set_parameters_atomically | rcl_interfaces/srv/SetParametersAtomically | 1ノード内の変更を一括適用 |

ROS executorで同期待機／spinを行わず、futureを次timer周期で回収する。
get/describe→変更前revision確認→対象別の適用条件再確認→atomic set→get/describeで適用後の実値を確認する。
同時に1トランザクションのみ。複数ノードを跨ぐ一括commitはしない。外部ROSクライアントとの
compare-and-setを保証するAPIではなく、事前読取時の競合と適用後の値を確認する方式。
通信timeout後に変更を再送しない。適用済みか不明な場合は現在値を再取得する。
メモリ上だけの変更で、YAMLを更新しない。新規ROS parameterなし。

実車ライン切替は生成済みの選択laneにあるline_typeに限定する。地図snapshot全体の再転送は不要。
Center速度は有限・非負のdouble。tuning有効時のline_type／速度変更はネイティブparameter serviceでも
AUTO/MANUAL中は走行中に切替可能。STOP中は従来の停車条件を要求する。snapshotの一括適用は検証済みの新地図を同一transactionで選択する。


### 車両状態とステア補正の連携

`tuning_enabled=true`時、追加で`/recovery/status` (`std_msgs/msg/String`, JSON schema `kart.recovery.v1`)
と`/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`, name=`kart_bridge`, hardware_id=`JPBB-01`)
を購読する。追加パラメータ・出力topicはない。HTTP statusの`vehicle`に限定した状態だけを返す。
復帰状態はstamp/受信時刻とも0.8秒以内、bridge診断は2.5秒以内だけ有効。
未起動・受信待ち/無効・受信停止・復帰topicの配信元競合を区別する。
共有diagnostics topicの他ノードは無視する。復帰のsuspectは衝突の確定判定ではない。
診断levelは整数とROSの1 byte表現の両方を0〜3の整数として受理する。
不正なlevelでは過去のbridge状態を破棄して受信待ちに戻し、参照ライン配信を継続する。
`test/test_vehicle_status.py`でbyte表現・不正値・次の正常受信への復帰をROS非依存で検証する。
車両パラメータ対象`vehicle`は`kart_bridge`の`steering_offset`を変更可能、
`steering_scale`・`initial_steering_trim`・`steering_trim_limit`は参照専用。
通常の新しいSTOP・1秒以上の低速確認を適用し、bridge自身のSTOPチェックも必要。

復帰状態の許可リストに`post_reverse_braking`（後退後の制動・停車確認）を含む。topicとJSON schema名は既存のまま。

復帰状態post_reverse_pulseを「後退後の制動準備（正パルス）」として表示する。復帰のpost_reverse_pulse_throttle / post_reverse_pulse_sもSTOP時の動的調整対象。

走行中チューニングの状態応答は`apply_issue`（ライン・追従・速度用）と
`stop_issue`（車両補正・復帰用）を返す。版競合・timeout拒否・atomic適用・readbackは維持する。
PROPO・不明mode・古い速度では走行中変更を許可しない。portable testでAUTO中の適用・rollbackを確認し、ROS/実車は未確認。

## 地図に保存するCenter速度

Studio v2の各laneとexportの各laneに`centerline_speed_mps`を保存する。既定1.5 m/s、
0〜20 m/sの有限値。旧ファイルの項目欠落時も1.5 m/sを使用する。
ROS parameter `centerline_speed_mps`の既定は-1.0（選択laneの保存値を使用）。
0以上の明示値は保存値より優先し、0は停止目標になる。lane切替・reload時は保存値を再評価する。
Race/Customの点ごとの速度profileと停止点は変更しない。
実車調整のsnapshotには解決済みのCenter速度を出し、既存のrevision・モード確認を維持する。
`test_speed.py`と`test_document.py`は保存値・旧形式・明示上書き・0・不正値を検証する。
ROSメッセージ配信・実車での速度追従は未検証。
