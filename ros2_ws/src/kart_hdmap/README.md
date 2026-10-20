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

line_typeのみ動的変更可能。他の独自parameterは起動時固定（read_only）。全値をpackage configとbringup configに記載。

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
追加parameterは`line_type=centerline`（動的）と`centerline_speed_mps=0.0`（起動時固定）（有限・非負）。
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
| 入力 | /operation_mode/state | kart_interfaces/msg/OperationModeState | reliable/volatile/depth1、STOP状態とstamp/受信期限0.3秒 |
| 入力 | /visual_slam/tracking/odometry | nav_msgs/msg/Odometry | sensor data QoS、速度normとstamp/受信期限0.5秒 |
| 入力 | /localization/vslam/diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth10、表示用の位置確定状態、1.5秒期限 |
| 入力 | /tf、/tf_static | tf2_msgs/msg/TFMessage | tf2_ros標準QoS、map→tuning_tracking_frameの取得、動的TFは0.5秒期限 |

出力topic・既存serviceは上記の表と同じ。追加ROS service/actionなし。
HTTP `POST /status`・`/apply`・`/rollback`は127.0.0.1:8782固定、Originなし・JSONのみ、
3 MiB＋request envelope上限。Notebookから既存SSH接続設定で呼ぶ。Dockerはhost networkingが必要。
状態監視は10 Hz、HTTP取得は約1 Hz。状態変更はHTTPスレッドで行わずexecutorで一括処理する。

適用には最新STOP、速度推定の絶対値0.05 m/s未満が1秒以上継続、参照ライン配信元が
このノードだけであること、地図資産hashと適用元revisionの一致が必要。
速度推定はVSLAM由来であり、物理的な停車の保証ではない。
有効なStudio v2 geometryを最大20,000生成点で受け、選択lane・lineと速度を一括交換する。
Center速度はsnapshotの値、Race/Customは生成済みprofileを使う。uniform speed overrideは適用時に-1へ戻す。
Center速度の起動parameterは保持し、適用値はHTTP active.centerline_speed_mpsで確認する。
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
この観測追加による新しいparameter、publisher、serviceはない。
