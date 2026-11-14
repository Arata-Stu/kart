# kart_lap_manager

閉じたHDMap参照ラインと健全なVSLAM姿勢から周回イベントを生成し、周回ごとのVGL再要求と
Jetsonキャッシュ解放サービスを独立に呼び出す。package / executable / 既定ノード名は
`kart_lap_manager` / `lap_manager` / `lap_manager`。独立Pythonプロセスで動作する。
サービス応答待ちや監視をセンサーcontainerから分離するため、Composable Nodeは提供しない。
車両指令・モード要求・録画開始は発行しない。走行モードに関係なく入力を満たす周回を数える。

## 周回の定義

参照ラインの点順を正方向、`points[0]`をfinishとする。最初の健全な姿勢から計測を開始し、
正方向finish通過までに周長の90%以上の**正味**進捗と5秒以上を観測したとき1周を発行する。
finish上で起動した場合は最初の1周から発行する。任意地点から最初のfinishまでの距離が
周長の90%未満ならその部分周は数えず、そのfinishから次周を計測する。
この90%は境界付近の初期位置誤差を許容する既定値であり、厳密に100%要求する場合は
`min_lap_fraction=1.0`を指定する。finish通過前にはVGL・cacheを要求しない。
停止中のfinish付近の揺れ、逆走、ショートカット、投影先の飛び、姿勢の大きな飛びで周回を水増ししない。
閉路以外、1m未満、退化segment、非有限座標、map以外のframeは受け付けない。
交差・近接した複雑な参照線では投影が飛ぶと部分周を破棄するため、誤計数回避と引き換えに周回を欠測し得る。

odometryのstampに対応する`map←base_link`をTFで取得する。TF欠落は入力の元stampと受信時刻を
保って期限内だけ再試行し、新しい入力で待機中の入力を上書きしない。診断は
`hardware_id=visual_slam`、`level=OK`、`localized_in_exist_map=Yes`、`vo_status=OK`を要求する。
姿勢と診断のheader時刻・steady受信時刻の両方を検査する。TF、健全診断、姿勢が途絶えた場合、
時計逆行、参照線変更、大きな`map→odom`補正では部分周を破棄し、次の健全姿勢から再計測する。
同一の参照線が再配信されても進捗は維持する。累積周回数はノード終了まで保持する。

VGL要求後も健全かつ連続した姿勢から次周の進捗を計測する。自分のVGL要求がpendingであり、
dispatch時に保存したfinishの開始境界をそのまま計測中、かつfinish通過から3秒以内の場合に限り、
到着したfresh hintで次周の開始境界を保持する。これにより通常の小補正のたびに1周飛ばす動作を避ける。
短い周回では推論中に周長の15%以上進む場合もあるため、進捗割合では保持対象を制限しない。
0.15mまたは0.15radを超える`map→odom`変化、姿勢jump、不健全診断などはこの場合も部分周を破棄する。
上流VGL poseのstampは推論前の撮影時刻なので、hintの有効期間には姿勢の0.5秒期限を流用せず
`vgl_result_timeout_s`を用いる。外部要求のhint、finishから離れたhintでは部分周を破棄する。
上流APIには要求IDがなく、同時に外部からVGLをtriggerした場合のhint対応は厳密には識別できない。
運用中のVGL要求元は本ノードに揃えること。

## 要求と結果

各周1回まで、VGLとcacheへ`Trigger`を非同期で送る。サービス不在、拒否、timeout時は同じ周を
再送しない。前回の要求がpendingなら次周の要求はskipし、statusへ記録する。
VGLの`success=true`は**探索受付**のみで、再局在成功や補正精度ではない。
VGLはhint到着後に新鮮で健全なVSLAM診断を観測するとpendingを解除するが、これは
hintに対応する内部再局在の完了やグラフ修復の証明ではない。既存の制御側の品質判定は維持する。
VGL探索のtimeout/cancelは上流APIにないため、応答／hint timeoutは要求を抑止したままとし、
遅れて届く受付応答＋hint＋健全診断で解除する。判定不能な通信例外や時計逆行では再起動が必要な場合がある。

cacheは別プロセスの`jetson_cache_service`が処理し、本ノードはサービス応答を結果として記録する。
cache timeout後も結果不明の要求へ重ねて送らず、遅れた応答があれば解除する。
VGLの状態とcache要求は独立であり、片方の不在／失敗でも他方の要求を妨げない。
キャッシュ解放はcuVSLAMのグラフエラーを修復する処理ではない。

## 入出力

topic/service名は起動時parameterで変更できる。既定名は全て絶対名なのでnamespaceでは変わらない。
ROS標準`/rosout`、`/parameter_events`は省略。

| 方向 | 既定topic名 | 型 | QoS | 内容 |
|---|---|---|---|---|
| 入力 | `/planning/reference_line` | `kart_interfaces/msg/ReferenceLine` | reliable / transient local / depth1 | 静的閉路、点順が正方向 |
| 入力 | `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | best effort / volatile / depth5 | TF照会の姿勢stamp、odom frame必須 |
| 入力 | `/localization/vslam/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable / volatile / depth10 | 追跡・既存地図localizeの健全性 |
| 入力 | `/localization/pose_hint` | `geometry_msgs/msg/PoseWithCovarianceStamped` | reliable / volatile / depth10 | VGL hint観測、再局在結果とは別 |
| 入力 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | TF listener標準 | 元odometry stampのmap/base/odom TF |
| 出力 | `/localization/lap_events` | `std_msgs/msg/String` | reliable / volatile / depth10 | 周回時JSON、`kart.lap_event.v1` |
| 出力 | `/localization/lap_status` | `std_msgs/msg/String` | reliable / volatile / depth1 | 既定1Hz、`kart.lap_status.v1` |

| 方向 | 既定service名 | 型 | 条件 |
|---|---|---|---|
| client | `/visual_localization/trigger_localization` | `std_srvs/srv/Trigger` | `vgl_enabled=true`のみ |
| client | `/kart/cache/clear` | `std_srvs/srv/Trigger` | `cache_clear_enabled=true`のみ |

JSONは`lap`、`course_length_m`、進捗、要求状態を含む。イベントには`stamp_s`と`lap_time_s`も含む。
statusの`reason`は部分周の未計測理由、`armed`は健全姿勢から進捗を計測中かを示す。
各要求の`state`、`requests`、`completed`、`failures`、`skipped`、`message`を公開する。
`completed`はcache応答成功またはVGL hint後の健全診断観測であり、実車補正精度を表さない。

## 全parameter

全て起動時固定。基準値は`config/lap_manager.yaml`、実運用の正本は
`kart_bringup/config/localization/lap_manager.yaml`。動作中の変更は受け付けない。

| 名前 | 既定値 | 意味・範囲 |
|---|---|---|
| use_sim_time | false | ROS標準時計。watchdogと要求期限はsteady clock |
| vgl_enabled | false | 各周のVGL要求を有効化 |
| cache_clear_enabled | false | 各周のcache要求を有効化 |
| reference_topic | `/planning/reference_line` | 閉路入力 |
| odometry_topic | `/visual_slam/tracking/odometry` | 姿勢stamp入力 |
| diagnostics_topic | `/localization/vslam/diagnostics` | VSLAM品質入力 |
| pose_hint_topic | `/localization/pose_hint` | VGL hint入力 |
| lap_events_topic | `/localization/lap_events` | イベントJSON出力 |
| lap_status_topic | `/localization/lap_status` | status JSON出力 |
| vgl_trigger_service | `/visual_localization/trigger_localization` | VGL client |
| cache_clear_service | `/kart/cache/clear` | cache client |
| map_frame / odom_frame / base_frame | map / odom / base_link | TF/入力frame、空文字不可 |
| pose_timeout_s | 0.5 | 姿勢stamp・受信鮮度、秒・正有限 |
| diagnostics_timeout_s | 1.0 | 診断stamp・受信鮮度、秒・正有限 |
| map_correction_translation_m | 0.15 | 連続sample間のmap→odom位置変化で部分周を破棄、m・正有限 |
| map_correction_yaw_rad | 0.15 | 同yaw変化閾値、rad・正有限 |
| service_timeout_s | 3.0 | VGL探索受付応答期限、秒・正有限 |
| vgl_result_timeout_s | 30.0 | VGL dispatchからのhint/健全診断待ちとhint撮影時刻age上限、秒・正有限 |
| cache_result_timeout_s | 15.0 | cache dispatchから実処理完了のTrigger応答期限、秒・正有限 |
| status_period_s | 1.0 | status出力間隔、秒・正有限。内部watchdogは固定0.1秒 |
| own_hint_grace_s | 3.0 | 自分のVGL hintによる次周境界保持のfinish後上限、秒・正有限 |
| min_lap_time_s | 5.0 | 1周の最小観測時間、秒・正有限 |
| min_lap_fraction | 0.9 | finish間の必要な正味進捗／周長、0.5..1.0 |
| max_lateral_distance_m | 1.0 | 参照線からの最大許容投影距離、m・正有限 |
| max_pose_speed_mps | 12.0 | 位置差jump判定の速度上限、m/s・正有限 |
| max_progress_speed_mps | 12.0 | arc進捗jump判定の速度上限、m/s・正有限 |
| jump_tolerance_m | 0.15 | jump許容=`上限速度×dt+本値`、m・正有限 |
| reverse_tolerance_m | 0.2 | 後退による進捗減少の許容、m・正有限。前進で相殺 |
| max_sample_gap_s | 0.5 | 連続した姿勢の最大stamp間隔、秒・正有限 |
| finish_zone_fraction | 0.15 | finish直前／直後zone／周長、0より大きく0.25未満 |

## 起動・検証

通常はbringup TUIの地図走行で2機能を個別にON/OFFする。単体起動はROS環境で:

```bash
colcon build --symlink-install --packages-select kart_lap_manager
source install/setup.bash
ros2 run kart_lap_manager lap_manager --ros-args --params-file \
  src/kart_lap_manager/config/lap_manager.yaml -p vgl_enabled:=true
```

単体起動には別途ReferenceLine、stereo VSLAM/VGL、固定取付TF、動的TFが必要。
cacheを有効化するときはJetson cache helperも別途起動する。

ROSなしのportable検証:

```bash
PYTHONPATH=ros2_ws/src/kart_lap_manager python3 -m unittest discover \
  -s ros2_ws/src/kart_lap_manager/test -v
```

単体検証は任意地点起動、複数周、finish振動、逆走・飛び・欠落、参照線更新、
サービス重複防止／timeout抑止、撮影stampが0.8秒古いVGL hint後の毎周要求、
8秒周回・VGL hint1.5秒遅延での3周連続要求、hint前の品質喪失での開始境界差替え、
TF補正差分、品質喪失、遅延TF、時計逆行を含む。ROS adapterはmockを使う。
起動回帰では、Lyricalの`Node.clients`が読取専用であるAPIも再現したmockで、
実際のconstructorとVGL/cacheの全ON/OFF組合せを確認する。
保守サービスの参照は`maintenance_clients`へ保持し、ROS executorが使う`Node.clients`を上書きしない。
ROS依存が導入された環境では`test_ros_startup.py`が独立contextで実際のノードを生成・破棄する。
このテストはspinせず、VGL要求・cache解放・車両指令を送らない。ROSがない環境ではskipする。
ROS colcon build・DDS/TF結合・Jetson負荷・実車周回判定・再局在精度は未確認。
