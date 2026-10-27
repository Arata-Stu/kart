# kart_parking

レース終了後、既存のraceline/centerlineを順方向にたどり、指定区画へ前進で駐車するplanner。
通常走行は元のReferenceLineを透過配信し、`parking/start`で駐車経路へ切り替える。
**実寸・壁・区画・goalのmap座標が必要。サンプルは架空地図であり実コース用ではない。**

## 経路と停止の仕様

- 閉じた参照ラインの点順を走行方向とする。分岐・逆向きラインの自動推定はしない。
- 駐車区画近くの出発点から、接線付き曲線を試し、通らなければ前進のみの状態格子探索を行う。
  旋回上限は`tan(max_steering_rad)/wheelbase_m`。探索は時間・展開数に上限があり、
  解なしは「物理的に不可能」の証明ではない。最短路・探索完全性は保証しない。
- 探索は指定した`maneuver`内に限定する。`forward_yaw`方向の進行を非負に制約し、
  前進ギアのUターンによる逆走も拒否する。区画へ直角に入る進行は許可する。
  曲がった進入区間では単一方向制約が保守的になり、通れる経路も拒否する場合がある。
- 入口を過ぎた／進入距離が不足した場合は**順方向にもう1周**する。切り返し・後退は生成しない。
  自動後退するrecoveryや固定AUTO throttle、Map Studioの走行中編集とは併用できない。
- 既存ラインにも、進入経路にもrear_axle基準の前後非対称長方形と余裕を適用する。
  点間移動・姿勢変化の余裕を加えた掃引チェックを行う。区画中心だけを衝突判定しない。
  既存ラインの走行可能性は元ライン生成・追従調整を前提とし、ここで周回ラインを作り直さない。
- 参照速度は元ラインを保持し、進入部の低速・終端0へ向けて後方に減速制約を伝播する。
  全経路の最近点を毎回選ばず、単調な進捗と前方windowで次周の重複位置への飛び越しを防ぐ。
  入口手前1mから`line_type=parking`とし、Pure Pursuitの駐車用先読み上限を使う。
- 区画内の車体全体・goal位置・yawが許容内に入り、推定速度と角速度が小さい状態を
  `settle_s`継続して初めて`parked`にする。到達時点から空ReferenceLineとSTOP要求を保持する。
  到達後の位置ずれ・動き出しは`failed`となり、自動的に走り直さない。
- 目標外の停止・壁接触による停滞を駐車成功と扱わない。進捗なしは`stall_s`で失敗停止。
  入力欠落・再局在不良・TF不良・経路変更・時計巻戻し・逸脱は失敗をラッチする。
  `reset`は新鮮なSTOPでのみ許可する。AUTOや録画開始はplannerから要求しない。

STOP要求・中立出力は物理停止やESC制動の証明ではない。停止確認はVSLAM推定に基づく。
実際の停止距離、低速追従、壁余裕、外形、舵角、自己位置誤差の検証が必要。
このplannerは静的地図を使う。未登録障害物や動く物体は検知しない。

## 地図ファイル

標準はMap Studioの「駐車 P1・P2・P3」で編集し、HDMap内の`parking_areas`へ保存する。
`polygon`（区画）、`goal_pose`（後輪軸x/y/yaw）、`entrance`（開口2点）、`maneuver`（進入可能範囲）、
`forward_yaw`（コース順方向）を区画ごとに持つ。走行領域はleft/right boundの間と選択区画のmaneuver・polygonの和集合から生成し、
壁は共通の`obstacles`。入口を塞ぐ壁・区画の別の辺からの進入を拒否する。

従来の独立site JSONも互換入力として使用できる。以下の表はその形式。
`config/site.example.json`は架空座標であり、実コースの値へ置き換える。
HDMap/VSLAMと同じ座標系のm・radを使用する。画像ピクセルや地面補正後の別座標を混在させない。

| JSON項目 | 意味 |
|---|---|
| `schema` | `kart.parking.v1` |
| `frame_id` | `map_frame`と一致する座標系 |
| `drivable` | コースと駐車区画を含む走行可能領域の外周、単純多角形。外側は走行不可 |
| `obstacles` | 内壁・中島・禁止領域の多角形配列。細い壁にも実際の幅を持たせる |
| `slots.P1/P2/P3` | 実行時に選ぶ区画。別IDも可 |
| `goal` | `[x,y,yaw]`、車体中心ではなくrear_axleの目標姿勢 |
| `area` | 駐車判定の区画。余裕を含む車体全体が入る必要がある |
| `maneuver` | 車線から区画へ離脱できる局所領域。車体全体と余裕が通れる形にする |
| `forward_yaw` | 入口付近のコース順方向。例: map +Xなら0、+Yならπ/2 |

多角形は終点を重複させず、3〜200頂点。境界への接触・自己交差・非有限値は拒否する。
壁が駐車入口を塞ぐように描かれていると経路は出ない。境界の開口を現地と合わせる。

## ノードとインターフェース

package `kart_parking`、実行ファイル／既定node名 `parking_planner`。
Python独立プロセス。探索workerを追従C++ containerから分離し、ROS executorで探索を待たない。
すべて相対topic名。以下はroot namespace時の名前。ROS標準rosout/parameter_eventsは省略。

| 方向 | 既定topic | 型 | QoS・内容 |
|---|---|---|---|
| 入 | `/planning/course_reference` | `kart_interfaces/msg/ReferenceLine` | reliable/transient_local/1、HDMapの閉路・点順・速度 |
| 入 | `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | sensor data QoS、速度・pose測定時刻 |
| 入 | `/localization/vslam/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable/volatile/10、既存地図局在とVO正常 |
| 入 | `/operation_mode/state` | `kart_interfaces/msg/OperationModeState` | reliable/volatile/1 |
| 入 | `/parking/goal` | `geometry_msgs/msg/PoseStamped` | reliable/volatile/1、選択区画内のgoal変更。STOP・mission未開始時のみ |
| 入 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | tf2標準。odom stampのmap→rear_axleとrear_axle→odom childが必要 |
| 出 | `/planning/reference_line` | `kart_interfaces/msg/ReferenceLine` | reliable/transient_local/1、20Hz。plannerが唯一のpublisher |
| 出 | `/parking/path` | `nav_msgs/msg/Path` | reliable/transient_local/1、開始時に全計画、reset時に空。可視化用 |
| 出 | `/parking/status` | `std_msgs/msg/String` | reliable/volatile/1、20Hz JSON: state/slot/goal/reason/progress_index/parked |
| 出 | `/operation_mode/request` | `kart_interfaces/msg/OperationModeRequest` | reliable/volatile/10、到達・失敗中STOPのみ |

速度判定はodom child原点の並進速度ノルム＋角速度ノルム×rear_axleまでの距離を使用する。
後輪軸速度の保守的な上界であり、停止判定用。車体前後速度・タイヤ速度の計測値ではない。

| サービス（提供） | 型 | 内容 |
|---|---|---|
| `/parking/start` | `std_srvs/srv/Trigger` | ready・新鮮なSTOP/AUTO・局在・コース整列を確認して駐車開始。再要求は拒否 |
| `/parking/reset` | `std_srvs/srv/Trigger` | 新鮮なSTOPでラッチ解除・通常ラインへ戻す |

状態: `waiting_for_course` → `preparing` → `ready` → `approach` → `parking` → `settling` → `parked`。
準備失敗は`state/reason`に理由、走行中の異常は`failed`。ready以前のstartは拒否する。

## 起動と操作

Jetsonの既存containerでビルド・sourceする。ホストMacではROS結合は検証できない。

```bash
bash scripts/build.sh --packages-up-to kart_bringup
source ros2_ws/install/setup.bash
bash scripts/bringup.sh --mode drive \
  --map-file /workspaces/maps/course/hd_map.yaml \
  --map-dir /workspaces/maps/course --model-dir /workspaces/models/vgl \
  --lane-id main --line-type raceline \
  --parking-slot P2 \
  --recovery off --no-tuning
```

map/modelのパス・lane IDは実際の構成に合わせる。設定正本は
`kart_bringup/config/parking/parking.yaml`。parking-slotだけを指定すれば、選択した`--map-file`内の駐車定義を使う。
parking-slot/siteをどちらも省略すると従来構成。
`tracking.launch.py`／`mission.launch.py`からは`parking_site:=... parking_slot:=P2`でも指定できる。

1. `/parking/status`が`ready`であることを確認。`/parking/path`は開始時に全経路が出る。
2. 通常どおりレース走行。plannerは周回ラインを透過する。
3. レース終了後、駐車開始を明示する（AUTO要求は送信しない）。

```bash
ros2 topic echo /parking/status
ros2 service call /parking/start std_srvs/srv/Trigger '{}'
```

4. `parked`を確認する。`failed`は成功ではない。Joy AUTO保持を含む既存の制御許可は維持される。
5. 次の試行はSTOPにしてからresetする。ラッチはAUTO再押下だけでは解除されない。

```bash
ros2 service call /parking/reset std_srvs/srv/Trigger '{}'
```

任意goalを置く場合はSTOP/reset後に`/parking/goal`へmap座標PoseStampedを送る。
位置・orientationはrear_axleの目標、z=0、単位quaternion。選んだP1〜P3のarea外は拒否。
区画自体の変更はSTOP後に`--parking-slot`を変えて再起動する。
レース終了の自動検出・周回数判定は追加していない。
Map Studioでの編集手順は`tools/app/README.md`の駐車区画節を参照。

## 走行前のオフライン計画

ROS通信や実車出力なしで、保存済みHDMapと開始姿勢から計画を出せる。

```bash
ros2 run kart_parking parking_plan \
  --map-file /workspaces/maps/course/hd_map.yaml --lane-id main --line-type raceline \
  --site /workspaces/maps/course/hd_map.yaml --slot P2 \
  --config /workspaces/ros2_ws/src/kart_bringup/config/parking/parking.yaml \
  --start 0.0 0.0 0.0 --output /tmp/parking-plan.json
```

開始姿勢も実際の値に置き換える。出力は新規JSONのみ、既存ファイルは上書きしない。
Python環境にPyYAMLがあれば`python -m kart_parking.offline`でも実行できる。
結果は静的幾何検査の計画であり実車追従の確認ではない。

## パラメータ

すべて起動時固定。package `config/parking.yaml`は単体起動用、実運用はbringup側。
寸法と舵角は`docs/policies/vehicle_geometry.md`／tracking.mdの暫定値を使用する。
動的操作はgoal topicとstart/reset。launchの明示指定のみYAMLを上書きする。

| 名前 | 既定値 | 意味 |
|---|---|---|
| `site_file` | `空文字` | 必須HDMap（map.json/hd_map.yaml）または互換site JSONパス |
| `slot` | `P1` | 区画ID |
| `map_frame` | `map` | 地図座標系 |
| `tracking_frame` | `rear_axle` | 追従基準TF |
| `input_timeout_s` | `0.3` | odom/modeのROS stampと受信時計の期限 s |
| `line_timeout_s` | `2.5` | 参照ライン鮮度 s |
| `localization_timeout_s` | `2.5` | VSLAM診断鮮度 s |
| `publish_rate_hz` | `20.0` | 配信・監視周期 Hz、steady clock |
| `wheelbase_m` | `0.257` | ホイールベース m |
| `max_steering_rad` | `0.3839724354` | 等価二輪最大舵角 rad、追従側と一致 |
| `front_m` | `0.2435` | rear_axle→前端 m |
| `rear_m` | `0.1065` | rear_axle→後端 m |
| `width_m` | `0.19` | 車幅 m |
| `margin_m` | `0.05` | 壁・区画境界との余裕 m |
| `sample_m` | `0.025` | 曲線・閉路のサンプル上限 m（最大0.05） |
| `step_m` | `0.15` | 前進探索primitive長 m |
| `xy_resolution_m` | `0.075` | 探索XY bin m |
| `yaw_resolution_rad` | `0.12` | 探索yaw bin rad |
| `max_expansions` | `12000` | 各探索の最大展開数、正整数 |
| `search_timeout_s` | `8.0` | 候補探索の時間上限 s（閉路検査時間を除く） |
| `departure_radius_m` | `4.0` | goalから探索する離脱候補の距離上限 m |
| `min_approach_m` | `0.5` | 離脱までの最小順方向距離 m、足りなければ次周 |
| `parking_speed_mps` | `0.25` | 進入部の速度上限 m/s |
| `deceleration_mps2` | `0.4` | 速度profileの減速度 m/s²、実測保証ではない |
| `max_cross_track_m` | `0.15` | 起動整列・実行中の許容横偏差 m |
| `goal_position_m` | `0.08` | goal位置許容 m |
| `goal_yaw_rad` | `0.15` | goal yaw許容 rad |
| `stopped_speed_mps` | `0.03` | 停止とみなす推定速度上界 m/s |
| `stopped_yaw_rate_rps` | `0.05` | 停止とみなす角速度ノルム rad/s |
| `settle_s` | `1.0` | 停止成立の継続時間 s |
| `stall_s` | `5.0` | AUTO中4cm以上の進捗がない場合の失敗時間 s |
| `horizon_m` | `4.0` | 配信する前方経路長 m（最小1） |
| `use_sim_time` | `false` | ROS共通。受信監視はsteady clockのまま |

正の数値を要求する。時計巻戻しで走行missionを無効化し、時計停止中も受信期限を監視する。

## 検証範囲

`test/test_parking.py`: 前進探索、壁・掃引、次周、停止確認、異常ラッチ。
`test/test_tracking.py`: 生成した進入経路と、入口通過後の再周回からの駐車を実装済みC++ Pure Pursuit＋理想二輪運動学で追従。
`test/test_adapter.py`: ROSなしで実callbackを呼び、同一形状のheartbeat時刻更新と経路変更時の無効化を検査。
ROS結合、Jetson実行負荷、実地図、実機、ESC、タイヤの滑り・制動・自己位置誤差は未確認。

```bash
PYTHONPATH=ros2_ws/src/kart_parking:ros2_ws/src/kart_hdmap python3 -m pytest ros2_ws/src/kart_parking/test -q
```

駐車の領域結合はShapely（ROS/Docker: `python3-shapely`、ローカルStudio: 起動Python環境へ`pip install shapely`）を使用する。
閉路boundは包含する内外周、開路boundは両端を結んだ帯として検証する。接続していない進入範囲・交差boundは拒否する。
和集合の継ぎ目は壁にせず、残った中島・外周・共通obstaclesに対して車体と余裕を検査する。
生成結果を別の手描き属性として保存せず、保存したboundとparking属性から再構成する。ROS topic・parameterの追加はない。
