# kart_recovery

RGB/片側IR＋IMUでスタック疑いを検出し、過去の操舵中央値を固定して一定時間後退する。
地図・VSLAM・odom・TFは使わない。e2e/rule共通のAUTO指令後段として動作する。

node名`recovery`、実行ファイル`recovery`。画像処理とmux/bridgeのexecutorを分離するため
独立Python processとし、Component pluginは提供しない。ament_cmake_auto＋ament_cmake_pythonを使用する。

## 指令経路と動作

```text
rule speed_controller / RGB・EVS E2E decoder
  → /recovery/input_cmd → recovery → /auto/control_cmd → 既存mux → bridge
```

通常は元の指令stampを保存して通過させる。AUTO以外は中立。起動後は新鮮なSTOPが必要。
スタック条件は「前進要求が継続＋十分な特徴点がある画像の低フロー＋低角速度」。
IMUの回転が大きいときは停止と判定しない。回転補正付きフローや物理速度推定は未実装。
特徴点不足、画像/IMU途絶はスタックではなく中立＋STOP要求になる。

`driving → suspect → braking → reverse_neutral → reversing → resume_neutral → driving`

- 通常AUTOで選択した前進指令の操舵を履歴化。同じ元stampの再配信は追加しない。
- 疑い開始をtとし、`[t-offset-span, t-offset]`の操舵中央値×scaleをlimitでclip。
  sample不足・間隔超過は停止。符号を反転しない。保存値は実舵角ではなく補正前の要求。
- brake要求を出し、低フロー/低角速度の継続を確認して中立を挟む。
  ESCの履歴・brake/reverse変換はbridgeの既存モデルへ集約する。
- 固定操舵・固定reverse要求をreverse_duration_sだけ保持。経過時間は指令時間であり実後退時間/距離ではない。
- 後退中に移動の証拠がなければSTOP。終了後は中立と静止傾向の確認を挟み、
  復帰終了後に受信した新しい通常指令から再開する。
- 復帰中、rule速度PIDはactiveに従いI/Dをreset。上流の入力健全性監視は継続する。
- STOP/MANUAL/PROPOで即時中断。鮮度切れ・時計逆行・tick gap・publisher競合・
  累積角速度積分上限・試行上限は中立＋STOP要求。異常後はSTOP→新たなAUTOが必要。
  回数は通常前進の進展がprogress_reset_s続いた場合のみresetする。

この動作は低速・低滑り時に進入軌道を短く戻る近似。後方障害物・逆走・物理停止・
実舵角・車体並進の保証はない。フローには動く物体や回転/振動が混ざり得る。
後退試験は後方を確保したコースで行い、閾値・制動・後退遷移を実測して調整する。

## センサ構成

| sensor_profile | 必須入力 | 動作 |
|---|---|---|
| infra_imu（既定） | image_topicの片側IR＋IMU | 低解像度の双方向検査付き疎フロー、複数画面領域の特徴点を要求 |
| rgb_imu | image_topicのRGB＋IMU | 同じ処理。image_topicもRGBへ変更する |
| evs_assisted | 上記の画像＋IMU＋イベント画像 | red_blue_white画像の活動率で静止判定をveto |

EVSは既存`/event_camera/event_image`を使い、raw packet配信や新たなカメラ占有を要求しない。
近傍支持のない孤立色画素を除き、活動率が閾値以上ならスタック/静止とみなさない。
EVSの無活動だけでは停止判定せず、EVSフロー・移動方向・速度も推定しない。
真のイベント時刻を使うノイズ除去や照明フリッカ除去は未実装で、センサ側の設定/フィルタ調整も必要。
EVS選択時の途絶はSTOPとし、黙って軽量構成へfallbackしない。

## 入出力

標準rosout/parameter_eventsと標準parameterサービスは表から省略。独自service/actionなし。
制御/状態topicは相対名（表はroot namespace）、センサtopicはparameterで指定する絶対名。

| 方向 | 既定topic | 型 | QoS・周期・意味 |
|---|---|---|---|
| 入力 | /recovery/input_cmd | kart_interfaces/msg/ControlCommand | reliable/volatile/depth1、元AUTO producerの補正前正規化指令 |
| 入力 | /operation_mode/state | kart_interfaces/msg/OperationModeState | reliable/volatile/depth1、mode heartbeat |
| 入力 | /realsense/infra1/image_rect_raw | sensor_msgs/msg/Image | best effort/volatile/depth5、最大15Hzで処理。mono8/rgb8/bgr8/rgba8/bgra8 |
| 入力 | /realsense/imu | sensor_msgs/msg/Imu | best effort/volatile/depth5、角速度norm使用。accel・orientation不使用 |
| 条件付き入力 | /event_camera/event_image | sensor_msgs/msg/Image | best effort/volatile/depth5、evs_assistedのみ。red_blue_white rgb8/bgr8 |
| 出力 | /auto/control_cmd | kart_interfaces/msg/ControlCommand | reliable/volatile/depth1、50Hz。通常は元stamp、生成指令は現在stamp。steering左正[-1,1]、駆動[0,1]排他 |
| 出力 | /operation_mode/request | kart_interfaces/msg/OperationModeRequest | reliable/volatile/depth10、fault中50HzのSTOP要求 |
| 出力 | /recovery/active | std_msgs/msg/Bool | reliable/volatile/depth1、50Hz、braking～resume_neutral中true |
| 出力 | /recovery/status | std_msgs/msg/String | reliable/volatile/depth1、10Hz＋状態変化。kart.recovery.v1 JSON、設定・採用span・固定操舵・観測値・理由 |

## 全parameter

全独自parameterは起動時固定(read_only)。変更はSTOPしてYAMLを編集し再起動する。
use_sim_timeはROS標準parameterだが起動後に変更しない。
pkgのconfig/recovery.yamlは単体起動基準、実運用正本はkart_bringup/config/recovery/recovery.yaml。
simはkart_bringup/config/sim/drive_recovery.yamlを使用し、ROS stamp/動作時間とsteady受信時間を分離する。

| 名前 | 既定値 | 意味 |
|---|---|---|
| `sensor_profile` | `infra_imu` | infra_imu / rgb_imu / evs_assisted。上記参照 |
| `image_topic` | `/realsense/infra1/image_rect_raw` | 使用する画像topic。RGB選択時は明示変更 |
| `imu_topic` | `/realsense/imu` | IMU topic。角速度のみ使用 |
| `evs_image_topic` | `/event_camera/event_image` | EVS併用時の既存イベント画像topic |
| `control_rate_hz` | `50.0` | steady制御timer周波数[Hz]、正値 |
| `image_rate_hz` | `15.0` | 画像処理のsteady上限周波数[Hz]、正値 |
| `input_timeout_s` | `0.25` | 元指令のROS stamp/steady受信期限[秒]、正値 |
| `sensor_timeout_s` | `0.3` | 画像・IMU・選択時EVSの期限[秒]。画像ペア間隔上限も兼ねる |
| `mode_timeout_s` | `0.3` | modeのstamp/受信期限[秒]、正値 |
| `future_tolerance_s` | `0.05` | 元stampの未来許容[秒]、非負 |
| `wall_timeout_s` | `0.0` | steady受信期限の明示上書き[秒]。0は各期限。sim設定5.0 |
| `processing_width` | `212` | フロー処理幅[pixel]、32以上。縦はアスペクト比を保持 |
| `min_features` | `20` | 有効追跡点の最小数、正整数 |
| `max_features` | `120` | 特徴点上限、min_features以上 |
| `min_feature_cells` | `4` | 4×3領域中の特徴点が必要な領域数、1..12 |
| `flow_fb_error_px` | `1.0` | 前後追跡誤差上限[pixel]、正値 |
| `evs_min_neighbors` | `2` | 孤立色画素除去の3×3近傍内活動画素数（自身含む）、1..9 |
| `evs_activity_threshold` | `0.02` | EVS支持付き色画素率の静止veto閾値、0..1 |
| `history_span_s` | `0.5` | 疑い開始前に参照する操舵区間[秒]、正値 |
| `history_offset_s` | `0.15` | 参照区間末尾を疑い開始より前へずらす秒数、非負 |
| `history_min_samples` | `5` | 参照区間内の必要な異なる元指令sample数、正整数 |
| `history_max_gap_s` | `0.12` | 参照区間内および両端の最大欠落間隔[秒]、正値 |
| `min_forward_throttle` | `0.03` | 履歴採用・スタック監視の前進要求下限(0,1] |
| `startup_grace_s` | `1.0` | AUTO進入/復帰終了後のスタック検出猶予[秒]、非負 |
| `stuck_duration_s` | `0.8` | 低フロー・低角速度・前進要求が続く疑い時間[秒]、正値 |
| `stationary_flow_px_s` | `2.0` | 静止傾向のフロー上限[pixel/秒]。処理解像度基準 |
| `moving_flow_px_s` | `6.0` | 進展のフロー下限[pixel/秒]。静止閾値より大きい |
| `stationary_gyro_rad_s` | `0.12` | 静止/進展判定を許す角速度norm上限[rad/秒]、非負 |
| `brake_command` | `0.1` | 復帰開始時の制動要求(0,1]。ESC処理はbridge |
| `brake_min_s` | `0.2` | 制動要求の最小保持時間[秒]、非負 |
| `stop_confirm_s` | `0.25` | 画像/角速度の静止傾向が続く確認時間[秒]、正値 |
| `stop_timeout_s` | `2.0` | 制動後の確認期限[秒]。brake_min_s＋stop_confirm_sより大きい |
| `reverse_neutral_s` | `0.2` | 制動後、後退前に中立を出す時間[秒]、非負 |
| `reverse_throttle` | `0.08` | 一定後退要求(0,1]。物理速度ではない |
| `reverse_duration_s` | `0.5` | 一定操舵・後退要求を出す時間[秒]、正値 |
| `recovery_steering_scale` | `1.0` | 過去操舵中央値への非負倍率。符号反転は禁止 |
| `recovery_steering_limit` | `0.5` | 後退操舵の絶対値上限[0,1] |
| `resume_neutral_s` | `0.3` | 後退後、再開前の中立最小時間[秒]。静止確認も要求 |
| `max_recovery_attempts` | `2` | 進展でresetされるまでの復帰回数上限、正整数 |
| `progress_reset_s` | `3.0` | 回数resetに必要な通常前進の進展継続時間[秒]、正値 |
| `max_recovery_rotation_rad` | `0.35` | 復帰中の角速度norm積分の上限[rad]。yaw角ではなく保守的な全軸回転量 |
| `max_tick_gap_s` | `0.25` | 制御時刻の最大進み[秒]。超過/逆行はfault。simではROS時間 |
| `use_sim_time` | `false` | ROS標準。実機false、sim true。動作途中の変更はしない |

## 起動と調整

Dockerへpython3-opencv依存を追加したため、通常の開発イメージ再ビルド後に実行する。

```bash
cd /workspaces
bash scripts/build.sh --packages-up-to kart_bringup
source ros2_ws/install/setup.bash
```

1. STOP中に`bash scripts/bringup.sh`のdrive/e2eで自動復帰を選択する。CLIでは`--recovery infra_imu`等。
   未指定時の静的既定はbringup/config/recovery/launch.yamlのenabled（既定false）。
2. 同ディレクトリのrecovery.yamlでprofile/image_topic、履歴span/offset、操舵倍率/上限、後退出力/時間を調整する。
3. 通常のbringupからdriveまたはe2eを起動する。録画STARTとAUTOは従来通り別操作。
4. `/recovery/status`と`/vehicle/control_cmd`で採用区間・固定操舵・phase・指令を確認する。
   動作確認は後方が空いている環境で行い、最初にブレーキ→中立→実後退のESC遷移を確認する。

単体起動時は上流producerを`auto/control_cmd:=/recovery/input_cmd`へremapして一つだけ起動する。
同時に通常bringupから復帰nodeを二重起動しない。

```bash
ros2 run kart_recovery recovery --ros-args \
  --params-file /workspaces/ros2_ws/src/kart_bringup/config/recovery/recovery.yaml
ros2 topic echo /recovery/status
```

モジュールだけの診断は前進を伴うAUTOにせず、STOPで画像/IMUのobservation_validとflowを確認する。
フロー閾値は処理解像度に依存し、RGBとIRでは独立して実測調整する。
記録する設定はstatus内のsettings。bagには元指令・active・statusも保存される。
通常学習教師は既存MANUAL/固定AUTO条件に従い、今回の自動復帰を教師として採用しない。

## 検証

```bash
PYTHONPATH=ros2_ws/src/kart_recovery:ros2_ws/src/kart_bringup \
  python3 -m pytest -q ros2_ws/src/kart_recovery/test ros2_ws/src/kart_bringup/test/test_recovery.py
# ROS環境
cd /workspaces/ros2_ws
colcon test --packages-select kart_recovery kart_control kart_bringup
colcon test-result --verbose
```

portableで状態遷移、履歴の参照/符号/上限、固定後退時間、異常中断、試行上限、合成画像の
フロー/特徴不足、イベント画像の孤立画素除去、ROS transportをstubにした鮮度/中断を確認する。
ROSビルド・launch/ESC結合・実センサbag・sim接触・実車走行は未確認。
