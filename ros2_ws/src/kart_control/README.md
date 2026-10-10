# kart_control

オフライン生成ラインをPure Pursuitで追従し、物理単位の目標舵角・速度を出す。
既定node名`pure_pursuit`、単独実行`pure_pursuit`、C++ Component `kart_control::PurePursuitNode`。
`ament_cmake_auto`で構築する。参照ラインは既存`kart_hdmap/hdmap_server`が配信する。

`speed_controller`が速度PIDと舵角の正規化を行い、`/auto/control_cmd`へ接続する。
AUTO/MANUALの指令選択は既存muxが担当し、このpackageからAUTOへの変更を要求しない。
車体寸法・舵角の校正値とPID係数は実車に合わせる。既定は未校正・全gain=0で駆動しない。
無効時の速度0は目標値であり、物理的な停止・ブレーキ作動を示すものではない。

## TopicとTF

全topicは相対名。表はroot namespace時。ROS標準rosout/parameter_events省略。サービスなし。

| 方向 | 既定topic | 型 | QoS・意味 |
|---|---|---|---|
| 入力 | /planning/reference_line | kart_interfaces/msg/ReferenceLine | reliable/transient_local/depth1、選択ラインと各点速度 |
| 入力 | /visual_slam/tracking/odometry | nav_msgs/msg/Odometry | best effort/volatile/depth5、観測時刻と入力継続の監視 |
| 入力 | /localization/vslam/diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth10、VSLAM再局在状態 |
| 入力 | /tf, /tf_static | tf2_msgs/msg/TFMessage | tf2標準、map→tracking_frameを取得 |
| 出力 | /control/tracking_cmd | ackermann_msgs/msg/AckermannDriveStamped | reliable/volatile/depth1、30 Hz、舵角rad左正・速度m/s前進のみ |
| 出力 | /control/tracking_status | std_msgs/msg/String | reliable/volatile/depth1、30 Hz、trackingまたは無効理由 |
| 出力 | /control/lookahead | geometry_msgs/msg/PointStamped | reliable/volatile/depth1、計算成功時、map座標の追従点 |

Pure Pursuit基準は**後輪軸中心**。既定`rear_axle`へ校正済みTFが必要。
base_linkが後輪軸中心ならtracking_frameをbase_linkに設定できる。
カメラ中心を車体後輪軸と同一とみなすidentity TFは追加しない。
TFは最新odomメッセージのstampで取得する。地図配信stampでTF変換せず、TFを配信もしない。
VSLAM診断hardware_id=visual_slam、localized_in_exist_map=Yes、vo_status=OKを要求する。
ROS時刻の経過とsteady受信時間の両方でfreshnessを判定し、bag停止・巻戻しも不正値を出さない。

## 計算

最近傍線分へ投影し、進行方向へlookahead分の弧長を進んだ点を選ぶ。
後輪軸基準で `steering = atan2(2 * wheelbase * target_y, target_x² + target_y²)`。
曲率計算は[Coulterの原典](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm)に基づく。
舵角・速度を上限で制限し、閉路は末尾から先頭へ接続する。
速度は投影地点のprofileを線形補間し、lookahead内の減速目標と開路終端へ減速上限を適用する。
終端の残距離がgoal_tolerance以内なら0。profileの0を最低速度へ引き上げない。
後方目標、距離超過、重複点、非有限値、空ライン、TF失敗、古い入力は無効として0を配信する。

自己交差・近接する別区間の進捗管理、オンライン経路切替、障害物回避は未実装。
最近傍投影は自己交差で別区間を選ぶ可能性がある。選択lane内の単純なコースで評価する。
舵角飽和時の横加速度制約・タイヤ限界・全区間の最適な速度計画も扱わない。
ライン切替は即時で、走行中切替の連続性を保証しない。下流adapterでモード/有効状態を扱うこと。

## 全parameter

起動時read_only。実運用はkart_bringup/config/control/pure_pursuit.yaml。
package configは単独起動用。use_sim_timeのみROS共通。

| 名前 | 既定値 | 意味 |
|---|---|---|
| tracking_frame | rear_axle | 後輪軸中心frame |
| map_frame | map | ラインframeと一致必須 |
| wheelbase_m | 0.257 | YAML既定。TT-02標準仕様の暫定値、実車の設定を確認。node単独でYAMLなしは0 |
| max_steering_rad | 0.0 | 要校正。0は計算無効、0以上1.57未満 |
| lookahead_m | 0.8 | 正値、弧長先読み距離 |
| max_speed_mps | 1.0 | 正値、目標速度上限 |
| max_cross_track_m | 1.0 | 正値、ラインからの距離上限 |
| goal_tolerance_m | 0.1 | 正値、開路終端の停止距離 |
| deceleration_mps2 | 1.0 | 正値、目標速度減速計算。実減速度保証ではない |
| pose_timeout_s | 0.2 | odom stamp/受信の期限 |
| line_timeout_s | 2.5 | reference stamp/受信の期限。HDMap既定1Hz |
| localization_timeout_s | 2.5 | 再局在診断stamp/受信の期限 |
| control_rate_hz | 30.0 | 壁時計timer周期、正値 |
| use_sim_time | false | bagではtrue、clockを供給 |

## 起動・container

`ackermann_msgs`が必須。`package.xml`の依存に加え、CMakeでもComponentライブラリへ
`target_link_libraries`でexportされたターゲットへ明示的に依存を設定する。
`ament_auto_add_library`とリンク指定の形式を揃える。kartのDockerイメージには`ros-lyrical-ackermann-msgs`を導入する。
通常はDockerイメージに依存を導入済みのため、`rosdep install`は不要。
依存導入前の既存コンテナで一時的に補完する場合だけ`rosdep install --from-paths src --ignore-src -r -y`を使う。
ヘッダー未検出が続く場合は`ros2 pkg prefix ackermann_msgs`で参照先を確認し、
`/workspaces/scripts/workspace/build.sh --packages-select kart_control --cmake-clean-cache`でCMake設定を再生成する。

```bash
cd ros2_ws
colcon build --symlink-install --packages-up-to kart_control kart_hdmap kart_bringup
source install/setup.bash
ros2 launch kart_bringup tracking.launch.py map_file:=/data/maps/course/map.json lane_id:=main
ros2 topic pub --once /hdmap/select_line std_msgs/msg/String '{data: raceline}'
```

カメラ/VSLAM/車両は別起動。まずwheelbase/max_steeringを実値へ設定する。
Centerline速度はhdmap.yamlのcenterline_speed_mps（既定0）で明示する。
Race/Customは保存済み速度profileを使用する。
最上位tracking.launch.pyがcontainerを作成し、modules/control/pure_pursuit.launch.pyはloadだけ。
`container_name:=/perception create_container:=false`で既存containerにloadできる。
外部containerは停止・自動unloadしない。再load前の既存Component整理は所有側の責務。
HDMap serverは既存Pythonノードを再利用するため別process。既にHDMapを起動中なら最上位を重複起動せず
Pure Pursuit moduleだけをincludeする。入力remapが必要ならmoduleのComposableNodeに明示する。

## 検証

ROS非依存C++計算のテストは直線、左右符号、閉路wrap、終端、ゼロ速度、遠方/後方向、
非有限値、重複点、未校正値を確認する。
ROS Componentのbuild/load、TF通信、bag上の追従と実車制御は未確認。
`colcon test --packages-select kart_control kart_hdmap`で計算・メッセージテストを実行する。


## 速度PID Component

既定node名/単独実行`speed_controller`、Component `kart_control::SpeedControllerNode`。
`tracking.launch.py`は同じcontrol containerへPure Pursuitと速度PIDをloadする。
`modules/control/speed_controller.launch.py`単体はcontainerを作成しない。
静的parameterはread_only、PID調整項目はROS parameter serviceで動的変更可能。
実運用の正本は`kart_bringup/config/control/speed_controller.yaml`。

### 入出力

全topic相対名。以下はroot namespace。ROS標準parameter service、rosout等は省略。

| 方向 | topic | 型 | QoS/役割 |
|---|---|---|---|
| 入力 | /control/tracking_cmd | ackermann_msgs/msg/AckermannDriveStamped | reliable/volatile/depth1、目標舵角rad・速度m/s |
| 入力 | /control/tracking_status | std_msgs/msg/String | reliable/volatile/depth1、tracking/zero_speed_targetだけ有効 |
| 入力 | /visual_slam/tracking/odometry | nav_msgs/msg/Odometry | best effort/volatile/depth5、実測twist |
| 入力 | /operation_mode/state | kart_interfaces/msg/OperationModeState | reliable/volatile/depth1、AUTO許可とheartbeat |
| 入力 | /tf, /tf_static | tf2_msgs/msg/TFMessage | TF標準、odom child→rear_axleの剛体変換 |
| 出力 | /auto/control_cmd | kart_interfaces/msg/ControlCommand | reliable/volatile/depth1、50Hz、正規化指令。reverseは常に0 |
| 出力 | /operation_mode/request | kart_interfaces/msg/OperationModeRequest | reliable/volatile/depth10、AUTO中の異常時STOPを最大10Hz |
| 出力 | /control/speed_diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth1、最大10Hz、状態・target/measured/error・P/I/D/FF・throttle/brake |

odometryのtwistは`child_frame_id`座標/原点の速度として扱う。
そのstampでtracking_frameへの回転Rと、後輪軸からchild原点への位置pを取得し、
`v_axle = R v_child - (R omega_child) × p`の前進成分を使用する。
TF欠落・空child frame・非有限速度は拒否する。VSLAM速度が実車速度として十分な精度/遅延かは未確認。

`u = kp*(target-measured) + integral + kd*filtered(-d(measured)/dt) + kff*target`。
積分はki込みの寄与を±integral_limitに制限し、飽和を深める方向では積分しない。
D項は実測速度の微分へ一次フィルタをかけ、目標値の段差によるderivative kickを避ける。
正出力をthrottle、負出力をbrakeへ分離し、同時には出さない。brake_limit=0なら負出力は中立。
目標速度0では積分をリセットしthrottle=0とする。reverseは出さない。
舵角は`steering_angle / max_steering_rad`を±1へ制限。左右正負の基板変換は既存bridgeが行う。
この線形舵角換算は要実車校正。ESCへのブレーキ適用可否は既存bridgeのEscStateが判断する。

### パラメータ

| 名前 | 既定値 | 動的変更 | 意味 |
|---|---|---|---|
| kp | 0.0 | 可 | 比例gain、出力/(m/s) |
| ki | 0.0 | 可 | 積分gain、出力/m |
| kd | 0.0 | 可 | 微分gain、出力/(m/s²) |
| kff | 0.0 | 可 | 速度比例フィードフォワード、出力/(m/s) |
| integral_limit | 0.2 | 可 | I項の出力上限、0..1 |
| derivative_tau_s | 0.1 | 可 | D項一次フィルタ時定数、非負。0でフィルタなし |
| throttle_limit | 0.2 | 可 | 前進出力上限、0..1 |
| brake_limit | 0.0 | 可 | ブレーキ出力上限、0..1。既定無効 |
| tracking_frame | rear_axle | 不可 | 後輪軸中心、Pure Pursuitと一致 |
| max_steering_rad | 0.0 | 不可 | 0は未校正、正値かつ1.57未満。Pure Pursuitと一致 |
| input_timeout_s | 0.15 | 不可 | 目標/速度のstamp・受信およびtracking_status受信期限 |
| mode_timeout_s | 0.3 | 不可 | mode stamp/受信期限 |
| max_target_speed_mps | 1.0 | 不可 | 超過目標は異常としてSTOP要求 |
| control_rate_hz | 50.0 | 不可 | 壁時計timer周波数 |
| use_sim_time | false | ROS共通 | bag時true。実車bridgeはfalse |

gainは有限・非負のみ受理する。不正変更は拒否。
変更が確定した次の制御周期でI/D状態をリセットする（無衝撃切替を保証するものではない）。
AUTO以外では中立を配信して積分をリセットする。
AUTOへの移行時は100ms中立を維持し、既存muxのneutral-input条件に合わせる。
AUTO中に入力期限切れ/不正値/TF不足/追従無効が起きると中立とSTOP要求を出し、異常をラッチする。
有効なSTOP/MANUAL/PROPOを観測するまでラッチを解除しない。
モードheartbeatだけで自動的にAUTOへ復帰する要求は出さない。
正常時のControlCommand stampは目標と速度の古い方を保ち、muxで鮮度を再検査する。

### 調整手順

1. STOP中に後輪軸TF、両nodeの最大舵角、Pure Pursuitのwheelbaseを実値へ設定する。
2. Kpから低速で調整し、必要ならKi、最後にKdを加える。全gainの初期値は0。
3. Foxgloveで`/control/speed_diagnostics`の目標/実測速度、誤差、I項と飽和を確認する。
4. 使用する値をbringup YAMLへ記録する。`ros2 param set`の変更は再起動で消える。

以下の数値は操作例であり、車体に対する調整済みgainではない:

```bash
ros2 param get /speed_controller kp
ros2 param set /speed_controller kp 0.1
ros2 param set /speed_controller ki 0.01
ros2 param set /speed_controller kd 0.0
ros2 param set /speed_controller throttle_limit 0.15
ros2 topic echo /control/speed_diagnostics
```

複数係数を同時反映したい場合は標準`/speed_controller/set_parameters_atomically`
（rcl_interfaces/srv/SetParametersAtomically）を使用する。
実行中変更は可能だが、I/Dリセットで出力段差が生じ得るため、初期調整はSTOP中の変更と低速試験を分ける。

Portable testはPI計算、reset、飽和時anti-windup、brake分離、D kick防止、FF、不正gain/dtを確認。
ROS parameter serviceによる実変更、Component build/load、モード遷移/TF結合、実車応答は未確認。


TT-02標準ホイールベース257mmをpackage/bringup YAMLへ設定した。
出典と暫定外形は[車体寸法方針](../../../docs/policies/vehicle_geometry.md)。
最大舵角は引き続き未校正0であり、この寸法更新だけでは追従出力は有効にならない。
