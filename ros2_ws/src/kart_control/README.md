# kart_control

## 固定指令による車両特性試験

`kart_control::FixedCommandNode`（node/単独実行`fixed_command`）は、指定した正規化steering・
throttle・brakeをAUTO中だけ保持する。固定スロットルは固定車速ではない。
通常は`tuning_collect.launch.py fixed_command:=true`から単独processで起動し、
カメラ処理と制御timerの実行を分離する。Componentとしても登録する。
VSLAMの速度や推定状態に依存するフィードバック制御ではない。

起動後に新鮮なSTOPを観測し、その後AUTOに入った場合のみ有効化する。
AUTO移行後entry_neutral_sの間は全中立を出版し、既存mux/bridgeの中立確認に合わせる。
MANUAL/STOP/PROPO中もAUTO topicへ中立を出版する。MANUALのteleop操作は既存Joy/muxが担当する。
mode通信・stamp期限切れ、制御tick途絶、時計逆行、AUTO publisher競合で中立＋STOP要求をラッチし、
新鮮なSTOPを経るまで再開しない。起動／再起動時に既にAUTOでも開始しない。
duration_s>0は中立待機終了後の保持秒数、満了すると同じく中立＋STOP要求。
中立は物理停止の保証ではない。両軸固定時のJoy切断挙動は既存mode manager/Joy仕様に従う。片軸固定ではteleop途絶でSTOP要求。
VSLAM途絶だけでは固定出力を解除しない。

### 入出力

全て相対topic。以下はroot namespace時。QoSはreliable/volatile。
標準parameter/rosout等を除き独自service/actionなし。

|方向|topic|型|depth / 周期|
|---|---|---|---|
|入力|`/teleop/control_cmd`|`kart_interfaces/msg/ControlCommand`|1、Joyによる非固定軸の入力|
|出力|`/control/fixed_config`|`std_msgs/msg/String`|1、1Hzと設定変更時のJSON|
|入力|`/operation_mode/state`|`kart_interfaces/msg/OperationModeState`|1、mode heartbeat|
|出力|`/auto/control_cmd`|`kart_interfaces/msg/ControlCommand`|1、control_rate_hz（既定50Hz）。stampは生成時刻、frame=base_link。steering固定時のみ手動reverseを使用|
|出力|`/control/fixed_status`|`std_msgs/msg/String`|1、最大10Hz。active/inactive/auto_entry_neutral/異常理由|
|条件付き出力|`/operation_mode/request`|`kart_interfaces/msg/OperationModeRequest`|10、AUTO時の異常／満了STOP要求、最大10Hz|

競合検出は分散publisherの完全な仲裁ではない。rule-based、E2E、別の固定publisherと同時起動しない。

### 全parameter

`fixed_axes`・`steering`・`throttle`・`brake`・`duration_s`は新鮮なSTOP中のみ動的変更可能。
他の試験parameterはread_only。`use_sim_time`はfalseのみ許可する。
pkg一覧は`config/fixed_command.yaml`、実運用正本は`kart_bringup/config/tuning/fixed_command.yaml`。

|名前|既定|意味|
|---|---|---|
|fixed_axes|both|both / throttle / steering。固定対象軸|
|teleop_timeout_s|0.15|片軸固定時のteleop stamp・steady受信期限秒|
|use_sim_time|false|実機試験のROS実時間。trueは拒否|
|steering|0.0|左正[-1,1]の正規化要求|
|throttle|0.0|前進[0,max_throttle]。brakeとの同時指定不可|
|brake|0.0|制動要求[0,1]。実際の適用はbridgeのESC状態に従う|
|max_throttle|1.0|throttle指定の上限。固定値の超過は拒否、手動前進・後退はclip|
|control_rate_hz|50.0|指令／gateのwall timer Hz、10..100|
|mode_timeout_s|0.3|modeのROS stamp・steady受信鮮度と制御tick間隔の期限秒|
|entry_neutral_s|0.3|AUTO移行後の中立秒数、0.1以上|
|duration_s|0.0|0はモード変更まで保持、正値は試験保持秒数|

別端末で`bash scripts/vehicle_tuning.sh control`を起動すると、固定軸と値を対話変更できる。
5項目を標準`/fixed_command/set_parameters_atomically`へ一括送信し、範囲・組合せ・STOP鮮度を検証する。
拒否時は全項目を維持する。受理された値は次の制御tickまたはmode受信時に反映し、次回AUTOで使用する。
再起動・録画停止は不要。設定はメモリ上のみでYAMLへ保存しない。

|fixed_axes|固定する指令|Joyから使う指令|
|---|---|---|
|both|steering / throttle / brake|なし|
|throttle|throttle / brake|steering|
|steering|steering|throttle / brake / reverse|

片軸固定ではteleopのstamp・受信鮮度と値域を検証し、途絶／不正時は中立＋STOP要求をラッチする。
手動前進・後退はmax_throttleでclipする。非固定軸の設定値は保持するが出力へ使わない。
条件JSONにはrevision・fixed_axes・steering・throttle・brake・duration_s・max_throttleを含め、
`/control/fixed_config`へ1Hzと変更後最初のtickで出版する。revisionはprocess内連番。
標準bagは設定と最終車両指令を記録する。既存オフライン解析は設定JSONを採用判定には使わず、
実際の`/vehicle/control_cmd`で区間抽出する。
gateのportableテストは`test/test_fixed_command.cpp`。
ROS Component/単独実行のビルド・実機mux/bridge結合・実走行は未確認。
起動／録画／解析の手順は[vehicle_tuning](../../../tools/vehicle_tuning/README.md#固定スロットルステアでの記録)。

実車の速度ゲイン調整用に、[VSLAMによるオフライン解析](../../../tools/vehicle_tuning/README.md)を用意する。
`scripts/vehicle_tuning.sh`で記録要件確認・特性推定・既存SpeedPid式に基づく候補試算ができる。
候補YAMLはレビュー用で、実効parameterを変更しない。現行のノード／topic／parameterは変更しない。

オフライン生成ラインをPure Pursuitで追従し、物理単位の目標舵角・速度を出す。
既定node名`pure_pursuit`、単独実行`pure_pursuit`、C++ Component `kart_control::PurePursuitNode`。
`ament_cmake_auto`で構築する。参照ラインは既存`kart_hdmap/hdmap_server`が配信する。

`speed_controller`が速度PIDと舵角の正規化を行い、`/auto/control_cmd`へ接続する。
AUTO/MANUALの指令選択は既存muxが担当し、このpackageからAUTOへの変更を要求しない。
車体寸法・舵角の校正値とPID係数は実車に合わせる。最大舵角の既定は未校正0。速度制御YAMLは試験由来のFF表と暫定P補正を持つ。
無効時の速度0は目標値であり、物理的な停止・ブレーキ作動を示すものではない。

## TopicとTF

全topicは相対名。表はroot namespace時。ROS標準rosout/parameter_events省略。サービスなし。

| 方向 | 既定topic | 型 | QoS・意味 |
|---|---|---|---|
| 入力 | /operation_mode/state | kart_interfaces/msg/OperationModeState | reliable/volatile/depth1、追従設定の動的変更用STOP確認。stamp/受信0.3秒以内 |
| 入力 | /planning/reference_line | kart_interfaces/msg/ReferenceLine | reliable/transient_local/depth1、選択ラインと各点速度 |
| 入力 | /visual_slam/tracking/odometry | nav_msgs/msg/Odometry | best effort/volatile/depth5、観測時刻と入力継続の監視 |
| 入力 | /localization/vslam/diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth10、VSLAM再局在状態 |
| 入力 | /tf, /tf_static | tf2_msgs/msg/TFMessage | tf2標準、map→tracking_frameを取得 |
| 出力 | /control/tracking_cmd | ackermann_msgs/msg/AckermannDriveStamped | reliable/volatile/depth1、30 Hz、舵角rad左正・速度m/s前進のみ |
| 出力 | /control/tracking_status | std_msgs/msg/String | reliable/volatile/depth1、30 Hz、trackingまたは無効理由 |
| 出力 | /control/lookahead | geometry_msgs/msg/PointStamped | reliable/volatile/depth1、計算成功時、map座標の追従点 |
| 出力 | /control/tracking_reference | kart_interfaces/msg/TrackingReference | reliable/volatile/depth1、30 Hzとwatchdog無効通知。実参照点・計測時の車両位置・設定lookahead・直線距離・valid |
| 出力 | /control/tracking_markers | visualization_msgs/msg/MarkerArray | reliable/volatile/depth1、同周期。参照点・車両→参照点の矢印・L/pathと直線距離dのラベル |

Pure Pursuit基準は**後輪軸中心**。既定`rear_axle`へ校正済みTFが必要。
base_linkが後輪軸中心ならtracking_frameをbase_linkに設定できる。
カメラ中心を車体後輪軸と同一とみなすidentity TFは追加しない。
TFは最新odomメッセージのstampで取得する。地図配信stampでTF変換せず、TFを配信もしない。
TF取得失敗時は`tf_unavailable`と速度0を配信し、WARNに変換元・先frame、要求stamp、
pose age、tf2例外の詳細を最大1秒に1回記録する。
実機ではodomとTFの到着順の差を吸収するため、同じodom stampのTFを既定50 msまで待つ。
TransformListenerの専用threadで受信し、待機後も全入力の鮮度を再確認する。
最新TFへの代替・過去の指令の再配信・期限切れ入力の延命は行わない。取得失敗時のSTOP条件は維持する。
sim timeでは待機を0にし、停止した/clockがsteady watchdogを妨げないようにする。
利用者の実機ログで11.138 msの未来外挿を確認したが、変更後のROS結合・実車確認は未実施。
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

lookahead_m・max_speed_mps・max_cross_track_m・goal_tolerance_m・deceleration_mps2は
新鮮なAUTO/MANUAL/STOP中に動的変更可能。それ以外の独自parameterは起動時read_only。
実運用はkart_bringup/config/control/pure_pursuit.yaml。
package configは単独起動用。use_sim_timeのみROS共通。

| 名前 | 既定値 | 意味 |
|---|---|---|
| tracking_frame | rear_axle | 後輪軸中心frame |
| map_frame | map | ラインframeと一致必須 |
| wheelbase_m | 0.257 | YAML既定。TT-02標準仕様の暫定値、実車の設定を確認。node単独でYAMLなしは0 |
| max_steering_rad | 0.0 | 要校正。0は計算無効、0以上1.57未満 |
| lookahead_m | 0.8 | 正値、弧長先読み距離 |
| max_speed_mps | 10.0 | 正値、目標速度上限 |
| max_cross_track_m | 1.0 | 正値、ラインからの距離上限 |
| goal_tolerance_m | 0.1 | 正値、開路終端の停止距離 |
| deceleration_mps2 | 1.0 | 正値、目標速度減速計算。実減速度保証ではない |
| pose_timeout_s | 0.2 | ROS時刻でのodom stamp期限。wall_timeout_s=0なら受信期限にも使用 |
| tf_lookup_timeout_s | 0.05 | 起動時固定、同じodom stampのTFを待つ秒数。0..0.05かつpose_timeout_s未満。sim timeでは指定値によらず0 |
| line_timeout_s | 2.5 | reference stamp/受信の期限。HDMap既定1Hz |
| localization_timeout_s | 2.5 | 再局在診断stamp/受信の期限 |
| wall_timeout_s | 0.0 | 受信の実時間監視。0は各入力期限を使用、正値は独立したsteady秒の期限 |
| control_rate_hz | 30.0 | 正値。sim time有効時はROS timer、無効時はwall timer |
| use_sim_time | false | bagではtrue、clockを供給 |

## 起動・container

`ackermann_msgs`が必須。`package.xml`の依存に加え、CMakeでもComponentライブラリへ
`target_link_libraries`でexportされたターゲットへ明示的に依存を設定する。
`ament_auto_add_library`とリンク指定の形式を揃える。kartのDockerイメージには`ros-lyrical-ackermann-msgs`を導入する。
通常はDockerイメージに依存を導入済みのため、`rosdep install`は不要。
依存導入前の既存コンテナで一時的に補完する場合だけ`rosdep install --from-paths src --ignore-src -r -y`を使う。
ヘッダー未検出が続く場合は`ros2 pkg prefix ackermann_msgs`で参照先を確認し、
`/workspaces/scripts/build.sh --packages-select kart_control --cmake-clean-cache`でCMake設定を再生成する。

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
`test_tracking_tf_wait.py`はROS環境で実Pure Pursuitを起動し、15 ms遅れで届く同時刻TFでは
trackingを維持し、TF配信を止めるとtf_unavailableになることを検証する。
本変更時のmacOSではROS未導入のためskip。設定parityテストとROS非依存の追従計算テストのみ実行済み。
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

`u = FF(target) + kp*(target-measured) + integral + kd*filtered(-d(measured)/dt)`。
`ff_speed_mps`と`ff_throttle`が非空なら速度→定常スロットル表を線形補間し、
`kff`は使わない。両方空なら従来の`kff*target`。長さ一致・2点以上・速度は正かつ
厳密増加・スロットルは0..1で非減少を要求する。最小速度未満は仮定の(0,0)から補間し、
最大速度超は末尾スロットルを保持する。目標上限の判定は別途維持する。
両YAMLの初期表は、2026-10-23のthrottle_testで記録したAUTO要求値と、ユーザーが
定常速度として採用したcamera poseの水平速度中央値12点（下表）。
後輪軸の符号付き前進速度として実測校正された値ではない。路面・電池・旋回による差はP/Iで補正する。

| speed [m/s] | throttle |
|---|---|
| 1.41 | 0.100 |
| 1.90 | 0.125 |
| 2.27 | 0.150 |
| 2.67 | 0.175 |
| 3.05 | 0.200 |
| 3.35 | 0.225 |
| 3.69 | 0.250 |
| 3.83 | 0.275 |
| 4.14 | 0.300 |
| 4.34 | 0.325 |
| 4.63 | 0.350 |
| 4.74 | 0.375 |

`throttle_rise_rate` / `throttle_fall_rate`で正規化出力/sの増減を制限する（0は無効）。
起動／停止・異常による全reset後は出力0から再開し、変化率制限を深める方向でも積分を止める。
目標0・brake出力・異常・非AUTO・復帰中はthrottleを直ちに0へ落とす。
診断の`ff`が定常基準、`requested_throttle`がPIDと上限適用後・変化率制限前、
`throttle`が最終出力。reverseは追加しない。
積分はki込みの寄与を±integral_limitに制限し、飽和を深める方向では積分しない。
D項は実測速度の微分へ一次フィルタをかけ、目標値の段差によるderivative kickを避ける。
正出力をthrottle、負出力をbrakeへ分離し、同時には出さない。brake_limit=0なら負出力は中立。
目標速度0では積分をリセットしthrottle=0とする。reverseは出さない。
舵角は`steering_angle / max_steering_rad`を±1へ制限。左右正負の基板変換は既存bridgeが行う。
この線形舵角換算は要実車校正。ESCへのブレーキ適用可否は既存bridgeのEscStateが判断する。

### パラメータ

表はノード単独の既定値。pkg/bringupの実機YAMLはkp=0.02（暫定）、ki=kd=kff=0、
throttle_limit=0.4、max_target_speed_mps=10.0を指定する。操舵未校正の拒否は維持。
Pure Pursuit側も目標速度上限は10.0m/s。参照ラインの要求速度は別に設定する。
4.74m/s超ではFF表の末尾値0.375を保持し、P/I補正後のthrottle上限0.4は維持する。
上限10.0m/sは、その速度への到達や速度推定精度を保証するものではない。
simは表なし・変化率制限なしで従来のgainを使う。

| 名前 | 既定値 | 動的変更 | 意味 |
|---|---|---|---|
| kp | 0.0 | 可 | 比例gain、出力/(m/s) |
| ki | 0.0 | 可 | 積分gain、出力/m |
| kd | 0.0 | 可 | 微分gain、出力/(m/s²) |
| kff | 0.0 | 可 | 表が空の場合の速度比例FF、出力/(m/s) |
| ff_speed_mps | [] | 不可 | FF表の速度[m/s]。YAMLは上記12点 |
| ff_throttle | [] | 不可 | 各速度の定常要求値[0,1]。YAMLは上記12点 |
| throttle_rise_rate | 0.0 | 不可 | 増加上限[正規化出力/s]、0で無効。YAMLは0.10（暫定） |
| throttle_fall_rate | 0.0 | 不可 | 減少上限[正規化出力/s]、0で無効。YAMLは0.20（暫定） |
| integral_limit | 0.2 | 可 | I項の出力上限、0..1 |
| derivative_tau_s | 0.1 | 可 | D項一次フィルタ時定数、非負。0でフィルタなし |
| throttle_limit | 0.2 | 可 | 前進出力上限、0..1 |
| brake_limit | 0.0 | 可 | ブレーキ出力上限、0..1。既定無効 |
| tracking_frame | rear_axle | 不可 | 後輪軸中心、Pure Pursuitと一致 |
| max_steering_rad | 0.0 | 不可 | 0は未校正、正値かつ1.57未満。Pure Pursuitと一致 |
| input_timeout_s | 0.15 | 不可 | ROS時刻での目標/速度stamp期限。wall_timeout_s=0なら受信とtracking_status期限にも使用 |
| mode_timeout_s | 0.3 | 不可 | ROS時刻でのmode stamp期限。wall_timeout_s=0なら受信期限にも使用 |
| max_target_speed_mps | 10.0 | 不可 | 超過目標は異常としてSTOP要求 |
| wall_timeout_s | 0.0 | 不可 | 受信の実時間監視。0はinput/modeの期限、正値は独立したsteady秒の期限 |
| control_rate_hz | 50.0 | 不可 | sim time有効時はROS timer、無効時はwall timer周波数 |
| use_sim_time | false | ROS共通 | bag時true。実車bridgeはfalse |

gainは有限・非負のみ受理する。不正変更は拒否。
変更が確定した次の制御周期でI/D状態をリセットする。走行中変更では変化率制限の前回出力を保持する（無衝撃切替を保証するものではない）。
use_sim_timeは起動時に選択する。trueではPure Pursuit/速度制御の周期とPIDのdt、
AUTO開始の100ms中立待ちをROS時刻に合わせる。falseの周期・PID dtは従来のsteady時刻。
同じsim時刻を繰り返しても積分しない。時刻巻戻しではPID/入力有効状態を破棄し、
clock_jumpとしてSTOP要求とラッチを行う。0.2秒を超える制御dtは従来どおり異常。
実時間の100ms watchdogはROS clock停止中も動作し、受信途絶時に無効出力／STOP要求を行う。
sim運用ではwall_timeout_s=5.0、実機運用では0.0（従来の短い受信期限）を使用する。
`/control/speed_diagnostics`（diagnostic_msgs/msg/DiagnosticArray、既存topic）に
`control_dt_s`を追加し、実際に使用した時間差を示す。診断出力の間引きは実時間のまま。

AUTO以外では中立を配信して積分をリセットする。
AUTOへの移行時は100ms中立を維持し、既存muxのneutral-input条件に合わせる。
AUTO中に入力期限切れ/不正値/TF不足/追従無効が起きると中立とSTOP要求を出し、異常をラッチする。
有効なSTOP/MANUAL/PROPOを観測するまでラッチを解除しない。
モードheartbeatだけで自動的にAUTOへ復帰する要求は出さない。
正常時のControlCommand stampは目標と速度の古い方を保ち、muxで鮮度を再検査する。

### 調整手順

1. STOP中に後輪軸TF、両nodeの最大舵角、Pure Pursuitのwheelbaseを実値へ設定する。
2. 表のFFと暫定kp=0.02から低速で確認し、必要ならKi、最後にKdを加える。
   1.41m/s未満は未計測域の補間であり、発進・低速性能を検証した値ではない。
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
実行中変更は可能。I/Dリセット時も変化率制限の前回出力を保持し、0からの再ランプによる急減を避ける。
停止・異常・非AUTOでの全resetは従来どおり出力履歴も破棄する。

Portable testはPI計算、reset、飽和・変化率制限時anti-windup、brake分離、D kick防止、
FF表の全点・補間・端点保持・従来FF、停止時の即時中立、増減制限、不正表/gain/dtを確認。
ROS parameter serviceによる実変更、Component build/load、モード遷移/TF結合、実車応答は未確認。


TT-02標準ホイールベース257mmをpackage/bringup YAMLへ設定した。
出典と暫定外形は[車体寸法方針](../../../docs/policies/vehicle_geometry.md)。
package単独の最大舵角は未校正0であり、この寸法更新だけでは追従出力は有効にならない。
bringupの暫定舵角は末尾の「実運用の暫定舵角」を参照する。


## 共通復帰モジュールとの接続

bringupの`config/recovery/launch.yaml`で明示有効化すると、speed_controllerの
`/auto/control_cmd`は`/recovery/input_cmd`へremapされ、kart_recoveryがmux前段を所有する。
speed_controllerは入力`/recovery/active`（std_msgs/msg/Bool、reliable/volatile/depth1、50Hz）
がtrueの間、I/Dをresetして中立を出版する。tracking/odom/mode監視は継続する。
active=trueのまま受信がmode_timeout_s（既定0.3秒、wall_timeout_s指定時はそのsteady期限）を
超えると中立＋STOP要求をラッチする。新規parameter/service/actionなし。
復帰観測はVSLAM非依存だが、ruleの既存localization健全性要件は緩和しない。
[復帰仕様・全設定](../kart_recovery/README.md)を参照。


### Pure Pursuit参照点の表示

`/control/tracking_reference`は追従計算が実際に使用したtargetを配信する。
`lookahead_m`は最近傍経路投影点から前方へ進む設定経路長で、車両からtargetへの
直線距離`target_distance_m`と区別する。開路終端では参照点が終点へ制限される。
表示のために参照点や制御指令を再計算しない。新規parameter・config項目はない。

Foxgloveの3Dパネルは表示frameを`map`にし、`/control/tracking_markers`を有効にする。
黄色い球が参照点、矢印が制御に使った車両位置から参照点、ラベルが`L(path)`と`d`（m）。
計算無効・watchdog失効時はDELETEALLで消去し、有効markerにも0.5秒のlifetimeを付ける。
元の`/control/lookahead`（PointStamped）も維持するが、こちらは成功時のみの出力。
現在のAUTO状態とは独立した計算結果の表示である。

Map Studioの実車調整でも同じTrackingReferenceを読み、黄色い参照点・結線・数値を表示する。
受信と計測stampが0.5秒を超えたもの、無効結果は調整bridgeで転送しない。
UIの切断・通信停止時も参照点表示を消す。
portable C++計算テストでは横ずれ時の経路長／直線距離の違い・開路終端の参照点を検証。
ROSメッセージ生成・marker表示・実車での確認は別途必要。


### Map Studioのパラメータ調整

地図画面の実車調整 → パラメータ調整 → Pure Pursuitで、実ノードの現在値と
read_only属性を取得し、上記5項目をノード単位のset_parameters_atomicallyで変更できる。
ネイティブROS parameter serviceからの変更も最新STOP（mode stamp 0.3秒以内、受信は
既存wall_timeout_sの規則）を要求し、非有限値・0以下は拒否する。
validatorは検証だけを行い、全callbackが承認した後の次制御周期に採用する。
UIは追加で1秒以上の停車を要求する。速度PIDも同じ画面からkp/ki/kd/kff・積分上限・
微分filter時定数・出力上限を変更可能。PIDの既存動的更新と積分reset規則は維持する。
独立した速度controllerのmax_target_speed_mps等は起動時上限として保持される。
変更はメモリ上のみで、再起動後はYAMLに戻る。追加parameterはなく、全既定値は既存configと同じ。
C++計算のportable検証とUI/通信state machineの単体検証は実施し、ROS結合・実車は未確認。

### 実運用の暫定舵角（2026-10-26）

利用者申告の推定22〜27°をもとに、bringup/config/controlのPure Pursuitと速度制御を
同じ22°（0.3839724354 rad）へ設定した。実測校正値ではない。
ノード単独・package configの既定0（未設定時は無効）は維持する。
前述の「未校正0」は単独起動の既定であり、現在のbringup実運用値とは区別する。
この設定変更は、舵角線形性・左右差・実車での追従を証明しない。
