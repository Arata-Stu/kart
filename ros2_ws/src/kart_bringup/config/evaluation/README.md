# Offline localization確認（Notebook）

top-level evaluation.launch.pyがlocalizationとHDMapをincludeする。
modules/evaluation/replay.launch.pyはbag再生とreplay_ready、
rviz.launch.pyはRVizを所有する。下位localization moduleはloadのみ。
実センサ、Joy、車両bridge、追従制御は起動しない。結果録画と周回VGLは明示的に有効化できる。

## 入力と時間・TF

rosbag2 metadata.yamlがあるdirectoryを指定する。左右ImageとCameraInfoは
localization/workflow.jsonの4 topic（既定/realsense/infra{1,2}/{image_rect_raw,camera_info}）。
sensor_msgs/msg/Image・sensor_msgs/msg/CameraInfo、および/tf_static（tf2_msgs/msg/TFMessage）が入力。
非空の記録を事前検証する。古い/tf・odom・制御指令・推定結果を除外する。
記録内の車体取付TFまで正しいかはこの検査では判断しない。

rosbag2_playerが/clock（rosgraph_msgs/msg/Clock）を100Hzで生成する。
localization・hdmap_server・rviz2はuse_sim_time=true。静的TFはtransient_local/reliableで再生。
VSLAMだけがmap→odom→base_frameを配信する。base_frame既定はcamera_link。
VGLはpose hintを送り、TFを配信しない。

## replay.yaml（ROS parameterではなくCLI設定）

対象: ROS Lyrical rosbag2_transport/ros2bag。
出典: https://github.com/ros2/rosbag2/tree/rolling/ros2bag/ros2bag/verb/play.py
CLI全機能を複製せず、採用する設定だけを記載する。再生開始時pauseとtopic限定はlaunchの固定動作。

| key | 既定 | 意味 |
|---|---|---|
| rate | 1.0 | 再生倍率。明示上書きは0より大きく4以下 |
| clock_hz | 100 | /clock出版Hz |
| read_ahead_queue_size | 1000 | 再生先読みmessage数 |
| extra_topics | [/tf_static] | カメラ4topicに追加する入力 |
| rviz | true | RViz起動。evaluation.launchのrvizで明示上書き可能 |
| base_frame | camera_link | 保存map評価でlocalizationが出力する姿勢frame。base_frame引数の空値で保持 |
| lap_vgl | false | 保存地図＋VGL評価で周回ごとのVGL探索を要求。lap_vgl引数の空値で保持 |

qos.yamlは/tf_staticだけkeep_last/depth=1/reliable/transient_local。
他topicのQoSはbag metadataを使用。物理センサを使うlive設定は変更しない。

## replay_ready node

ROS topicの購読者graphを観測してから/rosbag2_player/resume
（rosbag2_interfaces/srv/Resume）を呼ぶ。独自publish/subscribe topicなし。
再生開始後に終了するPythonの補助プロセスであり、画像を処理しないためcomponent化しない。
これはcomponent入力準備の判定であり、localization成功やGPU実行完了の保証ではない。
timeoutではbagをpausedのまま残し、ログでcomponent/engine確認を案内する。

| parameter | 既定 | 意味 |
|---|---|---|
| use_sim_time | false | 再生前なので壁時計を使う |
| imu_topic | 空文字 | VIO時にvisual_slamの購読も待つIMU topic。VOでは確認なし |
| image_topics | [/realsense/infra1/image_rect_raw, /realsense/infra2/image_rect_raw] | 準備を確認する左右画像 |
| subscriber_nodes | [visual_slam, visual_global_localization] | 両topicに必要な購読node名 |
| resume_service | /rosbag2_player/resume | 再生再開service |
| timeout_s | 120.0 | 準備・再開応答の待機上限。engine初回生成が長い場合は調整 |
| recorder_node | 空文字 | 任意の結果recorder名。指定時は/clockと配信済みrecord_topicsの購読を待つ |
| record_topics | [/clock] | 記録開始前に購読を確認するtopic。launchが結果記録の一覧を明示指定 |
| lap_node | 空文字 | 空なら周回準備確認なし。周回VGL有効時はlap_managerの準備を待つ |
| lap_input_topics | [/planning/reference_line, /visual_slam/tracking/odometry, /localization/vslam/diagnostics] | lap_nodeに必要な購読。ON時に実効lap設定から指定 |
| lap_status_topic | /localization/lap_status | lap_nodeが提供すべき状態publisher。記録の有無にかかわらずON時に待つ |

周回VGLがONのときは、3入力の購読と状態publisherのROS graph登録を確認してからresumeする。
結果recorderがONならlap_status publisher発見後にrecorderの購読準備も待つ。
bag停止中でもlap_managerの状態timerはsteady clockで動作する。
これはnodeの接続準備であり、参照ライン受信・自己位置確定・1周判定の成功は別途確認する。
`modules/evaluation/replay.launch.py`の`lap_node`・`lap_input_topics_json`・`lap_status_topic`は
全て空文字が既定。top-level評価launchだけが有効時の実効値を渡し、OFFではYAMLを保持する。

## 保存地図の周回VGL（任意）

`evaluation.launch.py`の`lap_vgl`は空文字が既定で、`replay.yaml`のfalseを保持する。
`lap_vgl:=true`またはbringup.shの保存地図＋VGL評価の周回設定で有効化する。
`enable_vgl=true`、同じmap座標のHDMap、選択laneの閉じた参照ラインが必要。
VSLAM単独・地図なし診断・開いたラインではONを拒否する。

ON時に`kart_lap_manager/lap_manager`（node名`lap_manager`）を1つ追加する。
全parameterの正本は`localization/lap_manager.yaml`で、共通の閉路・workflow・map frame検証を使う。
offlineでは`use_sim_time=true`、`base_frame`と`odom_frame`をlocalizationの実効値へ合わせ、
`vgl_enabled=true`・`cache_clear_enabled=false`を設定する。キャッシュ解放serviceは起動しない。
車両・センサ・追従制御の起動や走行指令の生成を伴わない。

入出力・判定parameterは[kart_lap_manager README](../../../kart_lap_manager/README.md)を参照。
主な入力は`/planning/reference_line`（`kart_interfaces/msg/ReferenceLine`、reliable / transient_local / depth 1）、
`/visual_slam/tracking/odometry`（`nav_msgs/msg/Odometry`、sensor data QoS）、
`/localization/vslam/diagnostics`（`diagnostic_msgs/msg/DiagnosticArray`、reliable / volatile / depth 10）。
状態`/localization/lap_status`とイベント`/localization/lap_events`は`std_msgs/msg/String`のJSON、
それぞれreliable / volatile / depth 1・10。既存`/visual_localization/trigger_localization`
（`std_srvs/srv/Trigger`）へ探索要求を送る。受付と再局在完了は区別する。

結果記録は`recording.yaml`のenabled=falseが既定（`eval_record:=true`でON）。
一覧へ`/localization/lap_events`・`/localization/lap_status`を含め、ON時は実効topicも重複なく加える。
manifest.jsonには`lap_vgl`と`lap_parameters`（OFFはnull）を保存する。
元bagの周回event・古い診断は再生せず、今回の推定結果から新たに周回を判定する。
開始・終了が周回途中なら完全な1周として数えない。早い再生倍率、VGL推論・時計待ち・
追跡品質の影響は実bagで確認する。bringupをCtrl-Cで終了すると結果recorderも終了し、元bagは変更しない。

`test_lap_evaluation_launch.py`は既定OFF・ON/OFF上書き・clock/frame・閉路制約・package確認・
manifestをROS非依存のaction stubで検証する。`test_replay.py`は周回nodeとrecorderのgraph準備待ちを検証する。
実ROS起動・実bagの周回検出／VGL再局在は対象環境での結合確認が必要。

## RViz

node名rviz2、parameterはrviz.yamlのuse_sim_time=true。
ROS Lyrical rviz2を使用し、display/tool設定はlocalization.rvizを正本とする。
RViz標準共通parameter以外に独自node parameterなし。

原本: NVIDIA公式isaac_ros_visual_slam、Apache-2.0、
[isaac_ros_cuvslam/rviz/default.cfg.rviz](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/e4dc8f59816dfca387bcb92364ecaf9207216475/isaac_ros_cuvslam/rviz/default.cfg.rviz)。

kart変更: Fixed Frame=map、画像display除外、HDMap MarkerArray/3種類のPath、
VGL PoseWithCovarianceを追加。既存のlandmarks/observations/localizer_map_cloud・VO/SLAM pathを保持。
HDMapのtopicは/hdmap/markers（visualization_msgs/msg/MarkerArray）、
/hdmap/{centerline,raceline,customline}（nav_msgs/msg/Path）。
VSLAM点群は/visual_slam/vis/{landmarks_cloud,observations_cloud,localizer_map_cloud}
（sensor_msgs/msg/PointCloud2）、軌跡は/visual_slam/tracking/{vo_path,slam_path}（nav_msgs/msg/Path）。
ただしNVIDIA release-5.0の[公式実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/release-5.0/isaac_ros_cuvslam/src/impl/visual_slam_impl.cpp#L335)では
observations可視化helperの初期化・終了がコメントアウトされており、topic定義があっても
`observations_cloud`の配信が接続されていない。APT実バイナリの動作は未確認で、可視化ONだけでは配信を保証しない。
2D Pose Estimateは/localization/pose_hint（geometry_msgs/msg/PoseWithCovarianceStamped）へ送る。
初期姿勢hintの受信とlocalize成功は別。VGL bootstrap/再要求とVSLAM localize結果を確認する。

### Offline評価の見方

Pathの配色は、HDMap参照ライン（centerline/raceline/customline）=灰色（RGB 190/190/190）、
SLAM Path=緑（25/255/0）、VO Path=青（25/25/255）。
`HDMap lanes and obstacles`（`/hdmap/markers`）は独自配色の参照ラインも重複表示するため既定OFF。
境界・障害物を確認するときは同displayをONにし、Namespacesで各laneの
`<lane>/centerline`・`<lane>/raceline`・`<lane>/customline`をOFFにして参照ラインの重複を避ける。
灰色Pathも確認したい種類だけONにする。新規ノード・topic・ROS parameterの追加はない。

1. **保存地図上の位置・向き**: Fixed Frame=`map`でSLAM PathとTFの実効base frame
   （評価の既定`camera_link`）を見る。実際に記録した走行と照らし、コーナーの順序・
   走行方向・折返し位置が妥当か、別の区間への誤対応がないかを確認する。
   参照ラインは走る目標であって真の走行軌跡ではなく、灰色との距離を自己位置誤差と扱わない。
2. **短時間の追跡**: 一時的にFixed Frame=`odom`へ切り替え、VO Pathと現在姿勢を見る。
   実際の動きにない停止・飛び・回転がないか、直進とカーブが連続しているかを確認する。
   この間はmap側の地図が動き得るため、地図に対する安定性は`map`へ戻して確認する。
3. **長時間・周回の安定性**: `map`で複数周を確認する。同じ場所を実際に再通過したときに
   位置・向きのずれが増え続けないかを見る。各周の実走行ライン差も含むため、
   軌跡の重なりだけで精度を決めない。Pathは保持数が有限で、全区間が残るとは限らない。
4. **地図補正・再局在**: 緑の現在位置が大きく変わった時刻を記録し、VGL hint・
   VSLAM診断・localize結果ログと照合する。`map→odom`は補正で変わり得るため、
   map表示の青い軌跡全体の移動やTFの長い接続線だけで追跡失敗と判定しない。
   VGL hintの表示・サービス受付は再局在完了ではない。
5. **追跡状態の裏取り**: RVizのdisplay Status=OKは表示・TF入力の成立確認。
   推定器の成功は別途`/localization/vslam/diagnostics`で`vo_status=OK`を確認し、
   保存地図評価では`localized_in_exist_map=Yes`も確認する。RViz標準displayではこの診断値を
   自動表示しないため、別terminalの`ros2 topic echo /localization/vslam/diagnostics`や結果bagを使う。

点群は位置合わせの補助に使う。表示中の点群が保存地図か現在の推定で生成したものかを区別し、
同じ推定に由来する点群と姿勢の重なりだけを独立した精度検証と扱わない。
VO/SLAM Pathの定義は[NVIDIA公式topic一覧](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html#ros-topics-published)、
odomの連続性とmap補正の区別は[REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)を参照。

初めは再生倍率1.0で結果記録をONにし、不審な地点は低倍率の別runで再確認する。
pause直後の鮮度警告と、元データの追跡不良は区別する。倍率変更は処理負荷・監視期限にも影響する。
cm/度単位の精度には、同じ時計・座標系・基準点の独立した正解姿勢が必要。
結果記録と`analyze_localization`によるLost/更新gap・参照比較は
[package README](../../README.md#offline-localization結果の記録と定量解析)を参照。
同じbagから作成した地図での評価と、別bagでの位置推定の再現性も区別する。

実bag、ROS、GPU、RViz表示は対象Notebookでの結合確認が必要。
位置合わせ・軌跡の目視確認用で、正解軌跡とのATE/RPE計算は含まない。

保存mapのoffline評価でも`tracking_mode:=0|1`でVO/VIOを切り替えられる。
既定空ならlocalization YAMLのVOを保持する。VIOではworkflow.jsonのimu
（既定`/realsense/imu`、sensor_msgs/Imu）が必須で、bag再生対象へ追加する。
replay_readyは指定された画像subscriberに加えてvisual_slamのIMU subscriberを待つ。
VGLにIMU購読は要求しない。古い動的TF・推定出力・clockは引き続き再生対象から除外する。
`base_frame`は未指定時にreplay.yamlのcamera_link、`imu_frame`は未指定時に
localization/cuvslam.yamlのcamera_gyro_optical_frameを保持する。
実機bagのIMUがcamera_imu_optical_frameなら明示指定する。
