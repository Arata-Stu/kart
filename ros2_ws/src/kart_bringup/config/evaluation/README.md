# Offline localization確認（Notebook）

top-level evaluation.launch.pyがlocalizationとHDMapをincludeする。
modules/evaluation/replay.launch.pyはbag再生とreplay_ready、
rviz.launch.pyはRVizを所有する。下位localization moduleはloadのみ。
実センサ、Joy、車両bridge、追従、録画は起動しない。

## 入力と時間・TF

rosbag2 metadata.yamlがあるdirectoryを指定する。左右ImageとCameraInfoは
localization/workflow.jsonの4 topic（既定/realsense/infra{1,2}/{image_rect_raw,camera_info}）。
sensor_msgs/msg/Image・sensor_msgs/msg/CameraInfo、および/tf_static（tf2_msgs/msg/TFMessage）が入力。
非空の記録を事前検証する。古い/tf・odom・制御指令・推定結果を除外する。
記録内の車体取付TFまで正しいかはこの検査では判断しない。

rosbag2_playerが/clock（rosgraph_msgs/msg/Clock）を100Hzで生成する。
localization・hdmap_server・rviz2はuse_sim_time=true。静的TFはtransient_local/reliableで再生。
VSLAMだけがmap→odom→base_linkを配信する。VGLはpose hintを送り、TFを配信しない。

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
| image_topics | [/realsense/infra1/image_rect_raw, /realsense/infra2/image_rect_raw] | 準備を確認する左右画像 |
| subscriber_nodes | [visual_slam, visual_global_localization] | 両topicに必要な購読node名 |
| resume_service | /rosbag2_player/resume | 再生再開service |
| timeout_s | 120.0 | 準備・再開応答の待機上限。engine初回生成が長い場合は調整 |

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
2D Pose Estimateは/localization/pose_hint（geometry_msgs/msg/PoseWithCovarianceStamped）へ送る。
初期姿勢hintの受信とlocalize成功は別。VGL bootstrap/再要求とVSLAM localize結果を確認する。

実bag、ROS、GPU、RViz表示は対象Notebookでの結合確認が必要。
位置合わせ・軌跡の目視確認用で、正解軌跡とのATE/RPE計算は含まない。
