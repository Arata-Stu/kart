# オフラインVSLAM設定

`mapping.launch.py`は`isaac_ros_cuvslam`の`nvidia::isaac_ros::visual_slam::VisualSlamNode`を
`kart_mapping_container`へロードする。ノード名は`visual_slam`。このlaunch自身はbagを再生しない。
`kart_mapping build_map`がCollectorとbag再生・保存を管理する。

## 出典

- 対象APT: `ros-lyrical-isaac-ros-cuvslam` / Isaac ROS **release-5.0**。
- 参照commit: `e4dc8f59816dfca387bcb92364ecaf9207216475`、確認日2026-10-07。
- [公式node実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/e4dc8f59816dfca387bcb92364ecaf9207216475/isaac_ros_cuvslam/src/visual_slam_node.cpp)
- [公式API](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)
- 58個の独自parameter宣言と設定キーを照合。ROS共通`use_sim_time`を追加。
- このMacにはAPT版を導入していないためインストール版の確認は未実施。
  実行時に`dpkg-query`の版をsnapshotへ記録する。APT更新時は宣言差分を再確認する。

## launch引数と入出力

唯一のlaunch引数は必須の`job_file`（ジョブJSON絶対パス）。`workflow.overrides`だけがYAMLを上書きする。
トピックのリマップはworkflowが正本。YAML・launchから別の静的値を重ねない。
node/Component名と固定QoS付き出力は上流node実装に由来する。

| 方向 | 解決後の既定topic | 型 | QoS・意味 |
|---|---|---|---|
| 入力 | `/realsense/infra1/image_rect_raw` / `/realsense/infra2/image_rect_raw` | `sensor_msgs/msg/Image` | SENSOR_DATA。左右画像 |
| 入力 | `/realsense/infra1/camera_info` / `/realsense/infra2/camera_info` | `sensor_msgs/msg/CameraInfo` | image_qos。校正 |
| 入力 | `/realsense/imu` | `sensor_msgs/msg/Imu` | imu_qos。tracking_mode=1のみ |
| 入力 | `/tf_static` | `tf2_msgs/msg/TFMessage` | TF listener。静的外部姿勢 |
| 出力 | `/tf` | `tf2_msgs/msg/TFMessage` | TF broadcaster。map→odom→base |
| 出力 | `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | DEFAULT。odom姿勢と速度 |
| 出力 | `/visual_slam/tracking/vo_pose` | `geometry_msgs/msg/PoseStamped` | DEFAULT。VO姿勢 |
| 出力 | `/visual_slam/tracking/vo_pose_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | DEFAULT。共分散付き姿勢 |
| 出力 | `/visual_slam/tracking/vo_path` / `/visual_slam/tracking/slam_path` | `nav_msgs/msg/Path` | DEFAULT。軌跡 |
| 出力 | `/visual_slam/status` | `isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus` | DEFAULT。追跡状態 |
| 出力 | `/visual_slam/vis/landmarks_cloud` / `/visual_slam/vis/loop_closure_cloud` | `sensor_msgs/msg/PointCloud2` | DEFAULT。ランドマーク |
| 出力 | `/visual_slam/vis/slam_odometry` | `nav_msgs/msg/Odometry` | DEFAULT。SLAM姿勢 |
| 出力 | `/visual_slam/vis/pose_graph_nodes` | `geometry_msgs/msg/PoseArray` | DEFAULT。pose graph |
| 出力 | `/visual_slam/vis/pose_graph_edges` / `/visual_slam/vis/pose_graph_edges2` | `visualization_msgs/msg/Marker` | DEFAULT。pose graph辺 |
| 出力 | `/visual_slam/vis/gravity` | `visualization_msgs/msg/Marker` | DEFAULT。重力表示 |
| 出力 | `/visual_slam/vis/velocity` | `visualization_msgs/msg/MarkerArray` | DEFAULT。速度表示 |
| 出力 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | DEFAULT。診断 |

SENSOR_DATAはbest effort / volatile / depth 5、DEFAULTはreliable / volatile / depth 10。
上流には追加の観測点・localizer表示、initial_pose、reset/load/localize/get_all_poses等のAPIがあるが、
この新規地図作成workflowでは利用しない。`/rosout`・`/parameter_events`等のROS標準APIは省略。
利用するserviceは`/visual_slam/save_map`、型`isaac_ros_visual_slam_interfaces/srv/FilePath`。
出力の一部は上流の可視化フラグ・subscriber有無・追跡状態で生成が変わる。

## 全設定の値と意味

すべて起動時固定。実運用値は`cuvslam.yaml`、次表はその値を説明する。

| Parameter | kart値 | 意味 |
|---|---|---|
| use_sim_time | true | bagのclock |
| num_cameras / min_num_images | 2 / 2 | 左右2カメラ・同期に必要な枚数 |
| num_input_masks | 0 | セグメンテーションマスク数 |
| multicam_mode | 1 | 上流のカメラ構成モード |
| sync_matching_threshold_ms | 5.0 | 同期許容幅ms |
| img_mask_top / bottom / left / right | 各0 | 入力画像の周辺マスクpixel数 |
| enable_image_denoising | false | ノイズ除去 |
| rectified_images | true | 歪み補正済み画像 |
| enable_ground_constraint_in_odometry / in_slam | false / false | 平面拘束 |
| enable_localization_n_mapping | true | SLAM有効 |
| tracking_mode | 0 | Stereo VO。明示指定1でVIO |
| debug_imu_mode | false | IMU debug |
| gyro_noise_density / gyro_random_walk | 0.000244 / 0.000019393 | gyroの白色雑音・random walk |
| accel_noise_density / accel_random_walk | 0.001862 / 0.003 | accelの白色雑音・random walk |
| calibration_frequency | 200.0 | IMUノイズ校正周波数Hz |
| depth_scale_factor | 1000.0 | depth単位スケール（RGBD未使用） |
| depth_camera_id / depth_enable_stereo_tracking | 0 / false | RGBD関連（未使用） |
| image_jitter_threshold_ms / imu_jitter_threshold_ms | 34.0 / 10.0 | 入力間隔の警告閾値ms |
| save_map_folder_path / load_map_folder_path | 空 / 空 | 自動保存・読込先。kartはserviceで明示保存 |
| localize_on_startup | false | 新規作成なので既存地図にlocalizeしない |
| localizer_horizontal_radius / vertical_radius | 1.5 / 0.5 | localize探索半径m |
| localizer_horizontal_step / vertical_step | 0.5 / 0.25 | localize探索刻みm |
| localizer_angular_step | 0.1745 | localize角度刻みrad |
| slam_max_map_size | 300 | 上流SLAM map容量設定 |
| slam_throttling_time_ms | 500 | SLAM処理間隔ms |
| map_frame / odom_frame | map / odom | 出力座標系 |
| base_frame | camera_link | 推定する基準。未校正の車体外部姿勢を仮定しない |
| camera_optical_frames | camera_infra1_optical_frame, camera_infra2_optical_frame | 左右光学座標 |
| imu_frame | camera_gyro_optical_frame | IMU座標（VIOのみ） |
| image_buffer_size / imu_buffer_size | 10 / 50 | 入力buffer |
| image_qos / imu_qos | SENSOR_DATA / SENSOR_DATA | 入力QoS |
| override_publishing_stamp | false | 元画像時刻を保持 |
| publish_map_to_odom_tf / publish_odom_to_base_tf | true / true | TF出力 |
| invert_map_to_odom_tf / invert_odom_to_base_tf | false / false | TFを逆にしない |
| enable_slam_visualization | true | 地図可視化出力有効 |
| enable_observations_view | false | 2D特徴点表示 |
| enable_landmarks_view | true | HDMap背景に使う点群 |
| path_max_size | 50000 | path保持上限 |
| verbosity | 1 | cuVSLAM log level |
| enable_debug_mode / debug_dump_path | false / /tmp/cuvslam | debug dump |
| enable_request_hint | false | 新規作成なので外部pose hint要求なし |

上流との差: use_sim_time、base/camera/IMU frames、可視化・landmarks、path_max_size、enable_request_hint。
他の値は確認したソース既定値。前述の58 parameter以外を隠れたlaunch既定値として追加しない。

`workflow.json`の項目とCollectorのREADMEは[kart_mapping](../../../kart_mapping/README.md)を参照。

## 保存地図の可視化再生

`capture_cuvslam.yaml`はIsaac ROS 5.0 cuVSLAMの全parameterを列挙した点群取得用設定。
`mapping.launch.py`はoperation=captureのjob専用。`load_map_folder_path`を作業コピーへ明示上書きし、
`localize_on_startup=false`、`enable_request_hint=false`のままworkerがLocalizeInMapを要求する。
`save_map_folder_path`は空で元地図を保存し直さない。可視化・landmarks表示を有効にする。
元の`cuvslam.yaml`は旧設定で、この起動では使用しない。
出典はYAML冒頭に記載。診断成功判定は公式5.0実装のhardware_id=visual_slam、
localized_in_exist_map=Yes、vo_status=OKを使う。
`capture.json`の全設定とtopic型は[kart_mapping README](../../../kart_mapping/README.md)参照。
