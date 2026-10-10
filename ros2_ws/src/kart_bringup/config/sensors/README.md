# Sensor modules

`mission.launch.py`だけがsensor containerを作成する。modules/sensorsのlaunchは既存containerへのloadのみ。
`create_sensor_container:=false sensor_container:=/name`で外部所有containerを使用できる。
RealSenseをsensor containerへload。OpenEBはdirect入力のため専用プロセスを所有。driveではVSLAM、e2eではimage encoder/TensorRTも同じcontainerへloadする。
VGL・vehicle・controlは別container。

## RealSense

`realsense2_camera::RealSenseNodeFactory`、ノード`/realsense`。
Dockerfile.realsenseのソースビルド版`r/4.56.3`を使用し、APT版を重複導入しない。
公式[rs_launch.py](https://github.com/realsenseai/realsense-ros/blob/r/4.56.3/realsense2_camera/launch/rs_launch.py)と
[parameters.cpp](https://github.com/realsenseai/realsense-ros/blob/r/4.56.3/realsense2_camera/src/parameters.cpp)を確認。
全設定は[realsense.yaml](realsense.yaml)。共通launch設定とD455の使用ストリーム・QoSを明示する。
SDKが機種ごとに列挙するセンサ制御、無効なfilter内部の調整値、image_transport pluginの設定は
使用せず、機種依存の既定値を保持する。GLSLはDocker側で使わず、そのコンパイル依存parameterを渡さない。

| 出力（既定） | 型 | 用途 |
|---|---|---|
| `/realsense/color/image_raw` | sensor_msgs/msg/Image | RGB記録、424×240、30Hz |
| `/realsense/color/camera_info` | sensor_msgs/msg/CameraInfo | RGB内部校正 |
| `/realsense/infra1/image_rect_raw`, `/realsense/infra2/image_rect_raw` | sensor_msgs/msg/Image | 左右Infra、424×240、60Hz |
| `/realsense/infra1/camera_info`, `/realsense/infra2/camera_info` | sensor_msgs/msg/CameraInfo | ステレオ校正 |
| `/realsense/imu` | sensor_msgs/msg/Imu | accel補間＋gyro、200Hz |
| `/realsense/accel/sample`, `/realsense/gyro/sample` | sensor_msgs/msg/Imu | 個別IMU、250/200Hz設定 |
| `/realsense/diagnostics` | diagnostic_msgs/msg/DiagnosticArray | カメラ状態、1秒周期 |
| `/tf_static` | tf2_msgs/msg/TFMessage | カメラ内部TF、transient local |

画像とCameraInfoはSENSOR_DATA QoS。公開されるSDK由来metadata/extrinsics topic、標準ROS管理topicは表を省略。
センサ入力はUSB。`camera_name: camera`、`base_frame_id: link`はTFの`camera_link`を作り、
`camera_infra1_optical_frame`等へ接続する。ROSノード名`realsense`とは別概念。
base_link→camera_linkは外部供給であり、ここでは配信しない。

`rgb_fps`/`infra_fps`launch引数の既定は空（YAML保持）。0で該当streamを無効、
30/60/90でprofileのHzだけを変更する。これは候補値であり、D455の全組合せの対応を保証しない。
`rs-enumerate-devices`と実際のtopic Hzで確認する。切替は起動時のみ、記録中の変更は行わない。
collectでもmapping用bagにはInfraを有効にする。driveではInfra OFFを起動前に拒否する。

設定の意味（既定値はYAMLが正本）:
- `use_sim_time=false`: 実機時計。`serial_no/usb_port_id/device_type=''`: 自動デバイス選択。
- `json_file_path/rosbag_filename=''`: SDK追加設定・SDK bag入力なし。`initial_reset=false`: 起動時リセットなし。
- `wait_for_device_timeout=-1.0`, `reconnect_timeout=6.0`: 接続待機・再接続間隔（秒）。
- `publish_tf=true`, `tf_publish_rate=0.0`: 内部静的TFのみ。`diagnostics_period=1.0`: 状態周期（秒）。
- `enable_color/infra1/infra2=true`, `enable_depth/infra/rgbd=false`: RGB＋左右IRのみ。
- `rgb_camera.color_profile='424x240x30'`, `depth_module.infra_profile='424x240x60'`: 幅×高さ×Hz。
- `depth_module.depth_profile='424x240x60'`: depthを別途有効化した際の設定。
  `depth_module.color_profile='0,0,0'`: 使用しないdepthモジュールcolorの自動profile。
- `rgb_camera.color_format=RGB8`, `depth_module.depth_format=Z16`, `depth_module.infra_format=RGB8`,
  `depth_module.infra1_format/infra2_format=Y8`: 各streamの画素形式。
- `rgb_camera.enable_auto_exposure=true`, `depth_module.enable_auto_exposure=true`: 自動露出。
  `depth_module.exposure=8500`, `gain=16`: 手動露出時のμs・SDK gain。
- `depth_module.emitter_enabled=0`: 投光OFF。`hdr_enabled=false`: HDRなし。
  `exposure.1=7500`, `gain.1=16`, `exposure.2=1`, `gain.2=16`: 未使用HDR各露光の設定。
- `enable_sync=false`, `depth_module.inter_cam_sync_mode=0`: wrapper同期なし、機器間同期なし。
- `enable_gyro/enable_accel=true`, `enable_motion=false`: USB個別IMUを使用、DDS機器のmotionは無効。
  `gyro_fps=200`, `accel_fps=250`, `unite_imu_method=2`: Hzと線形補間方式。
  `hold_back_imu_for_frames=false`: 画像待ちでIMUを保留しない。
- `angular_velocity_cov=0.01`, `linear_accel_cov=0.01`: IMU共分散設定。
  `clip_distance=-1.0`: 距離クリップ無効。
- `pointcloud.enable=false`: 点群生成なし。`stream_filter=2`, `stream_index_filter=0`:
  点群texture stream種別／index。`ordered_pc=false`, `allow_no_texture_points=false`: 未使用点群の形式設定。
- `align_depth/colorizer/decimation_filter/spatial_filter/temporal_filter/disparity_filter/hole_filling_filter/hdr_merge.enable=false`:
  すべての追加画像処理を無効化。
- `infra1_qos/infra1_info_qos/infra2_qos/infra2_info_qos/color_qos/color_info_qos=SENSOR_DATA`: 画像・内部校正のQoS。

## SilkyEvCam VGA（任意）

`openeb_ros2::DriverComponent`、ノード`/event_camera/event_camera_driver`。
`openeb_ros2`のkart branchを別途ビルドした環境でのみ`enable_evs=true`を使う。
必須のpackage依存には加えず、起動前のpackage検索で未導入をエラーにする。
設定は[openeb.yaml](openeb.yaml)。RAWはL1/R1と連携し、起動時に開始しない。
画像可視化とGPU Tensorを生成する。EVS E2Eモデルは未接続。bag_manager_openeb.yamlを選択して
`/event_camera/start_raw_recording`・`stop_raw_recording`（std_srvs/srv/Trigger）と連携する。
RAW設定はドライバparameter service経由。通常の診断は`/event_camera/diagnostics`（diagnostic_msgs/msg/DiagnosticArray）。
イベントpacket出版は無効。ROS標準管理topic/serviceは省略。

| parameter | 既定値 | 意味 |
|---|---|---|
| packet_publish_enabled | false | packetのROS出版を抑制 |
| serial / device_format / bias_file | 空 | 自動選択、機器形式・bias上書きなし |
| encoding | evt3 | packet encoding指定（出版無効） |
| frame_id | event_camera | センサframe。取付TFは別途必要 |
| raw_recording_enabled | true | native RAW記録機能 |
| raw_recording_auto_start | false | 起動時にRAWを開始しない |
| raw_recording_dir / raw_recording_basename | 空 | bag managerが記録開始時に設定 |
| raw_recording_split_duration_s | 0.0 | 時間分割なし |
| packet_duration_us | 1000 | packet時間幅 |
| publish_gap_warning_us | 4000 | packet出版間隔警告閾値 |
| packet_size_bytes | 1000000 | packetバイト上限 |
| publisher_depth | 8 | packet QoS depth |
| statistics_interval_s | 1.0 | 状態ログ周期 |
| debug | false | debug無効 |

Portableテストは`test/test_mission.py`。ROS component load、実機profile・画像配送、
TF、RAW/MCAP同時記録、GPU実行は未確認。

## EVS Tensor・可視化

OpenEB sourceはpackages.reposのf89015ba1f05d2fe432b270e73e132351c1a9377。
`modules/sensors/openeb.launch.py`はopeneb_tensor_pipelineへ全運用YAMLを渡す。
`openeb_tensor_pipeline.yaml`のtensor_backendはcuda_async（既定）/cpu/cudaから選択し、
対応するopeneb_tensor_<backend>.yamlを読む。全parameter・既定値・意味は
[parameter一覧](openeb_parameters.md)を参照（各運用YAMLは全parameterを記載、値変更なし）。
`openeb_visualization.yaml`は赤青白bgr8・25 Hz、購読者がいる時だけ生成。
起動時固定のTensor設定は再起動で変更する。driverのpacket_publish_enabledは動的変更可。

| 出力topic | 型 | QoS | 内容 |
|---|---|---|---|
| /event_camera/tensor | isaac_ros_tensor_msgs/msg/TensorList | reliable/volatile/depth 8（cpu,cudaは4） | FP32 NCHW [1,20,120,212]、250 Hz |
| /event_camera/event_image | sensor_msgs/msg/Image | best effort/volatile/depth 2 | ON赤/OFF青/背景白、25 Hz |
| /event_camera/events_raw | event_camera_msgs/msg/EventPacket | best effort/volatile/depth 8 | 既定OFF、動的ON可 |
| /event_camera/diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth 1 | driver/画像診断、1 Hz |
| /event_camera/tensor_diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth 10 | tensor診断、1 Hz |

ノードは/event_camera/{tensor_pipeline,event_camera_driver,event_tensor,event_preprocessor}。
標準ROS自動topicは省略。RAW start/stop/splitは同namespaceのstd_srvs/srv/Trigger。
Tensorはbag標準対象から除外。RAWと制御stampの正確な同期、GPU共有、実機性能は未検証。

Bias配置先はproject rootの`bias/evs/`。運用YAMLのbias_file既定は空。
起動時overrideのevs_bias_fileは空ならYAML保持、@defaultならカメラ既定、
それ以外は検証した.bias絶対パス。evs_backend/evs_serialも明示時のみ上書きする。
`bringup.sh`のTUIでdecoderとbiasを選択できる。
