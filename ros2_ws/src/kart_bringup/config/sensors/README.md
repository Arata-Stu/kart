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
| `/realsense/imu` | sensor_msgs/msg/Imu | accel補間＋gyro、400Hz設定（実受信Hz未検証） |
| `/realsense/accel/sample`, `/realsense/gyro/sample` | sensor_msgs/msg/Imu | 個別IMU、accel 200Hz / gyro 400Hz設定 |
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
  `gyro_fps=400`, `accel_fps=200`, `unite_imu_method=2`: Hzと線形補間方式。
  通常・環境記録とも、2026-10-23の実機D455（S/N 239222300906、FW 5.16.0.1）の
  `rs-enumerate-devices -o -c`が報告する高速側を採用（gyro 200/400Hz、accel 100/200Hz）。
  旧accel 250Hzはこの実機で非対応。設定変更後はbringupを再起動し、
  ドライバのOpen profileログでGyro 400Hz・Accel 200Hzを確認する。実受信Hzは未検証。
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
| /event_camera/tensor | isaac_ros_tensor_msgs/msg/TensorList | reliable/volatile/depth 8（cpu,cudaは4） | FP32 NCHW [1,20,120,160]、250 Hz |
| /event_camera/event_image | sensor_msgs/msg/Image | best effort/volatile/depth 2 | ON赤/OFF青/背景白、25 Hz |
| /event_camera/events_raw | event_camera_msgs/msg/EventPacket | best effort/volatile/depth 8 | 既定OFF、動的ON可 |
| /event_camera/diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth 1 | driver/画像診断、1 Hz |
| /event_camera/tensor_diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable/volatile/depth 10 | tensor診断、1 Hz |

ノードは/event_camera/{tensor_pipeline,event_camera_driver,event_tensor,event_preprocessor}。
`--no-evs-image`で起動した場合は`event_image_enabled=false`となり、画像publisher自体を
作らない。rosbag等が購読しても画像生成・配信は開始しない。変更は起動時に適用する。
標準ROS自動topicは省略。RAW start/stop/splitは同namespaceのstd_srvs/srv/Trigger。
Tensorはbag標準対象から除外。RAWと制御stampの正確な同期、GPU共有、実機性能は未検証。

Bias配置先はproject rootの`bias/evs/`。運用YAMLのbias_file既定は空。
起動時overrideのevs_bias_fileは空ならYAML保持、@defaultならカメラ既定、
それ以外は検証した.bias絶対パス。evs_backend/evs_serialも明示時のみ上書きする。
`bringup.sh`のTUIでdecoderとbiasを選択できる。

`rgb_profile` / `infra_profile` launch引数（空=YAML保持、`0`=OFF）は
解像度とFPSを一括指定する。例：`rgb_profile:=1280x720x30`。
候補一覧・対応条件は[bringup README](../../README.md#解像度とfpsの明示選択)を参照。
同一streamの`*_fps`との併用は拒否する。既定YAMLの解像度は変更しない。
実験用Infra候補`848x100x100`を追加（ユーザー提示一覧、2026-10-29）。RGBには追加しない。
左右の画素形式は既存Y8のまま。左右Y8・100 Hzの同時取得とCameraInfoは実機未確認。
Depth有効時は既存の同一profile規則が働くため、Z16/IMUとの併走も未確認。

環境記録用の完全な設定は`realsense_environment.yaml`。
`mission mode:=environment`でのみ選択し、RGB 1280×720×30・Depth/Infra 848×480×30・
加速度250Hz・gyro200Hz・線形補間統合IMUを有効にする。
DepthはZ16、未整列、点群なし、emitter無効。Infra profileの変更はDepthにも適用する。
通常の`realsense.yaml`はDepth無効のまま。

### bringupのIMU選択

対話bringupではRGB/Infraの後にIMU取得レートを選択する。
低Hzは加速度100Hz・ジャイロ200Hz、高Hzは加速度200Hz・ジャイロ400Hz、
なしは`enable_accel=false`・`enable_gyro=false`。有効時の`unite_imu_method=2`は線形補間。
現在のYAMLに対応する候補を既定とし、「設定を保持（YAML）」でも起動できる。
CLIは`--imu-profile low|high|off`、missionとRealSense moduleのlaunch引数は
`imu_profile:=low|high|off`（既定空文字でYAML保持）。collect/environment/drive/e2eで使用する。
低・高のレートはD455（FW 5.16.0.1）の実機列挙結果に基づく。
VIOまたはIMUを必要とする自動復帰と「なし」の組み合わせは起動前に拒否する。
変更は起動時のみ。既存topic名・型は変更しない。offline評価のIMUは記録bagを使用する。

```bash
bash scripts/bringup.sh --mode collect --imu-profile low
bash scripts/bringup.sh --mode environment --imu-profile high
bash scripts/bringup.sh --mode collect --imu-profile off
```

設定変換と対話選択のportable testを用意。ROS結合・実機の取得Hzは未検証。

### D455のstream別profile候補

2026-10-23の`rs-enumerate-devices -o -c`（D455 S/N 239222300906、FW 5.16.0.1、USB3.2）
に基づき、RGB8と左右Infra Y8のTUI候補を分離した。RGB 848×480は最大60Hz、
Infra 848×480は90Hzまで対応するため、RGBの848×480×90は候補から除外する。
RGB 640×480も最大60Hz。RGBの90Hz候補は424×240、480×270、640×360。
collect/environment/drive/e2eで同じ検証を使用する。profile引数だけでなく旧FPS引数と
有効なstreamのYAMLも検査し、非対応値はドライバ起動前に拒否する。
これらは個々のprofileの対応確認であり、複数stream同時配信の安定性は実機で別途確認する。

### cuda_async直接集計の選択

openeb_tensor_cuda_async.yamlのevent_aggregation（string、既定events）をevt3_runsへ
変更するか、全parameter入りopeneb_tensor_cuda_async_runs.yamlをevs_tensor_configへ
指定する。backendはcuda_asyncのまま。変更は再起動時に反映する。
受信byte列からCPU binへ直接加算し、完成FP32 TensorをGPUへ転送する。
イベント配列・イベント単位の転送を省くが、RAW所有bufferと全Tensor H2Dは残る。
出力shape/topicは同じ。ROS/USB/CUDA/Orinでの速度・結合は未検証。

### 直接集計のGPU併用を選択

cuda_asyncのevent_aggregation=evt3_runs_gpuは、CPUで直接bin集計、変更binのみGPU転送、
GPUでTensor構成・正規化を行う。既定eventsとCPU版evt3_runsは維持。
全設定入りconfig/sensors/openeb_tensor_cuda_async_runs_gpu.yamlをevs_tensor_config
（CLIでは--evs-tensor-config）に指定し、backend=cuda_asyncで再起動する。
診断tensor_upload=dirty_cpu_binsを確認する。MacでCPU模擬の一致は確認、CUDA/Orinは未検証。
