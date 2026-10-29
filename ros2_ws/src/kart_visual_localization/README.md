# kart_visual_localization

NVIDIA Isaac ROSのROS側実装をそのままコピーし、cuVSLAMの参照地図を固定するための
内部パッチだけを適用する別package。以前の独自Pythonノードは撤去した。
**ソース一致・変更部分のC++構文は確認済み。ROS/CUDAでのビルド・bag・実車動作は未確認。**

## コピー元と差分

- ROS: [release-5.0 / e4dc8f5](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/tree/e4dc8f59816dfca387bcb92364ecaf9207216475)
- cuVSLAM: 同ROSが指定する[33da606](https://github.com/nvidia-isaac/cuVSLAM/tree/33da60662b27c0c5b63e0597bd778066af2731ed)（15.0）
- `src/`、`include/`、`test/`、`launch/`、`rviz/`はサンプルデータ`test/test_cases/`を除く49ファイルが上流とバイト一致。
  C++ namespace、Component class、node名も変更していない。
- package名は`kart_visual_localization`。ROS共有libは`libkart_visual_slam_node.so`、
  coreは`libkart_fixed_cuvslam.so`として分離。aptファイルを上書きしない。
- ROS側の同期・QoS・VO/VIO/RGBD・TF・twist・共分散・平面制約・診断・サービスはコピー元のまま。
  独自周期、外れ値閾値、pose有効期限は追加しない。
- coreの動作変更は`backend/fixed_reference.patch`の2ファイルのみ。
  別途、coreのCMakeにOUTPUT_NAME指定を追加する。公開SDK APIは変更しない。
- CMake/package.xmlはpackage/lib名とprivate coreビルドに必要な差分がある。
  上流launchはapt pkg名を含むため`upstream_launch/`へ参考用にinstallする。
  上流ROS launchテストもapt版を対象にしてしまうため自動登録しない。
  上流C++ message sequencerテストはそのまま登録する。

ROSコピーはApache-2.0。private coreとその変更はNVIDIA Community Licenseの対象で、
取得commitのLICENSEもpackageへinstallする。

これはこのcommitとのソース一致であり、実機にインストールされたaptパッケージの版・
ビルドオプションとの一致を確認したものではない。

## 固定マップ化で変わる動作

1. 初期化、Odometry、初期LocalizeInMapの探索は上流どおり。
2. LocalizeInMap成功後、元のread-only地図を`shared_ptr<const LocalizerAndMapper>`で保持する。
   `UnionWith`によるライブ軌跡と保存地図の統合をしない。
3. 地図照合・landmark/pose graph可視化は保持した固定参照を使う。
   参照地図へのキーフレーム追加、landmark統計更新、PGOはしない。
4. 照合で得た姿勢は上流の`SetAbsolutePose`を使ってライブ軌跡のheadへ適用する。
   初期照合の遅延は上流のkeyframe対応と相対移動で現在headへ伝播する。
   VO、ライブ軌跡グラフ、tail、ライブ側の平面制約/PGOは維持する。
   保存地図とのLC edge追加・共同PGOは固定地図に変更を加えるため使わない。
   この経路では元のLC共分散による共同最適化の代わりに上流の絶対姿勢拘束が使われる。
5. 固定参照取得後の`save_map`は失敗を返す。ライブ軌跡を参照地図として保存しない。
   初期照合前の一時SLAM・保存は上流どおり。Resetで固定参照も破棄される。
   load_mapは上流どおり次のlocalization対象を変え、成功後に固定参照が切り替わる。

固定対象は保存済みの参照地図。VO内部と現在走行の一時グラフまで凍結するものではない。
初期localizationが成功するまでは固定参照での測位は始まらない。
`localized_in_exist_map`も上流の成功ラッチのままで、独自の鮮度診断には変えていない。
地図照合周期は上流のkeyframe/worker/throttling処理のまま。
`slam_throttling_time_ms=500`は成功後の抑制期間であり、固定2 Hzタイマではない。

## ビルド・起動

対象はIsaac ROS release-5.0対応のLinux/ROS/CUDA開発環境。
依存のCUDA、cuda_buffer、isaac_common、isaac_ros_common、interfaces等は上流と同じ。
cuVSLAM依存ライブラリ取得にはネット接続も必要。

```bash
git clone https://github.com/nvidia-isaac/cuVSLAM.git /tmp/kart-cuvslam-source
git -C /tmp/kart-cuvslam-source fetch origin 33da60662b27c0c5b63e0597bd778066af2731ed
cd ros2_ws
colcon build --symlink-install --packages-select kart_visual_localization \
  --cmake-args -DKART_CUVSLAM_SOURCE=/tmp/kart-cuvslam-source
source install/setup.bash
```

`KART_CUVSLAM_SOURCE`は指定必須。aptライブラリへのfallbackはない。
prepare.pyは指定checkoutを変更せず、固定commitをbuildディレクトリへexportしてパッチを適用する。
パッチ変更後は新しいbuildディレクトリを使う（既存生成物を自動上書きしない）。
bringupのinstall更新も必要。既存のbuild手順で`kart_bringup`をbuild/sourceする。

既存apt VSLAMを終了し、カメラ・校正TFを用意してから:

```bash
ros2 launch kart_bringup fixed_localization.launch.py map_dir:=/absolute/map_bundle
# 地図原点付近から上流の起動時localizationを使う場合:
ros2 launch kart_bringup fixed_localization.launch.py \
  map_dir:=/absolute/map_bundle localize_on_startup:=true
```

通常は上流の`visual_slam/initial_pose`（bringupでは`/localization/pose_hint`）または
`visual_slam/localize_in_map` serviceから初期化する。初期姿勢の扱いは上流のまま。
このlaunchはカメラ、VGL、制御を起動しない。
引数`map_dir`・`use_sim_time`・`localize_on_startup`は既定空＝YAMLを維持する。
実運用は`kart_bringup/config/localization/fixed_localization.yaml`。
このファイルのparameter値は既存`cuvslam.yaml`と同一。
`workflow.json`の画像・IMU・hint・diagnostics remapを使う。

Component: `nvidia::isaac_ros::visual_slam::VisualSlamNode`。
単体executable: `kart_visual_localization`、node: `visual_slam`。
**namespace/class名もコピーを維持するため、apt版と同じprocess/containerにロードしない。**
標準TF/topic/service名も同じなので同時起動せず、別processとして切り替える。

## 入出力（上流既定名）

名前は上流と同じ。先頭`/`のない名前は相対名。`i`はcamera index。

|方向|topic|型|
|---|---|---|
|入力|`visual_slam/image_i`、`visual_slam/seg_mask_i`、`visual_slam/depth_0`|sensor_msgs/msg/Image|
|入力|`visual_slam/camera_info_i`|sensor_msgs/msg/CameraInfo|
|入力|`visual_slam/imu`|sensor_msgs/msg/Imu|
|入力|`visual_slam/initial_pose`|geometry_msgs/msg/PoseWithCovarianceStamped|
|出力|`visual_slam/trigger_hint`|geometry_msgs/msg/PoseWithCovarianceStamped|
|出力|`visual_slam/status`|isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus|
|出力|`visual_slam/tracking/vo_pose`|geometry_msgs/msg/PoseStamped|
|出力|`visual_slam/tracking/vo_pose_covariance`|geometry_msgs/msg/PoseWithCovarianceStamped|
|出力|`visual_slam/tracking/odometry`、`visual_slam/vis/slam_odometry`|nav_msgs/msg/Odometry|
|出力|`visual_slam/tracking/vo_path`、`visual_slam/tracking/slam_path`|nav_msgs/msg/Path|
|出力|`/diagnostics`|diagnostic_msgs/msg/DiagnosticArray|
|出力|`visual_slam/vis/observations_cloud`、`landmarks_cloud`、`loop_closure_cloud`|sensor_msgs/msg/PointCloud2（後2つも同じvis/ prefix）|
|出力|`visual_slam/vis/localizer_map_cloud`、`localizer_observations_cloud`、`localizer_loop_closure_cloud`|sensor_msgs/msg/PointCloud2（全てvis/ prefix）|
|出力|`visual_slam/vis/pose_graph_nodes`|geometry_msgs/msg/PoseArray|
|出力|`visual_slam/vis/gravity`、`pose_graph_edges`、`pose_graph_edges2`|visualization_msgs/msg/Marker（全てvis/ prefix）|
|出力|`visual_slam/vis/velocity`、`visual_slam/vis/localizer`|visualization_msgs/msg/MarkerArray|
|入出力|`/tf`、`/tf_static`|tf2_msgs/msg/TFMessage（staticは購読）|

サービスは`visual_slam/reset`（Reset）、`get_all_poses`（GetAllPoses）、
`set_slam_pose`（SetSlamPose）、`save_map`/`load_map`（FilePath）、
`localize_in_map`（LocalizeInMap）。全て`isaac_ros_visual_slam_interfaces/srv`、同じprefix。
ROS標準parameter/clock/rosoutインターフェースも上流どおり。

## パラメータ

新規ROS parameterなし。以下の既定値は**上流ノード**の値。
`config/cuvslam.yaml`はkartの運用設定をコピーしているため、平面制約true、camera frame、
path_max_size=2048などは上流ノード既定と異なるが、既存apt向けkart設定とは同じ。
全60項目＋use_sim_timeをYAMLへ明示している。

|parameter|上流既定|意味|
|---|---|---|
|num_cameras / num_input_masks|2 / 0|カメラ数・マスク数|
|multicam_mode / min_num_images|1 / num_cameras|マルチカメラ処理方式・必要画像数|
|sync_matching_threshold_ms|5.0|画像同期許容差ms|
|img_mask_top / bottom / left / right|各0|画像端の除外画素数（各名はimg_mask_で始まる）|
|enable_image_denoising / rectified_images|false / true|ノイズ除去・入力がrectifiedか|
|enable_ground_constraint_in_odometry / enable_ground_constraint_in_slam|false / false|VO/SLAMの平面制約|
|enable_localization_n_mapping|true|SLAMを有効化|
|tracking_mode|0|0=VO、1=VIO、2=RGBD|
|debug_imu_mode|false|IMUデバッグ|
|gyro_noise_density / gyro_random_walk|0.000244 / 0.000019393|ジャイロノイズモデル|
|accel_noise_density / accel_random_walk|0.001862 / 0.003|加速度ノイズモデル|
|calibration_frequency|200.0|IMU校正周波数Hz|
|depth_scale_factor / depth_camera_id / depth_enable_stereo_tracking|1000.0 / 0 / false|深度スケール・対応カメラ・stereo併用|
|image_jitter_threshold_ms / imu_jitter_threshold_ms|34.0 / 10.0|入力周期異常判定ms|
|save_map_folder_path / load_map_folder_path|空 / 空|保存先・localization参照先|
|localize_on_startup|false|起動時に原点付近からlocalization|
|localizer_horizontal_radius / localizer_vertical_radius|1.5 / 0.5|探索範囲m|
|localizer_horizontal_step / localizer_vertical_step|0.5 / 0.25|探索刻みm|
|localizer_angular_step|0.1745|探索角度刻みrad|
|slam_max_map_size / slam_throttling_time_ms|300 / 500|ライブグラフ上限・LC成功後の抑制ms（固定参照は削減しない）|
|map_frame / odom_frame / base_frame|map / odom / base_link|出力座標系|
|camera_optical_frames / imu_frame|空配列 / 空文字|校正用frame|
|image_buffer_size / imu_buffer_size|10 / 50|バッファ長|
|image_qos / imu_qos|SENSOR_DATA / SENSOR_DATA|入力QoS|
|image_qos_depth / imu_qos_depth|5 / 5|QoS depth（commonライブラリ定義）|
|override_publishing_stamp|false|出版時刻を現在時刻に置換|
|publish_map_to_odom_tf / publish_odom_to_base_tf|true / true|TF出版|
|invert_map_to_odom_tf / invert_odom_to_base_tf|false / false|TF反転|
|enable_slam_visualization / enable_observations_view / enable_landmarks_view|false / false / false|可視化処理|
|path_max_size|1024|出力path長|
|verbosity / enable_debug_mode / debug_dump_path|1 / false / /tmp/cuvslam|ログ・デバッグ出力|
|enable_request_hint|true|localization失敗時にhint要求|
|use_sim_time|false|ROS共通clock設定|

## 検証・残る確認

```bash
python3 ros2_ws/src/kart_visual_localization/backend/verify_upstream.py /path/to/isaac_ros_visual_slam
python3 ros2_ws/src/kart_visual_localization/backend/prepare.py /path/to/cuVSLAM /tmp/fresh-fixed-source
```

2026-10-29: サンプルデータ除外後の49ファイルのバイト一致、固定commitへのパッチ適用、変更AsyncSlamの実ヘッダを
使った非CUDA C++構文検査を確認。これらはリンクや実行の確認ではない。
上流ROS launchテストは保持しただけで実行していない。
core全776ファイルも照合し、差分は上記2ファイルとライブラリ名指定のCMakeだけと確認。
運用YAMLの値一致、Python構文、package XMLを確認。既存bringup設定/localizationの
portable回帰は21 passed、1 skipped、6 subtests passed。新native経路の実行テストではない。
以前の独自Python版のテスト結果はこの実装の検証結果に含めない。

対象環境でCUDA/ROS build、専用SONAMEへのリンク、保存地図のデータhash不変性、
bagでの初期localization/継続照合/遅延補正/平面制約/Reset/再localizationを確認する必要がある。
同じbag・設定のapt版との比較でVO・twist・TF・diagnosticsも確認する。
原本ではなくコピー地図で試験する。GPU負荷・精度・周回ドリフト改善は未測定。

## 固定マップ版のoffline可視化

既存offline評価に`--eval-localization fixed`を追加。
対話操作は`bash scripts/bringup.sh` →「オフライン解析」→
「固定mapでlocalization（NVIDIAコピー・特徴点群・軌跡）」→ VO/VIO →
HDMap・ライン・対応VSLAM地図・bagを選ぶ。

```bash
bash scripts/bringup.sh --mode eval --eval-localization fixed
```

非対話では`--map-file /absolute/hdmap.json --lane-id <lane> --line-type <line>`と
`--map-dir /absolute/map_bundle --bag /absolute/bag --tracking-mode 0`を指定する。
既存evaluation.launch.pyを使い、RVizで固定参照landmark、観測特徴点、LC点群、
VO/SLAM path、Odometry、pose graph、HDMapを表示する。
`enable_slam_visualization/enable_landmarks_view/enable_observations_view=true`を
offline実行時だけ上書きし、通常運用YAMLは変えない。
入力は左右Infra・CameraInfo・tf_static、VIO時のみIMU。古い動的TFや推定出力は再生しない。
clockをbagに合わせ、購読準備後に再生を開始する。センサ実機・制御・VGLは起動しない。
初期探索は既存VSLAM単独offlineと同じ原点付近。地図とbagの開始位置が離れる場合は
初期姿勢指定が必要。初期localization前の一時地図と、成功後の固定参照地図を区別する。

直接launchする場合は`evaluation.launch.py vslam_backend:=fixed enable_vgl:=false`。
既定`vslam_backend:=apt`は従来どおり。固定版でも上流Component classを保持し、
選択した一方だけをVSLAM containerへロードする。同一containerへ両方を混載しない。
package/private coreのビルドが必要。portable確認とROS/CUDA/RViz実行確認は別で、後者は未確認。

## 開発コンテナ起動時のLFS検査

上流`test/test_cases/`のサンプルbag・mapは通常のbuild/起動で使用しないため同梱しない。
ポインタだけをコピーすると`isaac-ros activate`のLFS検査でコンテナ起動前に停止する。
除外した6ファイルの出典とLFS OIDは`backend/upstream_test_assets.json`に記録する。
ROSノードのsrc/include、テストコード、実際に使用するbag/地図は変更していない。
この問題の解消にkart側で`git lfs pull`をする必要はない。
上流launchテストを実行する場合は、別の上流checkoutで対象commitとLFS実体を取得し、
対象package名を含めテスト環境を準備する。これらのテストは本pkgの通常buildには登録していない。
