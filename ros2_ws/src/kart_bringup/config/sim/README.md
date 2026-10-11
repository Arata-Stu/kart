# Simulation configuration

`sim.yaml`がkart_simの運用parameter正本。全parameter・topic・TF・センサ校正条件は
[kart_sim README](../../../kart_sim/README.md)を参照。
`sim.launch.py`はmap/map_dirからmap_fileを解決し、明示指定された
camera_enabled/viewer_enabled/require_mode/imu_enabled/publish_truth_tf/monitor_enabled、
stereo_hz/rgb_hz/imu_hz、camera_rig_file/record_dirだけ上書きする。
各引数の既定は空文字でYAMLを保持。mapの既定minicar_2026、map_dir空はinstalled map。
起動前に設定pathと明示上書きを表示する。実機bridge・実センサ・vehicle TFは起動しない。

`vslam.yaml`はsim_vslam.launch.pyの/visual_slam設定正本。
localization/cuvslam.yamlの全60parameterを複製し、use_sim_time=true、既存map読込みなし。
設定の出典・バージョン・変更理由はYAML先頭に記載。IMUノイズ係数は既存推定器設定を保持し、
仮想センサへノイズを加える意味ではない。200Hz IMUと60Hz stereoを基準にする。
launchのtracking_mode引数は既定空（YAMLの0=VO）、明示1でVIO。
入力は/realsense/infra{1,2}/image_rect_raw、対応camera_info、/realsense/imu。
出力・topic型は[既存VSLAM設定README](../localization/README.md)と同じ。
CUDA/cuVSLAMのあるLinuxでのみ使う。sim側publish_truth_tf=falseを必ず指定し、
VSLAMがmap/odom/base_linkの動的TFを所有する。センサ固定TFはsimが提供する。
真値はsim_world座標に分離されるため、評価では初期姿勢の位置合わせが必要。

`monitor_enabled`既定falseでlocalhostのカメラ/走行/録画操作を有効にする。
`record_dir`既定`record/sim`、相対パスはlaunch cwd基準。開始時に新しいbag directoryを作成する。
monitorのAuto lapはmapのwaypointを真値で追従する取得用controller。
録画はROSなしでも動くrosbags writerで、固定カメラTFだけを保存しmappingへ渡せる。
運転/録画は明示ボタンで開始し、録画中のresetは拒否する。外部ROS制御との優先順位はpackage README参照。

mapなしbag診断は `scripts/bringup.sh` の4番目 → mapなしVSLAM診断から実行する。
launch直接指定なら `ros2 launch kart_bringup sim_vslam.launch.py bag:=/absolute/bag visualize:=true`。
`bag`既定空はlive入力、`rate`既定空はevaluation/replay.yamlの1倍（bagが必要）。
`visualize`既定空はYAMLのfalse、trueでenable_slam_visualization /
enable_landmarks_view / enable_observations_viewを有効化する。
`rviz`既定空は可視化設定に追従、true/falseでRViz起動だけ明示変更できる。
`vslam.rviz`に左右raw画像、observations/landmarks/loop closure PointCloud2、VO/SLAM Pathを表示する。
左右CameraビューではCameraInfo・TFを用いて3Dランドマークを画像へ再投影する。
全2D検出pixelではなく追跡・三角測量済み点の投影であり、時刻・TF・校正が必要。
左右mono8のImage表示はTFなしでも入力画像を確認できる。topic型はlocalization READMEを参照。
bag再生は左右画像・CameraInfo・/tf_staticだけ（VIOでは/realsense/imuも）に限定し、
録画済み/clock・動的/tf・真値・車両指令を除外する。再生時刻はplayerが生成する。
/replay_readyはevaluation/replay_ready.yamlの全parameterを使用し、subscriber_nodesのみ
[visual_slam]へ変更する。入出力・既定parameterはevaluation READMEを参照。

RVizのImage表示は`Topic` propertyに画像topicとQoSを指定する。
`Image Topic`は対象RVizのproperty名ではないため使用しない。
