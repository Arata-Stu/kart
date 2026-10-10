# 走行時VSLAM + VGL

Isaac ROS release-5.0を対象に、既存地図への再局在と追跡だけを起動する。
自身のROSノードはなし。カメラ、取付TF、車両、Foxglove bridgeは別起動。
静的設定はこのディレクトリが正本。引数で明示された値だけを上書きする。

## 構成と出典

| ノード | package / Component | container |
|---|---|---|
| visual_slam | isaac_ros_cuvslam / nvidia::isaac_ros::visual_slam::VisualSlamNode | kart_vslam_container |
| visual_global_localization | isaac_ros_visual_global_localization / nvidia::isaac_ros::visual_global_localization::VisualGlobalLocalizationNode | kart_vgl_container |

両方`rclcpp_components/component_container_mt`。VSLAMのinitial_pose callbackが
localize終了を待つ間も画像callbackを動かすためmulti-thread executorを使用する。
VGLとVSLAMは別process。可視化を止めても推定用画像の入力・画像処理は必要。

2026-10-08に確認した一次資料:

- [VGL概念・制約](https://nvidia-isaac-ros.github.io/concepts/visual_global_localization/index.html)
- [VGL公式実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_mapping_and_localization/tree/63cbbc20c0db49500107121f66d32b943f81bd99/isaac_ros_visual_global_localization)
- [VSLAM公式実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/tree/e4dc8f59816dfca387bcb92364ecaf9207216475/isaac_ros_cuvslam)
- [QoS補助実装](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/release-5.0/isaac_ros_common/src/qos.cpp)
- [VGL地図生成](https://nvidia-isaac-ros.github.io/concepts/visual_global_localization/tutorials/tutorial_cuvgl_map_creation.html)
- [VGLモデルのengine生成・実行](https://nvidia-isaac-ros.github.io/concepts/visual_global_localization/tutorials/tutorial_cuvgl_localization.html)

APT実版はこのMacでは未確認。実機では`dpkg-query -W 'ros-lyrical-isaac-ros-*'`を保存し、
上記commitとのparameter/API差分を確認する。visual_mappingの地図生成部分はAPT配布バイナリであり、
今回GPU実行はしていない。Dockerfile.kartにVSLAM/VGL/visual_mappingのAPT依存は導入済み。

## 地図とモデルの準備

cuVSLAMの`.mdb`だけではVGLはできない。VGLはkeyframe、画像特徴、BoW vocabulary/indexが必要。
Map Studioの公式offline生成で保存された`official/.../map_frames/rectified`と既存posesを再利用する。
元地図は変更せず、新しい出力フォルダにcuVSLAM/cuVGLと設定をまとめる。VSLAMは再計算しない。
これはHDMap点群captureとも別の工程であり、occupancy gridやFoundationStereo depth生成は不要。

モデルはALIKED + LightGlueを明示選択する。既定入力サイズは424×240だが、
**設定を書き換えるだけでは固定サイズONNXやTensorRT engineは小さくならない**。
対応するALIKED ONNXと、各GPU上で生成した互換engineを先に用意する。
JetPilotのALIKED workspaceでexportした成果物も下記配置なら使用できるが、実推論互換性は実機で確認する。
大型モデルへの暗黙fallbackはない。公式標準モデルを使う場合は実サイズを`--width/--height`で明示する。

```text
models/
└── aliked_lightglue/
    ├── aliked.onnx
    ├── aliked_<profile>.engine
    └── lightglue_aliked_<profile>.engine
```

各engineは1個だけ置く。JetPilot形式の`runtime_models/aliked_lightglue/*.engine`では
`runtime_models`の親の`aliked.onnx`も許可する。
公式exporterは`export_extractor_engine`と`export_lightglue_engine`。
上記公式tutorialの`--model_dir`を選択したONNXのフォルダへ、extractorの
`--configure_file`をその入力形状に一致する設定へ指定する。実行GPU/JetPack/TensorRTに合わせて生成する。
notepc製engineをそのままJetsonへ転送する運用を前提にしない。
地図生成側とJetson側では同じALIKED ONNXを使用し、engineだけ各環境で生成する。

ROS/GPUコンテナ内で:

```bash
cd /workspaces/kart/ros2_ws  # 実際のmount先に合わせる
colcon build --symlink-install --packages-select kart_bringup
source install/setup.bash
ros2 run kart_bringup prepare_vgl_map \
  --source-map /data/maps/course-v1 \
  --model-dir /data/models/aliked-424 \
  --output /data/localization/course-v1
```

`prepare_vgl_map`はROSノードではないCLI。必須引数は上記3つ。
`--width/--height`未指定時は`preparation.json`のwidth=424、height=240を使用。
feature_type=alikedは固定対応。元地図内・既存出力先への書込みは拒否する。
公式`create_cuvgl_map.py --help`で必要オプションを検査し、非対応版は終了する。
インストール済み`visual_mapping/configs/isaac`の全textprotoを新bundleへコピーし、
ALIKED入力形状と`save_debug_images=false`のみ変更する。
これはアルゴリズム内部設定の初期テンプレートであり、実行時はbundleの設定だけを読む。
GPUによるengine deserialize、出力ファイル検査の後に一時directoryをrenameする。
失敗時に不完全なbundleを完成品として公開しない。

`vgl_profile.json`にONNX、設定、cuVSLAM地図のhashと形状を保存し起動時も照合する。
engine hashは生成記録のみ（GPU間で異なるため同一性を要求しない）。
この検査は特徴の実精度やengineとONNXの数学的同一性を証明するものではない。

## 起動と再要求

左右のrectified画像・CameraInfo・校正済みbase_link→camera_link→光学frameのTFを別途供給する。
VGL/VSLAMで同じカメラ順序・外部姿勢を使う。

```bash
ros2 launch kart_bringup localization.launch.py \
  map_dir:=/data/localization/course-v1 \
  model_dir:=/data/models/aliked-424

ros2 service call /visual_localization/trigger_localization std_srvs/srv/Trigger '{}'
```

launch引数は`map_dir`、`model_dir`が必須。`base_frame`、`use_sim_time`は空が既定で
空ならYAMLを維持する。bagでは`use_sim_time:=true`にし`ros2 bag play BAG --clock --rate 1.0`。
bag収録済みの競合する動的TFを同時再生しない。
カメラ基準の試験では`base_frame:=camera_link`も可能だが、既にbase_linkを親に持つcamera_linkへ
odom→camera_linkを重複配信しないこと。車体では既定のbase_linkと実測取付TFを使う。

上流VGLは起動直後から探索する。成功すると推論を停止し、serviceまたはhint topicで再要求できる。
成功するまでは最大1 Hzで探索を続ける。探索終了のtimeout/cancelはこの構成にはない。
`Trigger.success`は要求受付のみ。`/localization/pose_hint`受信もVSLAMの再局在成功とは別。
`/localization/vslam/diagnostics`の`localized_in_exist_map=Yes`かつ`vo_status=OK`を確認する。
VGLのpose covarianceは現上流では既定ゼロであり、精度保証・確信度として使用しない。
Joyボタン割当・再局在中の走行制御は未接続。再要求serviceを後でJoy managerから呼べる構成とした。
VSLAM hint失敗→VGL再要求は接続したが、あらゆるtracking lossからの自動回復を保証するものではない。

## TFとFoxglove

動的TFはVSLAMだけが`map → odom → base_link`を発行する。VGLはmap座標のpose hintを送る。
HDMap/禁止領域は、このbundleのcuVSLAM地図と同じmap座標のものを読む。
別地図のHDMapは事前位置合わせが必要。TF名がmapというだけでは同一座標系とは限らない。
ループ閉じ・再局在でmap→odomは変わり得る。オンライン可視化は各stampのTFを使い、
オフライン点群の最終map→odomでの確定処理とは分ける。

Jetsonではenable_slam_visualization、landmarks、observations、VGL debug/rectified画像配信を無効化。
pose、odometry、Path、診断をFoxgloveで購読する。path_max_size=2048で保持数を制限する。
Pathはオンラインの軌跡であり、全過去姿勢を最終最適化したオフライン軌跡とは限らない。
VGL debug_image publisher自体は上流で生成されるが、debug無効時は画像payloadを出さない。
Foxglove bridge側でもカメラ画像topicを購読・転送しない設定にする（このlaunchはbridgeを起動しない）。

## 入出力

表はkartでremap後の名前。SENSOR_DATAはbest effort / volatile / depth **5を明示**。
DEFAULT出力はreliable / volatile / depth10。ROS標準rosout/parameter_eventsは省略。

| ノード・方向 | topic | 型 | QoS・条件 |
|---|---|---|---|
| 両方入力 | /realsense/infra1/image_rect_raw, /realsense/infra2/image_rect_raw | sensor_msgs/msg/Image | SENSOR_DATA、左右順 |
| 両方入力 | /realsense/infra1/camera_info, /realsense/infra2/camera_info | sensor_msgs/msg/CameraInfo | SENSOR_DATA |
| 両方入力 | /tf, /tf_static | tf2_msgs/msg/TFMessage | TF listener標準 |
| VGL出力→VSLAM入力 | /localization/pose_hint | geometry_msgs/msg/PoseWithCovarianceStamped | DEFAULT、VGL成功時、map座標 |
| VSLAM出力→VGL入力 | /visual_localization/trigger_localization | geometry_msgs/msg/PoseWithCovarianceStamped | 出力DEFAULT、入力reliable/volatile/depth100、内容は無視して探索要求 |
| VSLAM出力 | /visual_slam/tracking/odometry | nav_msgs/msg/Odometry | DEFAULT、odom基準 |
| VSLAM出力 | /visual_slam/tracking/vo_pose | geometry_msgs/msg/PoseStamped | DEFAULT、odom基準 |
| VSLAM出力 | /visual_slam/tracking/vo_pose_covariance | geometry_msgs/msg/PoseWithCovarianceStamped | DEFAULT |
| VSLAM出力 | /visual_slam/tracking/vo_path, /visual_slam/tracking/slam_path | nav_msgs/msg/Path | DEFAULT、購読・追跡状態による |
| VSLAM出力 | /visual_slam/status | isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus | DEFAULT |
| VSLAM出力 | /tf | tf2_msgs/msg/TFMessage | TF broadcaster標準 |
| VSLAM出力 | /localization/vslam/diagnostics | diagnostic_msgs/msg/DiagnosticArray | DEFAULT |
| VGL出力 | /localization/vgl/diagnostics | diagnostic_msgs/msg/DiagnosticArray | DEFAULT、探索診断 |
| VGL条件付き出力 | /visual_localization/debug_image | sensor_msgs/msg/Image | DEFAULT、この設定ではpayloadなし |

上流名はVSLAM入力`visual_slam/image_{0,1}` / `camera_info_{0,1}`、
VGL入力`visual_localization/image_{0,1}` / `camera_info_{0,1}`。
VGL出力`visual_localization/pose`とVSLAM入力`visual_slam/initial_pose`をpose_hintに接続。
VSLAM`visual_slam/trigger_hint`をVGLのtrigger topicへ接続。両ノードの`/diagnostics`を分離。
未使用VSLAM APIは[一覧](../mapping/README.md)と公式API参照。可視化出力はこの設定で無効。

| サービス提供ノード | 名前 | 型 | 意味 |
|---|---|---|---|
| VGL | /visual_localization/trigger_localization | std_srvs/srv/Trigger | 探索受付。完了を待たない |

## 全ROSパラメータ

起動時設定。実行中のYAML変更は反映しない。VSLAMは[mappingの全表](../mapping/README.md)と同じ
意味・値を基準に、以下を変更。QoS helperで追加されるdepthも明示して計60独自parameter。

| VSLAM parameter | 既定値 | 意味・差分 |
|---|---|---|
| use_sim_time | false | 実時刻 |
| base_frame | base_link | 車体姿勢、取付TF必須 |
| load_map_folder_path | 空→map_dir/cuvslam_map | 指定bundleを読む |
| save_map_folder_path | 空 | 元地図へ保存しない |
| enable_slam_visualization / enable_landmarks_view / enable_observations_view | false / false / false | 可視化処理無効 |
| path_max_size | 2048 | 軌跡保持数 |
| enable_request_hint | true | hint失敗時の外部再要求 |
| image_qos_depth / imu_qos_depth | 5 / 5 | 明示キュー長。IMU未使用 |

VGLの全設定:

| Parameter | 既定値 | 意味 |
|---|---|---|
| use_sim_time | false | ROS clock |
| map_dir / config_dir / model_dir | 空→明示bundle・モデル | 必須資産。launchが設定 |
| debug_dir / debug_map_raw_dir | 空 / 空 | debug保存/元画像。無効 |
| num_cameras / stereo_localizer_cam_ids | 2 / '0,1' | 左右2眼 |
| enable_rectify_images / publish_rectified_images | false / false | 入力はrectified、再出力しない |
| enable_continuous_localization | false | 成功後停止 |
| use_initial_guess | false | TFの初期推定に依存しないglobal検索 |
| image_qos_profile / input_qos / input_qos_depth | SENSOR_DATA / SENSOR_DATA / 5 | 入力QoS |
| image_sync_match_threshold_ms / image_buffer_size | 5.0 / 10 | ソフト同期許容ms・buffer。センサ精度の保証ではない |
| input_image_topic_name / input_camera_info_topic_name | visual_localization/image / visual_localization/camera_info | カメラ番号suffixを付ける上流名 |
| camera_optical_frames | camera_infra1_optical_frame, camera_infra2_optical_frame | 左右光学frame |
| base_frame / map_frame / odom_frame | base_link / map / odom | VSLAMと一致必須 |
| publish_map_to_base_tf / invert_map_to_base_tf / publish_map_to_odom_tf | false / false / false | VGLからTFを出さない |
| use_tf_transforms / use_topic_transforms | true / false | TFから外部姿勢取得 |
| verbose_logging / vgl_enable_debug / init_glog / glog_v | false / false / false / 0 | log・debug |
| localization_precision_level | 2 | 上流localizer精度設定 |
| vgl_frequency | 1.0 | 探索中の最大実行頻度Hz。カメラFPSとは別 |

`workflow.json`はROS parameterではない。表のtopic remapの正本。
container名・所有権・executorは`containers.json`に分離する。
`preparation.json`は地図生成CLI専用。`vgl_profile.json`は生成物の整合検査情報。
上流ノード以外の隠れたlaunch設定は重ねない。

## 注意点・検証範囲

公式は環境全体を網羅した地図、地図生成時の軌跡から約1 m以内での利用を推奨する。
画像入力は最低5 Hz、ステレオ同期は±10 µs、複数stereo間は±100 µsという要件がある。
ソフトウェア側の同期許容5 msでこの要件を満たしたと判断しない。
照明・視点・速度・motion blur・特徴不足による失敗があり、公式x86評価値は
Jetson/424×240/走行中の性能保証ではない。移動中の再局在は実bagで評価する。

Portable testは地図/ONNX/設定の不一致、TF競合、生成失敗のcleanup、既存出力保護、
pose-only設定を検査する。公式処理はmockであり実地図・GPU推論の検証ではない。
ROS colcon build、モデルexport、実GPU map生成、実bag/実車の精度・負荷は未確認。

## launch記述の標準化

Isaac ROS公式`visual_global_localization.launch.py`と`localize_realsense.launch.py`に合わせ、
`isaac_ros_launch_utils`のArgumentContainer → callback → component_container →
load_composable_nodesという構成に統一した。ノード定義は各ComposableNodeを明示する。
資産検証はcallbackで行うので、引数一覧表示だけではGPU/地図を必要としない。

```bash
ros2 launch kart_bringup localization.launch.py --show-args
colcon test --packages-select kart_bringup
colcon test-result --verbose
```

全8引数を`cli=True`で宣言。空値と明示falseを区別する。
containerが終了した際は公式helperの`on_exit=Shutdown()`により、このlocalization launch全体も終了する。
別launchの車両プロセスまで停止させる仕組みではない。

依存`isaac_ros_launch_utils`をpackage.xmlに明示。
Dockerfile.kartには既に`ros-lyrical-isaac-ros-launch-utils`が含まれるため追加Docker変更なし。
参照helper: [公式core.py](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/0bf10a87fafd4e88d8e3f3a4c6b82ecf68aefeb2/isaac_ros_launch_utils/isaac_ros_launch_utils/core.py)。

`test_localization_launch.py`は実際のROS/Isaac launch APIで引数公開・false上書き・
2containerへの分離読込み・資産不正時の起動阻止を検査する。
ROSのない環境ではskipし、疑似launch実装で合格扱いにしない。

## module分割とcontainer所有権

```text
launch/
├── localization.launch.py             # 資産/TF整合検証、container所有と配置
└── modules/localization/
    ├── vslam.launch.py                # 既存containerへVSLAMをloadするだけ
    └── vgl.launch.py                  # 既存containerへVGLをloadするだけ
```

最上位の検証済みparameter/remapをJSONのlaunch引数として下位へ渡す。
moduleの引数は`container_name`、`parameters_json`、`remappings_json`（全て必須）。
module自身はcontainerやセンサ、別moduleを起動しない。
moduleを直接includeする呼出側は資産・frame・TF配信者の整合検証を担当する。
通常の入口は`localization.launch.py`とし、JSONを利用者に手入力させる運用は想定しない。
includeは`scoped=True, forwarding=False`で行い、同名引数の漏出・暗黙継承を防ぐ。

追加の実行時引数（全て空文字が既定、未指定ならcontainers.json）:

| 引数 | configの既定値 | 意味 |
|---|---|---|
| vslam_container | kart_vslam_container | VSLAMのload先 |
| vgl_container | kart_vgl_container | VGLのload先 |
| create_vslam_container | true | 最上位launchがVSLAM用containerを作成するか |
| create_vgl_container | true | 最上位launchがVGL用containerを作成するか |

`containers.json`はvslam/vglそれぞれにname/create/typeを持つ。
typeは既定multithreaded、isolated_multithreadedも可。VSLAMで画像callbackと
localize callbackを並行実行するためsingle-threadは受け付けない。
外部containerのexecutorはこのlaunchでは検査できないため、所有側でmultithreadにする。
自分で作成するcontainerはroot namespaceのノード名。既存containerには
`/robot/perception`等の完全修飾名を指定できる。

既存containerに2ノードをloadする例:

```bash
ros2 launch kart_bringup localization.launch.py \
  map_dir:=/data/localization/course-v1 model_dir:=/data/models/aliked-424 \
  vslam_container:=/perception vgl_container:=/perception \
  create_vslam_container:=false create_vgl_container:=false
```

最上位で共通containerを1つ作成する場合は、両container名を`perception`にし、
createフラグは既定trueを保持する。同名containerは1回だけ作成する。
同じload先にtrue/falseや異なるexecutor指定が混在する場合は起動前にエラー。
VSLAM用だけ作成し、VGL用は外部へloadする構成も可能。

所有するcontainerはlaunch終了時に停止する。一方、外部containerは停止しない。
外部containerへload済みのComponentは、launch終了だけでは自動unloadしない。
再起動時の二重loadを避けるため所有側で管理し、必要なら`ros2 component list`でIDを確認して
`ros2 component unload /perception ID`を実行する。既存同名Componentの自動置換は行わない。
LoadComposableNodesがcontainerのload serviceを待つため、固定秒数のsleepは設けない。


## Notebookの可視化上書き

localization.launch.pyのvisualizeは空が既定（YAML保持）。trueを明示すると
enable_slam_visualization / enable_landmarks_view / enable_observations_viewを有効化する。
evaluation.launch.pyはこの上書きとuse_sim_time=trueを使用する。
live missionはvisualizeを上書きしない。driveでは実センサとVSLAMを同じcontainerへloadし、
VGLのcontainerだけlocalizationが作成する。
bag/RViz構成は[../evaluation/README.md](../evaluation/README.md)参照。
