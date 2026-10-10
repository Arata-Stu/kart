# kart_mapping

Notebook側でNVIDIA公式 `isaac_mapping_ros/create_map_offline.py` を実行し、保存済みcuVSLAM地図を作る。
HDMap用点群は地図生成とは別の`capture_snapshot`工程で取得する。

## 起動・責務

`ament_cmake_auto` + `ament_cmake_python`。実行ファイルは`build_map`。

```bash
colcon build --symlink-install --packages-up-to kart_mapping
source install/setup.bash
ROS_DOMAIN_ID=92 ros2 run kart_mapping build_map --job /absolute/path/job.json
```

通常はMap Studioが `bag`（絶対パス）、`output`（一時出力フォルダ）、`workflow`を含むJSONを渡す。
`workflow`の正本は `kart_bringup/config/mapping/workflow.json`。
公式CLIを `--steps_to_run edex compute_poses --use_raw_image=False` で呼び出す。
cuSFM・深度・占有地図・cuVGLは実行しない。独自ROSノード／publisher／subscriber／serviceは起動しない。
bag検査はrosbag readerで行い、ROSトピック再生はしない。Collectorとmapping.launch.pyは後続のcapture専用で、地図生成では使わない。

## 設定と入力

| 設定 | 既定値・意味 |
|---|---|
| ros_domain_id | 92。環境変数と一致させる。UI内で排他 |
| left_image / right_image | `/realsense/infra1/image_rect_raw` / `/realsense/infra2/image_rect_raw`。`sensor_msgs/msg/Image`、rectified入力 |
| left_info / right_info | `/realsense/infra1/camera_info` / `/realsense/infra2/camera_info`。`sensor_msgs/msg/CameraInfo` |
| tf_static | `/tf_static`固定。`tf2_msgs/msg/TFMessage`、基準から左右光学frameへの外部姿勢 |
| overrides.base_frame | `camera_link`。公式のbase_link_nameに渡す |
| overrides.camera_optical_frames | `[camera_infra1_optical_frame, camera_infra2_optical_frame]` |
| overrides.tracking_mode | 0のみ。今回はStereo。VIOは受け付けない |

起動後の設定変更なし。再生速度・固定drain秒数・save_map待機設定は公式経路では使用しないため削除。
カメラ設定はジョブごとに`camera_topics.yaml`として保存し、公式CLIへ渡す。
cuVSLAM内部の設定はインストールされた公式map_creation_configの既定値を利用する。

## 成果物・成功条件

公式コマンドの終了コード0、`latest`等のリンクを実体パスで重複除去して出力ディレクトリが一つ、`cuvslam_map/*.mdb`が非空であることを確認する。
`cuvslam_map/`を地図直下へ配置し、軌跡・中間データ・ログは`official/`配下に保持。
`mapping_result.json`に実行引数・APT版・bag metadata hash・点群未取得状態を保存する。
UIは`map.json`を`revision=1, snapshot_status=pending`で作成する。空のsnapshotを成功成果物として作らない。
公式CLIは`--print_mode=all`で詳細ログを出力する。出力地図のファイル名・サイズもログに記録する。
UI実行の失敗時は診断用に`map/.failed/<ID>/`へ中間成果物・公式ログを保持し、地図一覧には公開しない。中止時は一時領域を削除する。
診断後の`.failed`内データは手動削除可能（自動削除しないため容量に注意）。

点群取得の後続工程では、保存地図の読み込みと自己位置合わせを確認してからlandmarksを採用する。
終了時の最新TF採用・map座標の二重変換防止はその工程の要件として維持する。

## 検証範囲

公式CLI引数・成果物検査・点群未取得状態はROSなしのテストで検証する。
Linuxのcolcon build、インストール済み公式5.0 CLIとの互換性、実bag/GPUでの地図生成は未検証。
公式仕様: https://nvidia-isaac-ros.github.io/concepts/visual_global_localization/tutorials/tutorial_map_creation.html

## 保存地図からHDMap用点群を取得

`capture_snapshot --job <capture_job.json>` を追加。Studioの「保存地図から点群を取得」から起動する。
実GPU検証は未実施。
対象は点群未取得の地図のみ。地図生成時とmetadata SHA256が一致するbagを選択する。
metadata一致はbag全メッセージの同一性の証明ではないため、地図生成に使用したbagを保持する。
既存HDMapの座標を変えないため、snapshot取得済み地図への再取得は拒否する。

手順:

1. 元の`cuvslam_map`を一時領域へコピー。cuVSLAMにはコピーを渡し、元地図は保存し直さない。
2. `mapping.launch.py`が可視化ありの`visual_slam`を起動。別プロセスの`kart_map_collector`を先に購読開始。
3. bagの入力型・静的TFを検査し、左右画像の末尾header stampを調べる。
4. 入力購読とlocalize service準備後、センサー入力だけを再生。記録済み動的TF・自己位置・車両指令は混ぜない。
5. 最初のVSLAM診断が届いたら原点Poseをhintに`LocalizeInMap`を1回要求。
   元bagの開始位置付近を想定する。任意地点からの大域自己位置推定ではなく、VGLはまだ使わない。
6. serviceのsuccessは受理に過ぎない。診断の`localized_in_exist_map=Yes`かつ`vo_status=OK`で成功確認。
   成功前や追跡喪失前の点群・TFを破棄し、新しい成功区間のデータだけ採用。軌跡も成功前のposeを除外する。
7. 再生終了後、診断・点群・軌跡・map→odomが入力末尾の許容範囲に到達し、更新が静まるのを待つ。
   未localize、末尾未到達、TF不足、更新待ちtimeoutは失敗。失敗時に編集可能へ進めない。
8. 最新TFを一度だけ適用（既にmap座標なら適用しない）してsnapshot/cloudを保存し、map.jsonを最後にreadyへ更新。

### 追加ノード・IF

`kart_map_collector`の入力（root namespace固定）:

| topic | 型 | QoS | 用途 |
|---|---|---|---|
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | best effort / volatile / depth 5 | hardware_id=`visual_slam`の成功・追跡状態 |
| `/tf` | `tf2_msgs/msg/TFMessage` | best effort / volatile / depth 5 | 最新map→odom |
| `/visual_slam/vis/landmarks_cloud` | `sensor_msgs/msg/PointCloud2` | best effort / volatile / depth 5 | 成功区間の最後の非空点群 |
| `/visual_slam/tracking/slam_path` | `nav_msgs/msg/Path` | best effort / volatile / depth 5 | 成功区間の軌跡 |
| `/clock` | `rosgraph_msgs/msg/Clock` | ROS clock既定 | use_sim_time=true |

`/visual_slam/localize_in_map`のclient、型`isaac_ros_visual_slam_interfaces/srv/LocalizeInMap`。
要求map_folder_pathは作業コピー、pose_hintは原点・単位quaternion。独自publisherなし。
ROS標準のrosout／parameter IFは省略。Collector独自ROS parameterなし。
cuVSLAMの全parameterはbringup `config/mapping/capture_cuvslam.yaml`、
実行時にload_map_folder_pathと元jobのframe設定のみ上書きする。

### captureジョブ設定

bringup `config/mapping/capture.json`が正本。ROS parameterではなくworkerの起動時固定設定。

| 項目 | 既定値 | 意味 |
|---|---|---|
| replay_rate | 1.0 | bag再生倍率 |
| ready_timeout_s | 120.0 | service・購読準備待ち秒数。再生timeoutの余裕にも使用 |
| drain_timeout_s | 30.0 | 再生終了後の最大待機秒数 |
| settle_s | 3.0 | 点群・軌跡・TFの更新停止確認秒数。同じstamp・内容の再送は更新と数えない |
| tail_tolerance_s | 0.5 | 左右画像の末尾stampの小さい方に対する許容遅れ秒数 |

入力末尾確認は途中の全画像処理を保証しない。低速再生でもDDSや追跡の欠落はあり得る。
可視化topicは全ランドマークの完全exportではなく、最新の非空表示点群を保存する。
ROS 5.0上流は表示点群のバッファに上限を持つため、実コースの点群範囲は実データで確認する。
検証: 成功ゲート・古い診断・追跡喪失・末尾不足のportableテスト、アプリのready遷移と元地図保護の模擬テスト。
実bag／GPU／localize精度／ROS結合は未確認。
