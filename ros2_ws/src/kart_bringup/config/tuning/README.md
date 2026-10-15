# 手動の車両特性記録

`tuning_collect.launch.py`用の`cuvslam.yaml`はIsaac ROS release-5.0の全60独自parameterと
`use_sim_time`を明示する。公式ソースcommit・対応APT・確認範囲はYAML冒頭参照。
インストール済みAPTの厳密な版は実機未確認。parameterの意味は
[localizationの設定説明](../localization/README.md)とYAMLを参照。

localization用設定からの主な変更:

| parameter | 値 | 理由 |
|---|---|---|
| use_sim_time | false | 実時間の実機記録 |
| tracking_mode | 0 | 既定stereo VO、明示引数1でVIO |
| multicam_mode | 1 | Performance、明示引数0/2で変更 |
| base_frame | base_link | 車体前方の速度を得る。固定取付TF必須 |
| enable_localization_n_mapping | false | 局所odometryのみ |
| load_map_folder_path / save_map_folder_path | 空 | 保存地図不要、地図保存なし |
| localize_on_startup | false | 保存地図での再局在なし |
| publish_map_to_odom_tf / publish_odom_to_base_tf | false / false | 解析はtwistと固定TFを使う |
| override_publishing_stamp | false | 元観測時刻を保持 |
| enable_request_hint | false | VGLなし |
| enable_slam_visualization / enable_observations_view / enable_landmarks_view | 全てfalse | 記録中の追加可視化を抑える |

カメラは`sensors/realsense.yaml`、vehicleと録画は既存設定を使用。
sensor containerはmissionが所有し、追加cuVSLAM moduleはloadのみ。
実効parameterとtopic remapは起動ログへ出力する。
画像・CameraInfo・IMUのtopicは`localization/workflow.json`と共通。

記録・解析・設定候補の手順と全解析設定は
[tools/vehicle_tuning](../../../../../tools/vehicle_tuning/README.md)。
