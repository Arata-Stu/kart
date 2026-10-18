# 手動の車両特性記録

`launch.json`の`fixed_command`は既定false。明示`fixed_command:=true`で固定AUTO指令を追加する。
値の正本は`fixed_command.yaml`。全parameterの既定値・意味は
[kart_control](../../../kart_control/README.md#固定指令による車両特性試験)参照。
`fixed_axes/steering/throttle/brake/duration_s`だけ明示launch引数で上書きできる。
数値指令とduration_sの既定値は全て0、max_throttle=0.2、control_rate_hz=50、mode_timeout_s=0.3、entry_neutral_s=0.3。
手動でのSTOP→AUTOにより開始し、録画STARTは独立。MANUALのJoy操作は維持する。

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

### 固定軸の対話切替

`tuning_collect.launch.py fixed_command:=true fixed_axes:=throttle`でスロットルのみ固定できる。
`fixed_axes`の既定は`both`、他に`steering`。非固定軸はJoy操作。
別端末で`bash scripts/vehicle_tuning.sh control`を実行し、STOP中に固定対象・
steering/throttle/brake/duration_sを一括変更する。再起動や録画停止は不要。
AUTOへの移行と録画はJoyで操作する。変更はYAMLへ保存しない。
`teleop_timeout_s`の既定は0.15秒、片軸固定の入力途絶でSTOP要求。
条件は`/control/fixed_config`（std_msgs/msg/String、JSON、1Hzと変更時）へ記録する。
詳細と全parameterは[kart_control README](../../../kart_control/README.md)。
