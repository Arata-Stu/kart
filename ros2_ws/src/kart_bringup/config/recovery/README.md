# 復帰モジュール運用設定

`launch.yaml`の`enabled: false`が既定。`true`にするとruleのspeed_controllerと
RGB/EVS E2Eのdrive有効decoderを`/recovery/input_cmd`へremapし、
別processの`kart_recovery/recovery`を一つ起動する。観測専用E2Eは復帰nodeを起動しない。
`mission.launch.py mode:=drive/e2e`、`tracking.launch.py`、`e2e.launch.py`、
`evs_e2e.launch.py`の各経路で適用。これらAUTO producerは同時に起動しない。

`recovery.yaml`は全node parameterの実機正本。全て起動時固定で、STOPして再起動後に適用する。
既定値・意味・topic・調整手順は[kart_recovery README](../../../kart_recovery/README.md)を参照。
値は未校正候補で、後退時間は指令時間、後退出力は正規化要求である。

| profile | image_topicの設定 | 動作 |
|---|---|---|
| infra_imu（既定） | /realsense/infra1/image_rect_raw | 片側IR＋IMU。左右IRのステレオ演算は不要 |
| rgb_imu | /realsense/color/image_raw | RGB＋IMU。profileとimage_topicを両方変更 |
| evs_assisted | 上記のいずれか | 画像＋IMUに/event_camera/event_imageの活動vetoを追加 |

EVS併用時はmissionのenable_evsもtrueとし、既存event_image（red_blue_white、空windowも出版）を使用する。
EVS途絶は黙って軽量へfallbackせずSTOP。未使用にする場合はprofileを変更し再起動する。
E2EをEVSモデルで動かすことと、復帰検出にEVSを併用することは独立。
センサ停止/特徴不足で復帰の代わりにSTOPするため、使用画像とIMUは有効化しておく。

sim_driveはこのenabledを共用するが、node設定は`../sim/drive_recovery.yaml`を読み込む。
こちらはuse_sim_time=true、steady受信期限wall_timeout_s=5.0。実機へ流用しない。
simでもカメラ/IMUを必要とし、真値は復帰入力に使わない。
標準bagには/recovery/input_cmd、/recovery/active、/recovery/statusを含める。
全parameterと採用span/操舵はstatus JSONに記録される。


通常は`bash scripts/bringup.sh`のdrive/e2eにある「壁接触・スタック自動復帰」で選択できる。
CLIは`--recovery off|infra_imu|rgb_imu|evs_assisted`。未指定はYAML保持、指定時は今回だけ上書きする。
launch引数名は`recovery`（既定空文字）。明示profile選択ではsensor_profileとimage_topicだけを変更し、
image_topicはlaunch.yamlのprofilesを使う。後退時間・出力・操舵履歴等の調整値は保持する。
EVS補助では`--evs`も指定し、使用する画像のFPSを0にしない。


動的調整を有効にしたdriveでは、Map Studioの「パラメータ調整」→「復帰モジュール」で
core Settings（履歴・閾値・停止確認・ブレーキ／後退・操舵・回数等）をSTOP中に変更できる。
適用時に復帰履歴／phaseをresetする。sensor/topic・画像処理・timeoutは起動時固定。
静的YAMLの全項目・既定値は維持し、UI変更はメモリ上だけで次回起動には保存しない。

後退後の正パルスは復帰moduleのpost_reverse_pulse_throttle / post_reverse_pulse_sで調整する（既定0.08 / 0.05秒）。
後退後の制動・停車確認にはbrake_command、brake_min_s、stop_confirm_s、stop_timeout_sを共用する。
