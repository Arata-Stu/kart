# EVS / SilkyEvCam導入

2026-10-10。JetPack 7.2 / Isaac ROS 5.0 / ROS 2 Lyrical対象。
JetPilotのDockerfile.silky_evcamを参考にOpenEB 5.2.0を/usr/localへビルド。
Python bindingsとE2V/PyTorchは導入しない（ROS C++取得/Tensor生成用）。
公式SDK: https://github.com/prophesee-ai/openeb/tree/5.2.0

## Linuxホスト

CenturyArks公式プラグインソースを以下へ配置する。JetPilotの配置先にはREADMEのみ。
ソース本体は別配布のため自動取得・Git登録しない。

```
docker/silky_evcam_plugin_source/hal/
docker/silky_evcam_plugin_source/hal_psee_plugins/
docker/silky_evcam_plugin_source/licensing/
```

```bash
cd /path/to/kart
bash scripts/repos.sh import
./scripts/dev.sh --evs --build-local
```

以後は`./scripts/dev.sh`だけで同じEVS image/containerを選ぶ。
明示選択をcheckout内の`.kart-dev-profile`へ保存する（Git対象外）。
EVSなしへ戻す場合は`./scripts/dev.sh --no-evs`。選択はCLI呼出し前に保存するため、
Dockerビルドが失敗した場合も再実行は同じprofileを使用する。
pluginソースの存在確認はEVSイメージをビルドする時のみ行う。
通常devはOpenEBレイヤーを含まないため、全体ビルドでは
`--packages-skip openeb_ros2`を指定する。
packages.reposはopeneb_ros2 commit f89015ba1f05d2fe432b270e73e132351c1a9377を固定。
プラグインソースの版・カメラfirmwareとの互換性は配布元で確認する。
USBアクセスは既存CLIの/devマウントを利用。ホスト側の認識と権限をlsusbで確認する。

## EVSコンテナ内

```bash
cd /workspaces
bash scripts/build.sh --packages-up-to kart_bringup openeb_ros2
source /workspaces/ros2_ws/install/setup.bash
ros2 launch kart_bringup mission.launch.py mode:=collect enable_evs:=true
```

RealSenseと手動vehicleも起動するcollect構成。カメラだけ確認する場合:

```bash
ros2 launch kart_bringup modules/sensors/openeb.launch.py
```

カメラだけの起動ではbag managerなしなのでRAWは自動開始しない。
可視化はrqt/foxglove等で/event_camera/event_imageを購読する。
Tensorは/event_camera/tensor、既定cuda_async・FP32 [1,20,120,212]、250 Hz。
運用設定はkart_bringup/config/sensors/openeb*.yaml。
CPU/cuda選択はopeneb_tensor_pipeline.yamlのtensor_backendを変更し再起動する。
packet出版ON/OFFは:

```bash
ros2 param set /event_camera/event_camera_driver packet_publish_enabled true
ros2 param set /event_camera/event_camera_driver packet_publish_enabled false
```

missionのL1 START/R1 STOPでrosbagとnative RAWを同じsession dirへ記録する。
RAW状態は/bag/raw_diagnosticsを確認し、rosbagだけの成功と区別する。
RAW保存先とrosbag保存先は同じコンテナfilesystem上で解決されること。
Tensorは既定bag対象に含めない。250 Hz全量保存は約509 MB/s（FP32 payloadのみ）。
EVS単独モデルの学習データ時刻契約とTensorRT→制御の接続は未実装。
RGB e2e missionはEVSモデルへ自動切替しない。

## 検証範囲

固定commitの公開確認、local checkout、shell/Python構文、設定のparameter一致を確認。
Docker image build、Lyrical APT取得、colcon、camera、CUDA、RAW/MCAP同時記録は未確認。
このmacOS環境には対象ROS/CUDA環境とプラグインソースがない。
Jetsonでimage build→colcon→カメラ画像/Tensor→RAW/MCAP記録・再生の順に確認する。

## Bias調整とbringup選択

プラグイン配置先の親ディレクトリのみ用意する。ダウンロードした公式ソースに含まれる
hal/・hal_psee_plugins/・licensing/をそのまま親ディレクトリへ移動する。
未配置のままdev --evsを実行すると不足を表示する。

Bias配置先は`bias/evs/`。調整ツールはJetPilotから移植したC++/OpenCV GUI。
同じカメラを使うdriverを停止し、EVSコンテナ内で:

```bash
bash scripts/sensors/evs-bias.sh --build
# 保存されたcustom.biasを運用名へコピー
cp bias/evs/silkyevcam_custom.bias bias/evs/daylight.bias
bash scripts/bringup.sh
```

TUIでEVS有効→decoder→biasを選択する。非対話の場合:

```bash
bash scripts/bringup.sh --mode collect --evs --evs-backend cuda_async \
  --evs-bias-file /workspaces/bias/evs/daylight.bias --no-bridge --dry-run
```

--dry-runを外すと起動。--evs-serial SERIALでカメラを選択。
--bias-root DIRでbiasの一覧探索先を変更。カメラ既定は--evs-bias-file @default。
packet topicは出版せず、direct Tensorと赤青白画像を使う。
Bias内容のSDK互換性、チューナーGUI、調整値の実機適用は未確認。

## 学習用時刻保存

OpenEB用bag profileに/event_camera/tensor_timingを追加済み。
Tensorの窓とセンサ→ROS offsetを250 Hzの軽量JSONとして記録する。
各RAWを閉じた後、timestamp shiftを取得して同dirに保存する:

```bash
bash scripts/sensors/evs-raw-timing.sh --build /workspaces/record/session/camera.raw \
  > /workspaces/record/session/camera.raw.time_shift.json
```

この出力ファイルはコマンド実行前にshellが作るため、失敗時は空のJSONを利用しないこと。
式は`ros_ns=(raw_relative_us+raw_sensor_shift_us)*1000+sensor_to_ros_offset_ns`。
学習readerのtime shifting設定を合わせる。推定offsetはUSB遅延を含み同期精度は未検証。
openeb_ros2のこの追加はローカル変更。別環境へvcs取得するにはcommit/pushとpackages.repos更新が必要。
