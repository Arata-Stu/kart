# Kart EVS Bias Tuner

JetPilot `tools/silkyevcam_bias_tuner`（2026-10-10の実装）を移植した、ROS非依存の
C++17/OpenCV GUI。カメラを直接開くためevent packet topicを使用しません。
スライダー調整・イベントレート表示・背景ノイズ自動調整・起動時biasへのResetを備えます。
kartの可視化に合わせ、ON赤／OFF青／無イベント白へ変更しました。

## 実行

EVS Docker内でOpenEB SDK、HAL、OpenCV GUI、CMakeが必要です。
GUIのDISPLAY転送は既存Isaac ROS CLIを使用し、起動環境で画面表示を確認してください。
同じカメラを使用するbringup/driverを終了してから実行します。

```bash
cd /workspaces
bash scripts/sensors/evs-bias.sh --build
# 特定カメラ・保存済みbiasから開始
bash scripts/sensors/evs-bias.sh --serial SERIAL --bias-file /workspaces/bias/evs/daylight.bias
```

wrapperの保存先は`bias/evs/`。`--output-dir DIR`で変更可能です。
実行ファイルはtools/evs_bias_tuner/build/silkyevcam_bias_tuner。
ROS packageではないためament/ROSノード・topic・parameterはありません。
直接実行時のCLI既定値はserial空（最初のカメラ）、bias-file空（カメラ現在値）、
output-dir `.`（カレントディレクトリ）。`--help`はカメラを開きません。
bias-fileを指定すると読み込んだ値をResetの基準とします。

| 操作 | 動作 |
|---|---|
| スライダー | biasを変更 |
| a | 静止したカメラ・対象で背景ノイズを測定し自動調整 |
| s | silkyevcam_custom.biasを保存 |
| r | カメラから現在値をGUIへ再取得 |
| x / Restore startup biases | 起動時の値へ戻す・自動調整中止 |
| q | 終了 |

起動時にsilkyevcam_startup.bias、自動調整成功時にsilkyevcam_autotuned.biasを保存。
同名ファイルは上書きします。使用する設定は`daylight.bias`等へ別名コピーし、
bringupのbias一覧から選択してください。自動調整が走行条件で適切かは別途検証が必要です。

```bash
bash scripts/bringup.sh --mode collect --evs \
  --evs-backend cuda_async --evs-bias-file /workspaces/bias/evs/daylight.bias \
  --no-bridge --dry-run
```

実機GUI、biasのSDK読込・保存、調整結果は未確認です。
