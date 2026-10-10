# kart_e2e — DINOv3学習とIsaac ROS TensorRT推論

走行時は **公式GPU画像encoder → 公式TensorRTNode → kartのC++ control decoder**。
PyTorch ROS推論ノードは削除した。PyTorchはPCでの学習・ONNX exportだけに使用する。
TensorRT/encoderはIsaac ROS release-5.0 APIを対象とし、旧NITROS TensorListを使用しない。

```text
/realsense/color/image_raw + /realsense/color/camera_info
  → ResizeNode → ImageFormatConverterNode (RGB)
  → ImageToTensorNode → ImageTensorNormalizeNode
  → InterleavedToPlanarNode → ReshapeNode
  → /e2e/tensor_input [1,3,120,212] FP32
  → TensorRTNode (ONNX内でpadding → DINOv3 → head)
  → /e2e/tensor_output [1,1] or [1,2] FP32
  → kart_e2e::ControlDecoderNode
  → /e2e/control_cmd (既定) または /auto/control_cmd
  → 既存command_mux → vehicle bridge
```

## 出力モード

|学習mode|モデル出力|推論時のスロットル|
|---|---|---|
|steer_throttle|steering [-1,1]（左正）, throttle [0,1]|予測値、max_throttleで上限制限|
|steer_only|steering [-1,1]のみ|fixed_throttle、AUTO中の有効結果時だけ|

modeはcheckpoint → ONNX metadata → decoderへ引き継ぎ、出力数と照合する。
brake/reverseは0。固定スロットルは固定車速ではない。E2Eは速度PIDを経由しない。

## Package構成とROSノード

- `kart_e2e::ControlDecoderNode`: C++ Component、ノード名`e2e_control_decoder`。
  単体実行ファイル`e2e_control_decoder`も提供（MultiThreadedExecutor）。
- `model.py/train.py`: PCでのPyTorch学習。DINOv3ソースは公式固定commitを`e2e.repos`で取得。
- `export_onnx.py`: オフラインexportとONNX Runtime数値検証。実行ファイル`e2e_export_onnx`。
- `preprocess_bag.py`: rosbag教師抽出。実行ファイル`e2e_preprocess_bag`。
- `control_gate.hpp`: ROS非依存のAUTO・鮮度・出力制限。
- `contract.py`: TorchをimportしないONNX metadata/hash検査、launchでも使用。

全8 Componentを同じmultithreaded containerへloadし、intra-processを有効化する。
GPU bufferはIsaac ROS 5の`rosidl::Buffer`/`cuda_buffer`で引き継ぎ、decoderで
1～2個のFP32値だけをCPUへ読み戻す。CUDA完了を待つcallbackとwatchdogは別group。
元画像timestampを保持し、TFは使用／発行しない。

公式出典・全外部parameterは[bringup設定README](../kart_bringup/config/e2e/README.md)。
使用encoderは公式`isaac_ros_dnn_image_encoder` launchと同じComponent群から構成し、
不要なcropは省く。release-5.0には単独の`DnnImageEncoderNode` pluginがない。

## 推論の起動

Docker内の`/workspaces`を想定。既存Dockerfileはimage encoder/TensorRT aptを導入する。
decoderのビルドにはROS Lyrical、CUDA toolkit、isaac_ros_tensor_msgs、cuda_bufferが必要。
Jetsonの推論時にTorch、ONNX Runtime、DINOv3 Pythonソース、checkpointは不要。

```bash
cd /workspaces/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to kart_bringup
source install/setup.bash
ros2 launch kart_bringup e2e.launch.py model_dir:=/workspaces/models/e2e_v1
```

`model_dir`はexportが作る`model.onnx`と`metadata.json`を含むdirectory。
TensorRT engineは実行先GPUで作成する。既定`force_engine_update=true`で再構築するため、
初回／起動時に時間を要する。同じGPU・TensorRT・モデルで再利用するときだけ
`force_engine_update:=false`を指定する。engineはGPU/TensorRT間で持ち回らない。
モデルhashをengine名へ含めるが、GPU・TensorRT互換性まではhashだけでは保証しない。

既存containerへ追加する場合:

```bash
ros2 launch kart_bringup e2e.launch.py \
  model_dir:=/workspaces/models/e2e_v1 \
  create_container:=false container_name:=/shared_container
```

外部containerはmultithreadedが必要。module launchはload専用。
外部containerにloadしたnodeはlaunch終了だけではunloadされない。切替時は明示unloadする。
このlaunchはカメラ、車両、localization、jtopを重複起動しない。
ResizeNodeはImageとCameraInfoの**同一stamp**を要求する。
RGB/BGRの8bit画像を想定し、YUV・16bit等は別の前処理契約として扱う。

steer-onlyの固定スロットルは`fixed_throttle:=0.10`などで指定。
既定`drive_enabled=false`は確認用topicへ出力し車両に接続しない。
`drive_enabled:=true`はAUTO muxへ接続するため車両出力を伴う。
rule-basedのtracking/speed_controllerと同時起動しない。STOP中に旧制御を終了／unloadして切り替える。
中央source selectorによる動的なrule/E2E切替は未実装。

## Control decoderの入出力

root namespace時の既定。相対topicはnamespace/remapに従う。
全てvolatile。標準`/rosout`・`/parameter_events`は省略。独自service/actionなし。

|方向|topic|型|QoS|用途|
|---|---|---|---|---|
|入力|`/e2e/tensor_output`|isaac_ros_tensor_msgs/msg/TensorList|reliable / depth 1|controlという名前の連続FP32 [1,N]、N=1または2|
|入力|`/operation_mode/state`|kart_interfaces/msg/OperationModeState|reliable / depth 1|mode・元stamp・受信時刻|
|出力|`/e2e/control_cmd`|kart_interfaces/msg/ControlCommand|reliable / depth 1|既定、50Hzの制御gate出力|
|条件付き出力|`/auto/control_cmd`|kart_interfaces/msg/ControlCommand|reliable / depth 1|drive_enabled=true時に上記を置換|
|出力|`/e2e/status`|std_msgs/msg/String|reliable / depth 1|状態、最大10Hz|
|条件付き出力|`/operation_mode/request`|kart_interfaces/msg/OperationModeRequest|reliable / depth 10|drive有効時の異常STOP要求、最大10Hz|

50Hzはwatchdog/指令周期であり推論FPSではない。active指令には画像のstampをそのまま使用。
AUTO進入100msは中立。その後の期限切れ・不正値・mode途絶・競合AUTO publisher検出では
中立＋STOP要求をラッチする。新鮮なSTOP/MANUAL/PROPOを経て復帰する。
競合検出は起動順の競合を完全に防ぐarbiterではない。複数AUTO producerを起動しない。
中立指令は物理停止を保証しない。MANUAL/STOP中は確認用topicも中立。

### 全decoder parameter

全独自parameterは起動時固定。ROS共通`use_sim_time`以外の動的変更は不可。
pkg基準値は`config/control_decoder.yaml`、実運用はbringupの同名YAML。

|parameter|既定値|意味|
|---|---|---|
|use_sim_time|false|ROS共通、bag再生時の時計|
|output_mode|steer_throttle|launchではmetadataから解決。直接起動時は要整合|
|output_tensor_name|control|TensorListの名前|
|fixed_throttle|0.0|steer-only値[0,max_throttle]|
|max_throttle|0.2|スロットル上限[0,1]|
|input_timeout_s|0.15|撮影から出力までの期限秒。decoder受信wall鮮度も確認|
|mode_timeout_s|0.3|modeのstamp・受信wall鮮度上限秒|
|control_rate_hz|50.0|watchdog/指令周期[1,100]Hz|
|drive_enabled|false|AUTO muxへの接続|

## 学習・ONNX export（PCのみ）

[python_ws/e2e](../../../python_ws/e2e/README.md)の環境を利用する。
PyTorchは学習・変換用に残し、ROS推論には使用しない。

```bash
cd /workspaces
vcs import . < e2e.repos
ros2 run kart_e2e e2e_preprocess_bag /workspaces/record/train_run /workspaces/datasets/train_run
ros2 run kart_e2e e2e_preprocess_bag /workspaces/record/val_run /workspaces/datasets/val_run
ros2 run kart_e2e e2e_train \
  --train /workspaces/datasets/train_run --validation /workspaces/datasets/val_run \
  --repo /workspaces/python_ws/dinov3 \
  --weights /workspaces/weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth \
  --output /workspaces/runs/e2e_v1 --mode steer_throttle
ros2 run kart_e2e e2e_export_onnx /workspaces/runs/e2e_v1/best.pt /workspaces/models/e2e_v1 \
  --repo /workspaces/python_ws/dinov3
```

steer-onlyは学習を`--mode steer_only`にする。encoder既定凍結、`--finetune`で全体更新。
Torch CLIをROS外で実行する場合は`PYTHONPATH=ros2_ws/src/kart_e2e python -m kart_e2e.train ...`
または`-m kart_e2e.export_onnx ...`。rosbag抽出のみROSメッセージ環境が必要。

学習既定: epochs=20, batch-size=32, learning-rate=1e-4, seed=42, device=cuda。
train/validation/repo/weights/outputは必須。MSEと出力別MAEを保存し最良validationのbest.ptを保存。
公式重みはstrict load、取得失敗時にrandomへfallbackしない。train/validationは同一bag・同一データセットでも実行可能。
exportは固定batch=1、opset17。checkerと3入力のONNX Runtime比較が通ってからmetadataを作る。
モデルhash・前処理schema・出力modeをlaunchで照合する。出力directoryの上書きは拒否する。

### 前処理契約

標準カメラ入力は424×240。抽出PNGは元の解像度を保持する。
現在は計算量を抑えるため縦横1/2へ縮小し、モデルの固定サイズは変更しない。
RGB8 → linear resize 212×120（half-pixel、antialiasなし、8bitへ丸め）→ [0,1] →
ImageNet mean/std → NCHW。ここまでを公式GPU encoderが担当する。
正規化後の中央zero padding（左右6、上下4）をONNX内部に置き、ViT入力を224×128にする。
学習時は同じ配置・正規化とantialiasなしresizeを使用する。
GPU resizeとの丸め差は実GPUで未測定。完全bit一致を主張しない。
前のPIL bilinear（縮小時antialiasあり）からschema=2へ更新したので、schema=1 checkpointは
export対象外。再学習が必要。JetPilot checkpointの直接互換もない。

### rosbag教師抽出

既定topic/type: `/realsense/color/image_raw` (sensor_msgs/msg/Image)、
`/teleop/control_cmd` (kart_interfaces/msg/ControlCommand)、
`/operation_mode/state` (kart_interfaces/msg/OperationModeState)。
`--image-topic/--command-topic/--mode-topic`で変更可。
全topicに`--clock bag`受信時計を使うのが既定。`--clock header`は時計同期済みの場合のみ。
画像以前100ms以内の最新教師を対応（`--max-skew-ms 100`）、modeは300ms以内のMANUAL限定。
brake/reverse非0・非有限・範囲外を除外。出力はPNG、samples.jsonl、metadata.json。
RGB/BGR/RGBA/BGRA/mono8の非圧縮画像対応。反応遅れ・撮影と受信時刻の差は自動補正しない。
同一bag・同一データセットのtrain/validation共有を許可する。自動的な分割は行わない。
同じ画像を両方に使ったvalidation指標は未知データへの汎化評価にならない。

## 検証

C++ gateはROSなしでコンパイル・実行可能:

```bash
c++ -std=c++17 -Iros2_ws/src/kart_e2e/include \
  ros2_ws/src/kart_e2e/test/test_control_gate.cpp -o /tmp/test_e2e_gate
/tmp/test_e2e_gate
KART_DINOV3_REPO=/path/to/dinov3 PYTHONPATH=ros2_ws/src/kart_e2e \
  python3 -m unittest discover -s ros2_ws/src/kart_e2e/test -v
```

C++ gate、metadata・設定の単体検証、両modeのDINOv3 ONNX exportとORT出力比較を実施。
ONNX検証はrandom encoder＋合成入力による配線検証であり精度評価ではない。
ROS/CUDA環境がないためdecoderのcolcon build、GPU encoderと学習前処理の画素比較、
TensorRT engine生成・数値比較・推論レイテンシ、実bag・実機走行は未確認。


## Notebook UI

`./tools/app/start.sh`で起動し、上部「学習」を開く。
データセット作成 → 学習 → ONNX export → Jetson転送を実行できる。
最初に「学習環境」でPython・公式DINOv3ソース・重み・deviceを設定する。
[UI手順と保存先](../../../tools/app/README.md#e2e学習画面)。

公式重みの配置先は[weights/dinov3](../../../weights/dinov3/README.md)。UIの既定パスも同じ。

## 起動時のモデル設定

学習CLIの `--fixed-throttle`（既定0.0）と `--max-throttle`（既定0.2）は
0 ≤ fixed ≤ max ≤ 1の正規化値。UIにも同じ設定欄がある。学習lossは従来どおりで、
この2値は実行時設定としてrun.json・best.ptのruntimeへ保存する。
ONNX exportはmetadata.jsonのruntimeへ引き継ぐ。

bringupのe2eモードはmetadataからoutput_mode（steer_only/steer_throttle）、
固定throttle・上限をdecoderへ設定し、model_specで学習時の入力サイズ・前処理との一致を検証する。
RGB 424×240から212×120へresizeし、ImageNet正規化・224×128へのpaddingという既存契約は共通。
steer_onlyでは予測steer＋固定throttle、steer_throttleでは予測steer/throttleを使う。
起動はAUTOへの切替を行わず、decoderのmode gateが引き続き適用される。

旧checkpointにruntimeがない場合、export CLIへ `--fixed-throttle 0.1 --max-throttle 0.2`
等を明示して再exportする。推論起動時に不足値を推測しない。
単独e2e.launchのfixed_throttle明示上書きは可能だが、TUIではmetadataをそのまま使用する。
ROS node/topicの変更はない。decoderの全parameterは既存config/control_decoder.yaml参照。

## rosbagのオフライン評価（Notebook専用）

UIの「4 rosbag評価」でexport済みONNXとbagを選び、評価名を指定する。`e2e_evaluate` / `python -m kart_e2e.evaluate`はROSノードを起動せず、topicをpublishしないオフラインCLI。入力は`--model`（ONNX bundle）、`--bag`、`--output`（新規保存先）が必須。画像topicの既定は`--image-topic /realsense/color/image_raw`、教師操作は`--command-topic /teleop/control_cmd`、モードは`--mode-topic /operation_mode/state`、時計は`--clock bag`、過去ラベル許容差は`--max-skew-ms 100`。UIではデータセット作成欄の抽出設定を共用する。

学習と同じ前処理・教師対応付けでMANUALの有効ラベル付き画像を抽出し、ONNX Runtime CPUで推論する。ONNX内部にpaddingが含まれるため入力の二重paddingはしない。モデルmetadataのmode・SHA256・固定/最大スロットルを検証し、不正値や範囲外出力は失敗する。steer-onlyは固定スロットルを使用、steer+throttleは予測値へmetadataの最大スロットルを適用する。AUTO切替・watchdog等のオンライン状態機械は再現しない。

`report.json`にMAE/RMSE/P95絶対誤差（正規化操作値）、抽出件数、スキップ数、モデルSHA256、CPU推論中央値（前処理除外、warmupを含む）、最大500点の表示用サンプルを保存。`predictions.csv`には全評価フレームの時刻・教師・生出力・上限適用値・推論時間を保存する。データは`e2e/evaluations/<評価名>/`へ公開し、一時画像は終了時に削除する。学習と同じbagでの評価も許可するが、未知のコースへの性能評価ではない。運転成功率やTensorRT実機レイテンシは別途検証する。
