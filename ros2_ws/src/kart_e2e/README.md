# kart_e2e — RGB/EVS DINOv3学習とIsaac ROS TensorRT推論

RGB走行時は **公式GPU画像encoder → 公式TensorRTNode → kartのC++ control decoder**。
EVSは **OpenEB raw-count Tensor → TensorRTNode → 同じC++ decoder**。
[RAWから学習・推論までのEVS手順](#evs-rawから制御policyまで2026-10-18)を参照。
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
TensorRT engineは事前に実行先GPUで`scripts/e2e_trt.sh`を使って作成する。
既定`force_engine_update=false`。未build・不適合なら案内して起動を停止する。
bringupからのbuildは禁止し、engineはGPU/TensorRT間で持ち回らない。
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
|出力|`/e2e/status`|std_msgs/msg/String|reliable / depth 1|状態、10Hz＋状態変化時に即時|
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
|fixed_throttle|0.0|固定駆動量[0,max_throttle]。モデルmetadataを初期値として使用|
|use_fixed_throttle|false|起動時固定。trueならモデルの出力数に関係なくthrottleは固定値。missionのe2eはtrue|
|max_throttle|0.2|スロットル上限[0,1]|
|input_timeout_s|0.15|撮影から出力までの期限秒。decoder受信wall鮮度も確認|
|image_future_tolerance_s|0.005|画像stampがROS現在時刻より先になる許容秒。起動時固定、0..0.05、0で厳密判定|
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

学習既定: epochs=100, batch-size=32, learning-rate=1e-4（凍結）／1e-5（`--finetune`）, seed=42, device=cuda。
制御学習はSmooth L1（β=0.1）・Plateau scheduler ONが既定。
train/validation/repo/weights/outputは必須。MSEと出力別MAEを保存し最良validationのbest.ptを保存。
公式重みはstrict load、取得失敗時にrandomへfallbackしない。train/validationは同一bag・同一データセットでも実行可能。
exportは固定batch=1、opset17。checkerと3入力のONNX Runtime比較が通ってからmetadataを作る。
モデルhash・前処理schema・出力modeをlaunchで照合する。出力directoryの上書きは拒否する。

### 前処理契約

標準カメラ入力は424×240。抽出RGB（既定NPY、任意PNG）は元の解像度を保持する。
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
任意の`/teleop/fixed_config` (std_msgs/msg/String) は`--fixed-config-topic`で変更可。
全topicに`--clock bag`受信時計を使うのが既定。`--clock header`は時計同期済みの場合のみ。
画像以前100ms以内の最新教師を対応（`--max-skew-ms 100`）、modeは300ms以内、採用モードはbagの固定設定から自動判定する（下記）。
brake/reverse非0・非有限・範囲外を除外。出力は圧縮なしRGB NPY（既定）、samples.jsonl、metadata.json。
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
missionのe2eモードはuse_fixed_throttle=trueを明示し、どちらのモデル形式でも予測steer＋固定throttleを使う。
出力数やmetadataのoutput_modeは変更せず、steer_throttleの2成分も既存契約どおり検証する。
単独decoder／下位e2e.launchでuse_fixed_throttle=falseの場合は、従来どおりsteer_throttleの予測throttleを使用する。
起動はAUTOへの切替を行わず、decoderのmode gateが引き続き適用される。

旧checkpointにruntimeがない場合、export CLIへ `--fixed-throttle 0.1 --max-throttle 0.2`
等を明示して再exportする。推論起動時に不足値を推測しない。
CLI --e2e-fixed-throttle／missionのe2e_fixed_throttle／単独e2e.launchのfixed_throttleで固定値を上書きできる。
TUIはE2E専用の固定値入力を表示する。未指定はmetadataのruntime.fixed_throttle、上限はruntime.max_throttle。
収集用--auto-fixed-throttleはE2E CLIでは拒否する。missionはYAMLに収集用設定があってもJoy固定を-1へ上書きする。
ROS node/topicの変更はない。decoderの全parameterは既存config/control_decoder.yaml参照。

## rosbagのオフライン評価（Notebook専用）

UIの「4 rosbag評価」でexport済みONNXとbagを選び、評価名を指定する。`e2e_evaluate` / `python -m kart_e2e.evaluate`はROSノードを起動せず、topicをpublishしないオフラインCLI。入力は`--model`（ONNX bundle）、`--bag`、`--output`（新規保存先）が必須。画像topicの既定は`--image-topic /realsense/color/image_raw`、教師操作は`--command-topic /teleop/control_cmd`、モードは`--mode-topic /operation_mode/state`、時計は`--clock bag`、過去ラベル許容差は`--max-skew-ms 100`。UIではデータセット作成欄の抽出設定を共用する。

学習と同じ前処理・教師対応付けで自動判定した教師モードの有効ラベル付き画像を抽出し、ONNX RuntimeのCUDAを優先して推論する（利用不可・初期化失敗は理由をログに出してCPUへfallback）。ONNX内部にpaddingが含まれるため入力の二重paddingはしない。モデルmetadataのmode・SHA256・固定/最大スロットルを検証し、不正値や範囲外出力は失敗する。steer-onlyは固定スロットルを使用、steer+throttleは予測値へmetadataの最大スロットルを適用する。AUTO切替・watchdog等のオンライン状態機械は再現しない。

`report.json`にMAE/RMSE/P95絶対誤差（正規化操作値）、抽出件数、スキップ数、モデルSHA256、実際のprovider一覧・CUDA fallback理由・推論平均・中央値・P95・最小・最大（前処理・最初の10回のwarmupを除外）、最大500点の表示用サンプルを保存。`predictions.csv`には全評価フレームの時刻・教師・生出力・上限適用値・推論時間を保存する。データは`e2e/evaluations/<評価名>/`へ公開し、一時画像は終了時に削除する。学習と同じbagでの評価も許可するが、未知のコースへの性能評価ではない。運転成功率やTensorRT実機レイテンシは別途検証する。

### Notebook TensorRT build

UI「ONNX export」の「NotebookでTensorRT build・速度計測」は選択ONNXを`python -m kart_e2e.build_engine --model <bundle> --output <新規ディレクトリ>`で処理する。ROSノード・topicは追加しない。x86_64限定、`trtexec`はPATHまたは`/usr/src/tensorrt/bin/trtexec`から利用し、ONNXのSHA256/入出力契約を検証する。精度はFP32（既定、TF32無効）またはFP16を選択。RGB形状は固定1×3×120×212、EVSはbundleの20ch形状を使用する。`--warmUp=500 --duration=3 --exportTimes=<file>`で合成入力の計測を行い、engine・timings.json・report.jsonを`e2e/engines/<name>/`へ保存する。reportはONNX/engineバイト数、ビルド＋計測合計秒、GPU情報、利用可能なcomputeMs/latencyMs/h2dMs/d2hMs統計、再現コマンドを含む。詳細TensorRTバージョン・GPU計測ログはUIジョブログに残る。未対応オプションやGPU不足は失敗として表示し、engineを成功公開しない。

Notebook engineはJetsonへ転送しない。既存の転送機能は引き続きONNXとmetadataのみを送信する。合成入力のTensorRT計測はrosbagでの精度検証ではなく、engineの数値一致も別途必要。

## E2E TensorRT事前build

Jetsonのkartコンテナ内で`bash /workspaces/scripts/e2e_trt.sh`を実行する。`/workspaces/models`以下の転送済みONNX bundleをfzfで選択し、次にFP32/FP16を選択する。直接指定は`bash /workspaces/scripts/e2e_trt.sh /workspaces/models/<モデル名>`、探索先変更は`--models-root <dir>`。`KART_TRT_PYTHON`既定は`/opt/inference/bin/python`。ROS起動・車両指令publishは行わない。

FP32が既定。TensorRT 10では`--fp16`も指定可能、TensorRT 11以降ではこのフラグを拒否する。`--force`で再buildする。モデル配下の`model_<ONNX SHA先頭12桁>.plan`と同名`.json`にハッシュ・GPU UUID/名前・アーキテクチャ・CUDAドライバ・TensorRT版を保存する。`.build.log`と`.timings.json`には詳細ログと合成入力による速度計測を保存。warmup 500ms、計測3秒で、GPU compute/latencyの平均・中央値・P95が得られる場合に表示する。FP16の精度一致やrosbag走行性能はこの計測では検証しない。稼働中の推論を止めてから実行する。

E2E launchは有効なmanifestとハッシュ・ハードウェア一致、およびTensorRT deserializeとFP32入出力binding/形状検証が成功したengineだけを使用する。`force_engine_update=false`が既定で、true指定は拒否する。engineが未build・不適合なら案内を出して起動を停止し、自動buildしない。事前に`scripts/e2e_trt.sh`でbuildする。Notebookで作ったengineをJetsonへ流用しない。build失敗時は以前の成功engineを保持する。

## DINOv3版の統一（2026-10-15）

EventStateの20ch EVS蒸留と共通の公式commit `adc254450203739c8149213a7a69d8d905b4fcfa` にRGB/EVSのソース版を揃える。
`e2e.repos`と`kart_e2e.contract.DINOV3_COMMIT`を同期する。
既存checkoutは作業変更を確認してからこのcommitへ更新する。重みは別管理。
旧commitのRGB checkpoint/ONNX metadataはSPEC不一致で拒否される。
metadataだけを書き換えて再利用しない。旧版との出力一致を確認した明示的移行、
または新版での学習・ONNX再exportとtarget TensorRT engine再生成が必要。
今回の変更はソース版の統一のみ。20ch入力・EVS前処理・制御headの対応は別実装。
実DINOv3重みロード、ONNX、ROS、Jetsonの検証は未実施。

## EVS histogram encoder準備（2026-10-15）

時間補間なし・40ms・10bin・positive→negativeの20chを採用。モデルJSONから
content解像度160×120/320×240を取得する。driverはFP32 raw counts、mean0/std1を維持。
`python -m kart_e2e.evs_contract --model-json event_encoder.json --base-config <openeb_tensor_cuda_async.yaml> --output <new.yaml>`
で全parameterを保持した専用設定を生成する。線形補間モデルと異なる統計解像度は拒否。
モデルは `kart_e2e.evs_model.load_evs_encoder(weights, repo, device)` でロードし、
`encode_openeb_counts(encoder, counts)` がpadding→固定正規化→patch tokensを返す。
上記PythonはPCの学習/比較用。JetsonのROS inferenceへTorchを導入しない。
入力はTensorList /event_camera/tensor由来の[B,20,Hcontent,Wcontent]、未正規化counts。
新しいROSノードやtopicは追加しない。制御headとONNX/TensorRT接続は以下の2026-10-18実装を参照。
DSEC学習は整列RGBに合わせrectified座標、kart既定driverはraw sensor座標。
同じ画素座標イベント集合の集計規約は合わせるが、実カメラの校正差は別途評価する。

## EVS RAWから制御policyまで（2026-10-18）

RGBと独立した入力契約で、**RAW＋bag → raw-count dataset → EVS encoder＋制御head学習 → ONNX → TensorRT → 既存C++ decoder**を接続する。
encoder単体の蒸留重みは制御policyではない。必ずkartの操作教師でheadを学習する。
Web UIのdataset作成・学習フォームはRGB用。以下のEVS工程はCLIを使う。
既存RGB checkpoint/export/launchの契約は変更しない。

### 入力と時刻

- EventStateの`event_encoder.pt`と対応する`event_encoder.json`を用意する。
  20ch・40ms・10bin・時間補間なし・positive→negative・sum pooling。
  固定統計は`representation.histogram_version=2`と`window_boundary=[start,end)`が必須。
  旧版統計やlinearモデルをmetadata書換えだけで流用しない。
- contentは160×120または320×240。前者はcountsの中央zero paddingで160×128、後者はpaddingなし。
  **counts padding → 固定mean/std → encoderのpatch tokens平均 → 384→128→制御head**。
  学習もONNXも未正規化FP32 countsを受け取り、二重正規化しない。
- RAWはMetavision `RawReader(..., do_time_shifting=False)`でセンサ時刻のまま読む。
  この経路ではshift sidecarを加算しない。相対時刻readerの式と混用しない。
  SDKはEVS DockerのPythonから利用する。旧imageはPython bindingsがOFFだったため、
  Linuxホストで`./scripts/dev.sh --evs --build-local`から再buildする。学習とexport自体はSDK不要。
- bag内の`/event_camera/tensor_timing`（`std_msgs/msg/String`, `openeb.tensor_timing.v1`）から
  `[window_start_sensor_us, window_end_sensor_us)`とoffsetを採用。
  `sensor_us*1000+sensor_to_ros_offset_ns`を整数nsで検査し、
  `tensor_stamp_ros_ns`以前の最新操作を教師にする。窓終端・Tensor stamp・bag受信時刻を同一視しない。
  `latest_event_sensor_us`より後のRAWイベントはその窓へ入れない。
- 教師は`/teleop/control_cmd`（`kart_interfaces/msg/ControlCommand`）、
  状態は`/operation_mode/state`（`kart_interfaces/msg/OperationModeState`）。通常bagはMANUAL、固定収集bagは固定AUTO限定（下記）、brake/reverse除外。
  既定は操作・状態のheader時計、教師期限100ms、mode期限300ms。時計を確認した場合のみ`--clock bag`を使う。
- sequence逆行／重複、epoch逆行、epoch内の窓逆行、RAW内の時刻resetは拒否。
  sequence欠落数はmetadataへ保存し、欠落窓やoffsetを補間しない。
  RAW先頭・末尾の不完全窓と分割をまたぐ窓は除外。同じ窓を複数RAWが覆う曖昧な指定は拒否。
- RAW全イベントをメモリへ読み込まず、順次decodeして必要な窓だけ保持する。
  RAWのSHA256、stream/epoch、各sampleの窓・offset・元stampを保存し、作成中のRAW変更を拒否する。
  datasetは成功時のみ公開し、既存出力を上書きしない。

RAWはraw sensor座標でありDSECのrectified座標とは異なる。offsetは受信遅延を含む推定値。
RAWにはオンラインのpacket drop／遅延状態が保存されないため、再構成countsと当時のGPU Tensorの完全一致は保証しない。
古いbagにTensorTimingがない場合は時刻を推測してdataset化せず、再収集または別途検証した時刻対応が必要。

### 1. 記録とdataset作成

EVS Docker内の`/workspaces`、ROSとworkspaceをsource済み、学習Pythonの環境を使用する。
DINOv3固定ソースは`vcs import . < e2e.repos`、学習依存は`python_ws/e2e/requirements.txt`。
以下の`<bag>`とRAWは録画停止済みの実際のパスに置き換える。

```bash
cd /workspaces
export PYTHONPATH="/workspaces/ros2_ws/src/kart_e2e${PYTHONPATH:+:$PYTHONPATH}"

# 記録。R1開始/L1停止、走行はMANUAL。起動だけでは録画しない。
bash scripts/bringup.sh --mode collect --evs --run-name evs_manual_01

# まずSDKからRAWとROS bagを同じPythonで読めるか確認する。
python -c 'import metavision_core.event_io, rosbag2_py, kart_interfaces.msg'

# bag内が単一stream/epochならRAWを直接列挙できる。
python -m kart_e2e.preprocess_evs <bag> /workspaces/datasets/evs_train \
  --raw <camera.raw> \
  --model-json /workspaces/weights/evs/event_encoder.json
```

複数stream/epochを含む場合は`--raw`の代わりに`--raw-manifest <manifest.json>`を指定する。
同じstream/epochの分割RAWも列挙できる。対応が曖昧な場合は自動選択しない。
stream/epochはbagのTensorTimingから確認し、各RAWの所属を指定する。
manifest内の相対pathはmanifestの親directory基準。

```json
{
  "schema": "kart.evs_raw_manifest.v1",
  "files": [
    {"path": "camera_000.raw", "stream_id": "<記録されたstream_id>", "sensor_epoch": 0},
    {"path": "camera_001.raw", "stream_id": "<記録されたstream_id>", "sensor_epoch": 0}
  ]
}
```

新しいROSノードは追加しない。CLI実行ファイルは`e2e_preprocess_evs`、同じ処理を`python -m kart_e2e.preprocess_evs`でも実行可能。
独自ROS parameter／出力topic／serviceはなし。全CLI引数:

|引数|既定値|意味|
|---|---|---|
|bag / output|必須|入力rosbag directory／新規dataset directory|
|--raw / --raw-manifest|片方必須|RAW一覧（単一stream/epoch限定）／明示対応JSON|
|--model-json|必須|EventState encoder JSON。EVS policyのmetadata.jsonも使用可能|
|--command-topic|/teleop/control_cmd|操作教師|
|--mode-topic|/operation_mode/state|教師の走行モード判定|
|--fixed-config-topic|/teleop/fixed_config|任意のstd_msgs/msg/String。bag全体の収集意図と固定AUTO状態の自動判定|
|--timing-topic|/event_camera/tensor_timing|センサ窓とROS時刻の対応|
|--clock|header|教師・modeのheaderまたはbag受信時計|
|--sample-stride|10|stream/epochごとにTimingをN件おきに採用。250Hz記録なら通常25Hz程度|
|--max-skew-ms|100|過去教師の最大時間差、0より大きく1000以下|

出力は`counts/*.npy`（[20,H,W] raw FP32）、`samples.jsonl`、`metadata.json`。
保存量の目安は160×120で1sample約1.5MB、25Hzで約38MB/s。不要な高密度抽出を避ける。
検証用bagについても別directoryへ同じ抽出を行う。

### 2. 制御head学習・ONNX export・オフライン評価

```bash
python -m kart_e2e.train --modality evs \
  --train /workspaces/datasets/evs_train \
  --validation /workspaces/datasets/evs_validation \
  --repo /workspaces/python_ws/dinov3 \
  --weights /workspaces/weights/evs/event_encoder.pt \
  --output /workspaces/runs/evs_control_v1 \
  --mode steer_throttle --device cuda

python -m kart_e2e.export_onnx \
  /workspaces/runs/evs_control_v1/best.pt /workspaces/models/evs_control_v1 \
  --repo /workspaces/python_ws/dinov3

python -m kart_e2e.evaluate \
  --dataset /workspaces/datasets/evs_validation \
  --model /workspaces/models/evs_control_v1 \
  --output /workspaces/evaluations/evs_control_v1
```

学習引数はRGBと共通、追加`--modality`は既定`rgb`、EVSでは`evs`を指定。
EVSの`--weights`はRGB公式重みではなくEventState encoder bundle。
既定はencoder凍結。`--finetune`で全体更新。
`--mode steer_only --fixed-throttle 0.10 --max-throttle 0.20`も対応。
datasetの解像度・固定統計・時刻窓のSPECとencoderの不一致を拒否する。
`best.pt`にはEVS encoderも含めるため、復元・exportに元encoder.ptは不要（固定DINOv3ソースは必要）。
ONNXのFP32入力は`input_tensor=[1,20,120,160]`または`[1,20,240,320]`、出力は`control=[1,1|2]`。
ONNX checkerと3種類の入力のORT比較を通した後だけhash付きmetadataを作る。
`e2e_evaluate --dataset`はRGB/EVS共通。EVSで`--bag`だけを渡すことは不可。
出力CSV・MAE/RMSE/P95はopen-loop教師比較であり、閉ループ性能ではない。

### 3. JetsonでTensorRT build・起動

ONNX bundle（model.onnx＋metadata.json）をJetsonへ配置し、JetsonのEVS Dockerで実行する。
Torch、DINOv3 Pythonソース、encoder.ptは推論には不要。

```bash
# 実行先GPUで明示build。binding名・FP32・形状・hash・GPU/TRT版を検証する。
bash scripts/e2e_trt.sh /workspaces/models/evs_control_v1

# 実行時の全OpenEB設定をモデルから作成。選ぶbackendのbaseを使う。
python -m kart_e2e.evs_contract \
  --model-json /workspaces/models/evs_control_v1/metadata.json \
  --base-config /workspaces/ros2_ws/src/kart_bringup/config/sensors/openeb_tensor_cuda_async.yaml \
  --output /workspaces/models/evs_control_v1/openeb_tensor.yaml

# 実車AUTO muxへ接続するmission。起動ではAUTOも録画も開始しない。
bash scripts/bringup.sh --mode e2e --evs \
  --e2e-model-dir /workspaces/models/evs_control_v1 \
  --evs-tensor-config /workspaces/models/evs_control_v1/openeb_tensor.yaml \
  --rgb-fps 0 --infra-fps 0
```

上記のEVS E2EでRGB/Infraを両方0にした場合はRealSense/IMUと空のsensor containerを起動しない。
RGB/Infraの記録を併用するなら各fpsを指定する。EVS推論の入力には使わない。
推論2 Componentは**既存OpenEBプロセスの`/event_camera/tensor_container`へload**する。
RGB encoder、localization、追従controller、別のAUTO producerを追加しない。
EVSモデルとdriverの解像度・bins・窓・raw正規化・tensor名を起動前に照合する。

センサと車両が既に別途起動済みで、確認用topicへだけ出力する場合:

```bash
ros2 launch kart_bringup evs_e2e.launch.py \
  model_dir:=/workspaces/models/evs_control_v1 \
  evs_tensor_config:=/workspaces/models/evs_control_v1/openeb_tensor.yaml
```

この単体launchはcontainer・カメラ・車両を作成せず、`drive_enabled=false`。
`drive_enabled:=true`はAUTO muxへ接続する明示指定。missionでは既存設定`e2e_drive_enabled=true`を使用。
既存のAUTO・入力鮮度・競合publisher・異常STOPラッチ・出力上限gateはRGBと共通。
元Tensor stampをそのまま指令へ伝える。50Hzはgate周期で、EVS推論FPSではない。

|方向|既定topic|型|QoS・内容|
|---|---|---|---|
|TensorRT入力|/event_camera/tensor|isaac_ros_tensor_msgs/msg/TensorList|reliable / volatile / depth1 consumer、raw FP32 [1,20,H,W]|
|TensorRT出力・decoder入力|/e2e/tensor_output|isaac_ros_tensor_msgs/msg/TensorList|reliable / volatile / depth1、control [1,N]|
|decoder入出力|上記Control decoderの表と同じ|同表参照|mode入力、control_cmd/status/条件付きSTOP要求|

追加ROS parameterはなし。EVS TensorRTの**全parameter**はbringup `config/e2e/evs_tensor_rt.yaml`、
decoderは既存pkg `config/control_decoder.yaml`とbringupの同名YAMLを使う。
追加launchの構成正本は`config/e2e/evs_launch.json`。
全launch引数は空文字が既定（JSON/YAML保持）:
`model_dir`（必須bundle）、`evs_tensor_config`（必須・driverへ適用した全YAML）、
`container_name`（既定/event_camera/tensor_container、変更不可）、`input_topic`（既定/event_camera/tensor、変更不可）、
`fixed_throttle`、`use_fixed_throttle`、`drive_enabled`、`use_sim_time`（既存decoderと同じ意味）。
module内部の`input_topic`追加は未指定時に既存RGB `/e2e/tensor_input`を使用する。
外部driverが実際に指定YAMLで起動済みかは利用者が確認する。launch終了だけでは外部containerからunloadされない。

### 検証範囲

`test_evs_pipeline.py`にhistogram境界・極性・sum pooling・最新event上限・RAW端・epoch・因果教師・
RAW重複拒否・ONNX metadata/engine bindingのテストを追加。
`KART_DINOV3_REPO`を設定すると、公式固定DINOv3＋**ランダム初期重み**と合成countsで両headの
1epoch学習・encoder凍結・checkpoint復元・ONNX/ORT比較・dataset評価を実行する。
320×240のfinetune勾配・ONNX比較も検証する。これは接続の検証であり、蒸留済み重みの品質や走行精度ではない。
RAW reader／ROS bag reader／TensorRT／launchの一部テストは代替fixture。
実RAW＋実bag抽出、EVS SDK結合、CUDA/TensorRT実build、ROS結合、Jetson性能、実車走行は未確認。


再現テスト（PCの学習環境、公式ソースは固定commit、重みのダウンロードなし）:

```bash
PYTHONPATH=ros2_ws/src/kart_e2e \
KART_DINOV3_REPO=/workspaces/python_ws/dinov3 \
python -m pytest -q ros2_ws/src/kart_e2e/test
```


## AUTO固定スロットル・手動ステア

`auto_fixed_throttle=-1.0`（既定）は通常動作。0..speed_scale（標準上限0.3）を
指定すると、□AUTOで固定throttle＋手動ステア、△MANUALで通常R2/L2＋手動ステアになる。
AUTO中もステアを操作でき、△でSTOPを挟まずMANUALへ直接引き継げる。
正常なMANUAL→AUTOも直接切替可能。STOPからの初回進入は全入力中立が必要で、
AUTO初回進入は0.3秒中立を配信してmux/bridgeのHOST handshakeを待つ。
直接切替時は中立待ちを挟まない。×はSTOP、固定AUTO中の○はbrake優先＋STOP、L2は中立＋STOP。
入力途絶・異常・再起動後はSTOPから再armし、モード変更だけで異常解除しない。
固定値は速度ではなく補正前の正規化要求で、起動時固定。変更はSTOP後に再起動する。

JoyManagerNodeが`/teleop/control_cmd`（kart_interfaces/msg/ControlCommand、reliable/depth1）へ
元Joy stampで合成指令を出版する。bringupは固定設定からcommand_muxの
`auto_use_teleop`（bool、ノード既定false）を導出し、有効時はAUTOでもこの入力を選ぶ。
この間はE2Eなどの`/auto/control_cmd`を車両へ流さない。
`/teleop/fixed_config`（std_msgs/msg/String、reliable/volatile/depth1、10Hz）は
`kart.auto_fixed_throttle.v1`、enabled、throttle、speed_scale、entry_neutral_s、mode、armedを記録する。
armedは論理的許可で、実車速度や基板ackを示さない。独自service/actionは追加なし。
MANUALは通常のthrottle trim、固定AUTOは既存bridgeのAUTO補正規則を使用する。
録画はR1開始／L1停止（bringup標準）で、走行モード操作とは独立。

```bash
bash scripts/bringup.sh --mode collect --auto-fixed-throttle 0.10 --run-name auto_fixed_010
# EVSの場合は --evs を追加
```

CLI/launchの旧`manual_fixed_throttle`は`auto_fixed_throttle`へ変更した。
実運用設定はkart_bringup/config/input/joy_manager.yaml、未指定ならYAMLを保持する。
TUIの「AUTOスロットル」で通常／固定を選ぶ。collect/driveで利用可能。E2Eでは収集用の固定設定を無効化し、モデルのステア＋E2E専用固定スロットルを使用する。
RGB/EVS教師抽出はbag内の固定AUTO有効設定を自動検出する。固定収集bagでは固定AUTOだけを採用し、MANUALの復帰・調整区間を全て除外する。通常bagは従来どおりMANUALだけを採用する。
ROS結合・実車動作は未確認。portable testで固定出力、直接切替、異常ラッチ、HOST handshakeを検証する。


### 固定AUTO収集の自動判定（RGB／EVS共通）

追加フラグなしで、bagを一度走査してから抽出モードを決定する。
RGBのevaluate --bagも同じ抽出器を使う。namespace変更時は各CLIの--fixed-config-topicを指定する。
`/teleop/fixed_config`に`schema=kart.auto_fixed_throttle.v1`かつ`enabled=true`が
1件でもあれば、bag全体を`fixed_auto_only`として扱う。設定がSTOP/MANUAL中に記録されても
固定収集の意図として検出する。**このbagのMANUALは復帰・調整であり、全区間を教師から除外する。**
固定を途中で無効化しても、そのbagのMANUALを教師へ復帰させない。
AUTO中の人間のsteeringは`/teleop/control_cmd`から値を変えずに採用する。
通常のE2E自動操舵指令を教師へ代用しない。

各候補ではmodeの鮮度300ms、既存の操作期限（既定100ms）と有効範囲を確認する。
さらにサンプル以前300ms以内の最新fixed_configが対応schema・enabled・armed・mode=AUTOであり、
command.throttleが設定throttleと一致すること（絶対誤差1e-6以内）を要求する。
未来の設定、設定途絶、STOP、brake/reverse、固定値不一致の中立待ちを除外する。
ゼロ固定も含め、AUTOの連続観測開始からentry_neutral_s（通常0.3秒）は保守的に除外する。
MANUAL→AUTOの直接切替でも抽出時はこの待ち時間を取る。modeの観測欠落が300msを超えた場合も
連続観測を取り直す。教師指令が今回のAUTO開始より古い場合も除外し、復帰MANUALの指令を持ち越さない。armedは物理運動や基板ackを証明する値ではない。

固定AUTO有効設定がないbagは`manual_only`で従来動作を維持する。
旧`kart.manual_fixed_throttle.v1`もMANUAL収集として扱う。
固定設定topicがない古いbagの収集意図を、ファイル名やthrottle値から推測しない。
固定AUTOのbagに設定topicを記録し忘れた場合は自動判定できない。
未知schema・壊れたJSON・型や範囲の異常は、MANUALへの誤フォールバックを避けるため抽出エラーとする。

Stringにはheaderがないため設定だけはbag受信時刻を使う。RGB/EVSの既存clock選択は維持し、
header選択時も同じROS時計であることが必要。受信遅延を逆補正したり、未来の設定を先取りしない。
出力metadata.jsonの`teacher_selection.policy`に自動判定結果、`fixed_config_records`に設定件数、
`accepted`/`rejected`に教師候補の判定内訳を保存する（RAW範囲・画像処理の最終採用数は`samples`）。
各samples.jsonlにも`teacher_source=fixed_auto|manual`を保存する。

通常と同じコマンドで新しい出力先へ再抽出すれば、既存の固定AUTO bagにも反映される。
学習済みモデルには遡及反映されないため、抽出後のdatasetで再学習する。
`test_teacher_selection.py`と`test_evs_pipeline.py`で混在モード、鮮度、設定変更、
人間のステア保持、旧bag互換、RGB/EVSの抽出を合成入力で確認する。実bag／ROS SDKでの実行は未確認。


### E2E AUTOが途中で停止する場合

□は押下の立ち上がりでAUTO要求を送り、保持中の繰り返し要求や自動再armは行わない。
入力期限は標準150msで、画像stampからの経過とtensor受信からの経過の両方を検査する。
mux/bridgeも標準150msの指令期限を持ち、元画像stampを保持するため、decoderからの
出版Hzが高くても古い画像の指令は有効にならない。AUTO進入の中立期間は100msで、
進入後の新しい画像の推論が必要。mode失効・他AUTO publisher・推論不正・USB/基板異常でもSTOPとなる。

`/operation_mode/request`のsourceで要求元を確認し、`/e2e/status`の最初の状態変化を照合する。
状態topicは既存std_msgs/msg/String・reliable/depth1で、10Hzに加え状態変化時に即時出版する。
decoderが最初にSTOP判定した時はWARNINGに理由、mode、image_age_ms、mode_age_ms、
tensor_receipt_age_ms、prediction_count、input_timeout_ms、予測steering/throttleを残す。時刻がないageは-1。
最初の理由が次tickのfault_latchedに置き換わる前に記録するための診断であり、停止判定や
STOP要求周期は変えない。標準bagにもe2e/statusを保存する。未記録の過去bagへは遡及しない。
実機ログで原因を特定する。固定スロットルの値やモデルの操舵精度だけから停止原因を推測しない。


画像stampの小さな未来ずれは`image_future_tolerance_s=0.005`（5ms）まで許容する。
実機ログのimage_age_ms=-2.1／tensor_receipt_age_ms=7.3／prediction_count=1／steering=0.337598は、
従来のnow < image_stampという判定でSTOPしていた。5ms以内でも元の画像stampを保持し、
過去150msの期限を延長したり時刻を書き換えたりしない。5msを超える未来ずれ、古い画像、
AUTO進入前の画像、不正な値、受信途絶、mode失効は引き続きSTOPとなる。
許容値の上限は50msで、有限値を要求する。ログにもimage_future_tolerance_msを出す。
C++ gateで2.1ms、5ms境界、境界超過、150ms超過、非有限stamp、異常ラッチを検証する。
ROS/GPUの実機再走行は未確認。センサstampが未来になった理由そのものは別途時刻同期を確認する。

### RGB特徴量のオフライン確認

Notebook学習UIの「特徴量を確認」で学習済み実験（best.pt）、RGBデータセット、
確認枚数（既定12、1〜64）を選択する。全区間から均等に選択し、結果履歴を開くと
RGB（モデル入力212×120）、DINOv3パッチ特徴のPCA、ステア出力の入力勾配を並べて表示する。
フレームスライダーで教師と予測も確認できる。学習環境のPython・device・encoder_repoを使用する。

```bash
python -m kart_e2e.features --checkpoint /path/to/run/best.pt \
  --dataset /path/to/dataset --repo /path/to/dinov3 \
  --output /path/to/new-result --device cpu --max-frames 12
# ROS環境にinstallした場合は ros2 run kart_e2e e2e_features も同じ引数
```

ROSノードではなくオフラインCLIで、ROS topic・独自ROS parameterはない。
checkpoint/dataset/repo/outputは必須。deviceはcpu、max-framesは12が既定。
重みのダウンロードは行わず学習済みcheckpointを読み込む。RGBと20ch EVSに対応。
`e2e/features/YYYY-MM-DD/HH-MM-SS/<name>`にreport.json、画像、PCA基底を保存し、
checkpoint・manifest・入力画像のSHA256を記録する。UIのごみ箱・復元にも対応する。
PCAの基底と色範囲、入力感度の強度基準は結果内で共通。別結果の色は直接比較できない。
PCAは空間的な特徴の違いであり、制御headがその領域を使った証明ではない。
赤い入力感度は実際のステア出力の局所勾配で、attentionや走行成功率とは異なる。
PyTorch checkpointの確認であり、ONNX/TensorRTとの数値同等性はexport時の検証を用いる。


### 全体fine-tuning

`--finetune`はDINOv3 backboneを凍結せず、制御headと一緒に全体をAdamWで更新する。
学習率未指定時は全trainable parameterに`1e-5`（通常のhead学習は`1e-4`）を使用する。
`--learning-rate`を明示すると指定値を優先する。epoch既定は両方式とも100。
Notebook UIでは「全体をfine-tuning」を選択する。標準学習率は切替に追従し、
別の値を手入力している場合は保持する。学習名候補に`-finetune`を付け、既存実験と別に保存する。
初期値は学習環境のローカル事前学習重み。既存best.ptからの再開ではない。
RGB/EVSのCLIに共通し、best.pt→ONNX→TensorRTは同じ経路を使用する。
`run.json`とcheckpointのprovenanceにfinetune・実効学習率・更新対象parameter数を記録する。
起動ログにも実効設定を出力する。全体学習の品質・GPUメモリ使用量は実データ／対象GPUで別途確認する。


### 凍結学習の特徴量キャッシュとDataLoader

Notebook UIの「特徴量キャッシュを使用」は既定ON、DataLoader workersは既定8（0〜64）。
CLIも`--feature-cache`が既定ON、`--num-workers 8`が既定。`--no-feature-cache`で無効化できる。
`--finetune`時はチェック設定にかかわらずキャッシュを使わず、backboneまで更新する。
UIはfine-tuning中のキャッシュ欄を無効表示し、通常学習に戻すと選択状態を復元する。

凍結時のRGB CLS特徴／EVSの正規化・padding後のpatch平均特徴（ViT-S/EVSは384次元、Tinyは192次元）をFP32で保存する。
初回だけencoderを実行し、以降のepoch・同条件の別実験はheadのみ学習する。
RGBファイル/countsの内容と順序、encoder重みSHA256、モデル仕様、前処理実装、Torch版、deviceが一致した
キャッシュのみ再利用する。ラベルは毎回現在のデータセットから取得する。学習率・epoch・head出力数の
変更では特徴量を再利用できる。入力・重み等が変われば別キャッシュになり、元のキャッシュは保持する。
一時領域からatomic公開し、同じキーはfile lockで排他。特徴量ファイルのSHA256とshapeを確認し、
破損時はエラーで停止する（表示されたcacheディレクトリを削除して再作成できる）。

UIの保存先は`e2e/.feature_cache/<sha256>/`。CLIの`--cache-dir`既定は出力先の親の
`.feature_cache`。既存datasetや画像は変更しない。キャッシュ作成／再利用はログに表示し、
`run.json`・checkpoint provenanceに要求値・実効値・cache key・num_workersを残す。
各DataLoaderに8 workerを使用し、spawnで起動する。学習・検証loaderはそれぞれworkerを保持する。
CUDA時はpin_memoryとnon_blocking転送を使う。小規模CPU検証には`--num-workers 0`も指定可能。
保存するbest.ptには従来どおりencoderも含み、export／TensorRT推論時にキャッシュは不要。


### RGBデータセットの高速保存形式

新規RGB抽出は`images/00000000.npy`を既定とする。圧縮なしNumPy、uint8、
`[height,width,3]`、RGB順で、元解像度と画素値を保持する。教師の選択・時刻対応・
学習前処理・モデル入力契約は変更しない。424×240では1枚約305 KB（headerを含む）。
metadata.jsonにimage_format=npy、image_dtype=uint8、image_layout=HWC_RGBを記録する。

`e2e_preprocess_bag BAG OUTPUT --image-format png`で互換用PNGを選択できる。
PNGは画素値を保持しcompress_level=1で高速保存する。`--image-format`既定はnpy。
既存PNGデータセットもそのまま学習・特徴量可視化・ONNX評価・特徴量キャッシュに使用できる。
ファイル拡張子で読み分け、NPYはallow_pickle=False、dtypeとshapeを検証する。
可視化の表示用画像は引き続きPNG。既存データセットの自動変換や上書きは行わない。
Notebook UIの新規作成およびrosbag評価の一時抽出もNPYとなる。EVS countsの形式は変更しない。

### ROS RGB前処理の数値診断（読み取り専用）

`e2e_rgb_pipeline_check`はROSノードではないオフラインCLIで、独自ROS parameter・publisherはない。
実機で短い診断bagを記録し、captureはROS環境でシリアライズ済みCPUデータを抽出、
compareは学習用Torch環境で比較する。別interpreterで実行できる。制御や設定は変更しない。

```bash
# 通常E2Eが動いている環境で、数秒分を記録してCtrl-C。
ros2 bag record -o /tmp/kart_rgb_probe_bag \
  /realsense/color/image_raw /e2e/rgb/image /e2e/tensor_input /e2e/tensor_output
# ROS環境
ros2 run kart_e2e e2e_rgb_pipeline_check capture \
  --bag /tmp/kart_rgb_probe_bag --output /tmp/kart_rgb_probe
# 学習環境（probeディレクトリをNotebookへ渡して実行してもよい）
python -m kart_e2e.rgb_pipeline_check compare --probe /tmp/kart_rgb_probe \
  --output /tmp/kart_rgb_probe_report.json --model-dir /path/to/deployed/model
```

captureのbag/outputは必須。max-framesは既定16（1〜256）。topic既定は上記4つで、
`--image-topic/--resized-topic/--input-topic/--prediction-topic`で上書き可能。
compareのprobe/outputは必須、model-dirは任意（省略時は前処理だけ比較）。
全ファイルの既存出力を上書きしない。captureのtopic引数は購読先ではなくbag内の読取topic。
未installの場合は`PYTHONPATH=ros2_ws/src/kart_e2e python -m kart_e2e.rgb_pipeline_check ...`も使える。

同一header stampで画像とtensorを照合する。`normalization_matches`は実際のリサイズ後RGBから
再計算した正規化・NCHWとROS入力tensorの最大差が1e-5以内かを示す。
`resize_within_one_lsb`は元RGBの学習前処理とGPUリサイズ後画像を比べ、uint8の丸め差1を許容する。
許容差はGPU kernelの一致を証明しない。RGB/BGR逆転、二重scale、layout誤りは合成fixtureで検出を確認。
モデル指定時は、同じONNXをCPUで元RGB／実ROS入力のそれぞれから実行し、操舵への影響と
記録TensorRT出力との差も数値化する（TensorRT差の合否を自動判定しない）。
model-dirには実機で使ったmodel.onnx/metadata.jsonを指定し、別モデルとの比較を避ける。
実ROS TensorListの記録・GPU readback・Jetson実機診断は未検証。capture/compareの単体テストは代替bag fixture。


### DINOv3 ViT-Tinyと2段階蒸留（RGB）

`--architecture`は既定`dinov3_vits16`、追加`dinov3_vitt16`で独自Tinyを選択する。
固定commitの公式DINOv3実装を使い、ViT-Sと同じ12層・patch16・head次元64・RoPE・
storage token 4・LayerScale・FFN比4を維持する。Tinyはembed_dim=192、heads=3、
encoder 5,492,160 parameter。入力・正規化・paddingは既存RGBと同じ。
公式配布のTiny事前学習重みではなく、ローカルViT-Sから蒸留して作る。

**第1段階：特徴蒸留**

同じRGBに対する教師のCLS特徴384次元へ、TinyのCLS特徴192次元を学習用Linearで射影し、
`MSE + 0.5*(1-cosine)`を最小化する。教師はeval/frozen、Tiny本体と射影は全体更新する。
異なる次元を直接同一視せず、教師空間に近づける。完全な特徴一致や制御精度を保証しない。
参考は[GEPの特徴次元射影とMSE/cosine alignment](https://github.com/uzh-rpg/generative_event_pretraining/blob/master/src/gra.py)。
GEPのevent/RGB間alignmentをそのまま導入するのではなく、同一RGBによる圧縮用に独立実装した。
今回は下流control headが使うCLS特徴を対象とし、patch tokenや生成モデルの学習は行わない。

```bash
python -m kart_e2e.distill --train /path/to/train --validation /path/to/val   --repo /workspaces/python_ws/dinov3 --teacher-weights /path/to/vits16.pth   --output /path/to/tiny-encoder --device cuda
# --teacher-weightsにはkart RGB ViT-Sのbest.ptも指定可能
```

CLI（ROS install後は`ros2 run kart_e2e e2e_distill`）であり、ROSノード・topic・parameterはない。
train/validation/repo/teacher-weights/outputは必須。既定はepochs=10、batch-size=32、
num-workers=8、learning-rate=1e-4、mse-weight=1、cosine-weight=0.5、seed=42、device=cuda。
`--feature-cache`既定ON（`--no-feature-cache`でOFF）、教師特徴だけをキャッシュする。
cache-dir既定はoutput親の`.feature_cache`。teacherはローカルファイルをstrict loadし、
checkpoint/SPEC不一致を拒否する。重みをダウンロードしない。EVS/他encoderを教師にしない。
蒸留中にTiny本体は変わるため、その特徴はキャッシュしない。

最良validation_lossの`encoder.pt`（format=`kart.rgb_encoder.v1`）へTiny encoder、
学習用射影、SPEC、損失、教師SHA256、provenanceを保存する。metrics.jsonlは各epochの
train/validation_loss・mse・cosineを分けて記録する。run.jsonに設定とcache keyを保存する。

**第2段階：制御head学習**

```bash
python -m kart_e2e.train --architecture dinov3_vitt16   --weights /path/to/tiny-encoder/encoder.pt --train /path/to/train --validation /path/to/val   --repo /workspaces/python_ws/dinov3 --output /path/to/tiny-control --device cuda   --mode steer_only --fixed-throttle 0.1
```

既定はTinyを凍結し192→128→control headのみ学習。必要なら`--finetune`で全体更新。
蒸留用射影はencoder bundleに残すが、制御モデルにはロードせず、export/実行にも含めない。
既存の画像/NPY、特徴量キャッシュ（Tinyは192次元）、損失表示、特徴量可視化に対応する。
ViT-S重みをTinyに無理に部分ロードせず、Tiny学習には蒸留済みencoder bundleを必須とする。

最終best.ptは従来の`e2e_export_onnx`に渡せる。metadataのmodel_spec.encoderでSmall/Tinyを
識別し、Tinyも同じ`image=[1,3,120,212]`、`control=[1,1|2]`でTensorRT→既存decoderへ接続する。
既存ViT-S checkpoint/SPECは保持。EVSのTiny化は未対応で、EVS経路を変更しない。
合成画像・ランダム初期ViT-Sによる蒸留/両head学習/復元/ONNX・ORT/可視化の接続をテストする。
実事前学習重みでの蒸留品質、TensorRT実build、Jetson速度、実車の走行精度は別途検証する。

Notebook UIは「RGBモデル」でTinyを選び、データを選択して第1段階を実行する。
教師は学習環境の公式ViT-S重み、または学習済みViT-S実験から選べる。
完了後に蒸留済みTiny encoderを選び、下の通常学習ボタンで第2段階を実行する。
蒸留損失カーブと制御MSEは別表示。encoderは
`e2e/encoders/YYYY-MM-DD/HH-MM-SS/<name>`に保存し、ごみ箱/復元・使用中lockに対応する。

### Notebook GUIでのEVS操作

`tools/app`の学習UIから20ch EVSを選択できる。
「学習環境」でEventState encoder PTと対応するJSONを指定し、
「データセット」で入力種別EVS・bagを選択する。bag直下のRAWが1件、
記録timingのstream/epochが1組なら自動対応する。複数の場合は詳細設定で
RAW manifest絶対パスを指定する。CLIも`--raw`/`--raw-manifest`省略時は同じ自動検出を使う。
`preprocess_evs`を実行するため、学習用PythonにはRAW reader SDKが必要。
次に「学習」でEVSへ切り替え、学習・検証データを選択する。
既定encoder凍結、epoch 100、Smooth L1（β=0.1）、Plateau scheduler ON、workers 8、特徴量キャッシュON。
全体fine-tuning、損失カーブ、ONNX exportは共通操作で使用できる。
EVSモデルの評価ではbagの代わりに作成済みEVSデータセットを選択する。
RGB/EVSデータの混在とEVSへのRGB Tiny蒸留を拒否する。
特徴量可視化も20ch EVSに対応。新規ROSノード・topic・parameterはない。

### RAW readerのNumPy / h5py ABIエラー

今回の環境ではNumPy 2.5.4でイベント時刻に不正な巨大値・負値が出てSDKがsegfaultし、
NumPy 1.26.4では同じRAWの両reader先頭30万イベントが正常だった。
h5py更新だけではSDKのイベント読み出しの互換性は解決しない。
EVS RAW抽出は専用venvのNumPy 1.26.4 / h5py 3.11.0を使用し、
学習・ONNX用Pythonとは分離する。GUIの「EVS RAW抽出用Python」で指定する。
DockerのEVS最終イメージは`/opt/evs-raw/bin/python`を用意する。
既存コンテナでは次で作成できる。

```bash
uv venv --python /usr/bin/python3 --system-site-packages /workspaces/.venv/evs-raw
uv pip install --python /workspaces/.venv/evs-raw/bin/python \
  --no-deps --only-binary=:all: 'numpy==1.26.4' 'h5py==3.11.0'
```

SDK/ROSをsystem-site-packagesから参照し、NumPy/h5pyだけvenvの版を優先する。
GUIはrepo配下の`.venv/evs-raw/bin/python`があれば既定に採用し、なければ
`/opt/evs-raw/bin/python`を使用する。`KART_EVS_RAW_PYTHON`でも既定を指定できる。
RAW readerはNumPy 2ではSDKを呼ぶ前にエラーにし、専用環境への変更を案内する。
恒久反映はLinux側で`./scripts/dev.sh --evs --build-local`により再buildする。
先頭30万イベントの成功は、RAW全体・抽出完了や学習の成功を意味しない。
Docker buildおよび実RAWの全区間抽出は対象環境で確認する。

RAWの時刻逆行は`RAW timestamp regression`として、ファイル・0始まりevent index・
chunk内/境界・前後時刻・逆行量（us）・要求窓をエラーへ出力する。
この検査だけではセンサー時刻リセットと局所的なイベント順序乱れを区別できない。
ログの逆行量とbagのstream/epoch、SDK直接読出しの結果を比較して原因を調べる。
局所的な順序乱れには`--max-reorder-us`（既定5000µs、0..100000、0で厳格検査）を使用する。
観測最大時刻からの遅れが許容幅以内なら、窓終端＋許容幅まで先読みしてから
`[start,end)`のイベントを安定整列する。元の時刻・座標・極性・個数は変更せず、時間補間はしない。
chunk内・chunk間とも同じ上限で検査し、累積的な逆行も観測最大時刻から測る。
負時刻・許容幅を超える逆行は停止する。RAW端の不完全窓は従来どおり除外する。
metadata.jsonに設定値とRAWごとの観測最大遅れを保存する。GUIにも同じ許容幅を設ける。
実ログの4095µsに対する既定5000µsであり、全記録の上限を保証する値ではない。

抽出中の変更検査はSDKが生成・更新する`<RAW名>.tmp_index`を除外する。
対応するRAWが存在する`*.raw.tmp_index`だけが対象で、RAW本体・metadata.yaml・
rosbag storage等は監視を継続する。変更時はファイル名をエラーに表示する。
SDKの索引作成を録画継続と誤認しないための処理であり、録画中の抽出を許可するものではない。

### 20ch EVSのオフライン特徴量可視化

`e2e_features` / `python -m kart_e2e.features`はdatasetのmodalityでRGB/EVSを判定する。
引数・既定値は共通（max-frames=12、1..64、device=cpu）。EVSは学習済みEVS checkpointと
完全一致するmodel_specのcountsデータセットを使用し、入力種別やSPECの不一致を拒否する。
RAW SDKは不要で、学習用PythonのPyTorchで実行する。

EVSでは未正規化20ch countsをモデルの固定padding/mean/stdで処理し、
実際のpatch tokensを共有PCAで可視化する。ステア感度は実際の制御headの出力から、
未正規化20ch countsへの絶対勾配をchannel平均したもの。画像上の赤は局所感度であってattentionではない。
入力表示はpositive=赤・negative=青、各10binの合計。別に正10bin/負10binの一覧PNGも出力する。
表示のみlog1p＋非ゼロ値の共有p99スケールを使用し、モデル入力は変更しない。
160x120と320x240のcontent座標を保持し、PCAはpadding分を除いて表示する。
`kart.evs_features.v1`のreportとinput/PCA/saliency/channels PNG、PCA基底、入力hashを保存する。
RGB reportとの読み込み互換を維持。GUIで実験を選ぶと同じmodalityのdataset候補を表示する。
合成counts・ランダム重みで実装検証し、学習済み実データでの定性評価は利用者環境で行う。

### 制御学習のSmooth L1と学習率スケジューラ

RGB Small/Tiny・EVSの制御学習で共通。`--loss smooth_l1`が既定、
`--smooth-l1-beta 0.1`が既定（正値必須）。正規化指令の誤差がβ未満は二乗項、
それ以上は線形項になる。MSEと損失値の尺度は異なる。従来比較は`--loss mse`。
学習・検証の実際の目的関数を`train_loss`/`validation_loss`、比較用MSE・MAEを別途保存する。
`best.pt`は従来どおり`validation_mse`最小で選び、推論・exportの契約は変更しない。

`--lr-scheduler plateau`が既定。`none`で無効化できる。ReduceLROnPlateauで
検証の実目的関数`validation_loss`を監視する。`--lr-factor 0.5`、
`--lr-patience 2`、`--min-lr 1e-7`が既定。
factorは0より大きく1未満、patienceは0以上、min-lrは0以上かつ有効時に初期LR以下。
PyTorchのmode=min、threshold=1e-4（relative）、cooldown=0、eps=1e-8を使用する。
patience=2では改善しないepochが3回続いた時点で低下し、次epochから適用する。
各epochに使用した`learning_rate`と、更新後の`next_learning_rate`をmetrics.jsonlへ残す。
run.json/checkpointに損失・β・scheduler設定・監視/選定指標を保存する。

GUIは損失関数・βとschedulerを選択でき、Plateau選択時に詳細値を設定する。
損失カーブはMSEと実学習lossを切替可能。数値表には使用LRと次epoch LRも表示する。
過去のMSEのみの履歴は従来どおり表示する。Tinyの第1段階の特徴蒸留は独自目的関数のまま。

### RGBのscratch / CNN比較

RGBの`--architecture`は`dinov3_vits16`（384次元）、`dinov3_vitt16`（192）、
`dinov3_vitb16`（768）、`simple_cnn`（192）を選択できる。
ViT-Bは同じ固定公式DINOv3ソースの`dinov3_vitb16(pretrained=False)`で構築し、
事前学習済みを選んだ場合だけローカル重みをstrict loadする。重みは自動取得しない。
ViT-BにViT-S重みを暗黙流用しない。

`--initialization pretrained`が既定。重み付き学習は既定100 epoch。
`--initialization scratch`では重みを読み込まず、encoderとheadを必ず全体学習し、
既定100 epoch・学習率1e-4を使う（`--epochs`/`--learning-rate`で上書き可能）。
`--weights`はpretrainedでは必須、scratchとの併用は誤設定として拒否する。
scratch中は特徴量キャッシュを無効化する。凍結したランダム特徴をbaselineとしない。
事前学習済み全体fine-tuningの既定学習率1e-5は維持する。
`--seed`は既定42（0..2147483647）、GUIでも指定可能。

CNNは3→24→48→96→192の4段Conv3x3/stride2/pad1＋GroupNorm8＋GELU、
空間平均で192次元とする。scratch専用でDINOソースも重みも不要。
RGB resize/正規化/padding、headの隠れ次元128・GELU・Dropout0.1と出力制約は全モデル共通。
CNN仕様は`encoder=simple_cnn`、`encoder_commit=kart.simple_cnn.v1`で区別する。
CNNの前処理は比較のためImageNet正規化を維持し、モデルに合わせて暗黙変更しない。

```bash
# 事前学習済みViT-B: 重みは利用者が配置
python -m kart_e2e.train --architecture dinov3_vitb16 \
  --train /workspaces/datasets/train --validation /workspaces/datasets/validation \
  --repo /workspaces/python_ws/dinov3 --weights /workspaces/weights/dinov3/dinov3_vitb16.pth \
  --output /workspaces/runs/base_pretrained --device cuda
# 同じViT-Bのscratch、50 epoch（--weightsを指定しない）
python -m kart_e2e.train --architecture dinov3_vitb16 --initialization scratch \
  --train /workspaces/datasets/train --validation /workspaces/datasets/validation \
  --repo /workspaces/python_ws/dinov3 --output /workspaces/runs/base_scratch --device cuda
# 小規模CNNのscratch。共通CLIの--repoは渡すがCNNでは参照しない
python -m kart_e2e.train --architecture simple_cnn --initialization scratch \
  --train /workspaces/datasets/train --validation /workspaces/datasets/validation \
  --repo /workspaces/python_ws/dinov3 --output /workspaces/runs/cnn_scratch --device cuda
```

GUIではRGBモデルと初期化方法を選択する。CNN選択時はscratch固定。
ViT-B事前学習済み重みは「学習環境」の専用欄で設定する。
学習runにarchitecture・初期化・seed・実trainable parameter数を保存する。
同じbag分割・テストbag・損失・seed条件を使い、既存のONNX export→bag評価で比較する。
同一epoch比較と収束後比較は別に扱い、50 epochでの収束を保証しない。
RGB Small/Tiny/Base/CNNの復元・export・評価・オフライン特徴量可視化を共通経路で扱う。
CNNのPCAは16画素strideの畳み込み空間特徴でありViTのpatch tokenとは構造が異なる。
EVSのモデル/初期化契約は変更しない。ROSノード・topic・parameter追加なし。

### AMP（混合精度）学習

制御学習の`--precision`は`auto`（既定）/`fp32`/`bf16`/`fp16`。
GUIの「学習精度」でも同じ値を指定する。RGB Small/Tiny/Base・CNN・EVSの
head学習／全体fine-tuning／RGB scratchに共通で適用する。Tiny第1段階の特徴蒸留は対象外。

- auto: CPUはFP32。CUDAは選択したdeviceのnative BF16（compute capability 8以上かつ
  `is_bf16_supported()`）を優先し、非対応ならFP16を使う。
- fp32: AMPなし。従来比較用。
- bf16: CUDA native BF16必須。CPU・非対応GPUでは明示エラー。
- fp16: CUDA必須。`torch.amp.GradScaler`を併用し、非有限勾配時はPyTorch標準のstepスキップ・scale調整を行う。

forwardだけautocastし、出力をFP32へcastした後、loss・比較MSE・MAEをFP32で計算する。
backwardはautocast外。model自体をhalfへ変換せず、parameter/state_dictはFP32を維持する。
optimizerは`zero_grad(set_to_none=True)`を使用する。
特徴量キャッシュの初回encoder計算はFP32を維持し、キャッシュ内容をAMP設定で変更しない。
キャッシュ使用時のAMPはheadに適用する。全体更新では従来どおりキャッシュを無効化する。

run.jsonとcheckpoint provenanceに要求`precision`、`effective_precision`、`amp_enabled`、
`grad_scaler_enabled`、`feature_cache_precision`を保存する。FP16ではmetrics.jsonlに`grad_scale`も記録する。
起動ログとGUI実験一覧に実効精度を表示する。既存のFP32 checkpoint復元・export契約は維持する。
AMPの検証forwardもautocastするため、指標集計がFP32でもFP32 forwardと完全一致する保証はない。
`--precision fp32`との精度比較と、対象GPUでの処理時間・メモリ計測を行うこと。
CPU/モックによる選択・学習互換テスト済み。CUDA上のBF16/FP16学習テストはGPUがある場合のみ実行する。

### TensorRT build精度の選択

学習時AMPとTensorRT build精度は独立。ONNXは従来どおりFP32入出力を使用する。
FP16 buildは内部の低精度演算を許可する指定で、全layerのFP16化を保証するものではない。
ビルド後にRGB/EVSそれぞれのtensor名・shape・FP32入出力をdeserializeして検証する。

Notebook GUIは「ONNX export」画面の「NotebookでTensorRT build・速度計測」で
モデルとFP32/FP16を選択する。既定FP32。`build_engine --precision fp32|fp16`も共通。
FP16は`trtexec --fp16`、FP32は`--noTF32`を使用する（本経路は従来のTensorRT精度フラグ向け）。
精度をreport.jsonとUI履歴に保存し、異なる精度のNotebook engineは別成果物へ保存する。

Jetsonは次でモデル選択→build精度選択の順にfzfを表示する。

```bash
bash scripts/e2e_trt.sh
# モデルを指定して精度だけ対話選択
bash scripts/e2e_trt.sh /workspaces/models/policy
# 自動実行や精度を固定する場合、従来フラグで精度選択を省略
bash scripts/e2e_trt.sh /workspaces/models/policy --fp16
bash scripts/e2e_trt.sh /workspaces/models/policy --fp32
```

モデルのengine保存先は従来どおり1つ。要求精度と検証済みreportが一致すると再利用し、
異なる精度を選ぶと再buildする。新engineの検証成功後に置換し、失敗時は旧engineを保持する。
CLIの精度メニューの先頭はFP32。`--force`で同じ精度も再buildできる。
ROS TensorRT nodeの入出力設定は変更不要。学習AMPからbuild精度を暗黙に決めない。
ローカルテストでメニュー順・flags・精度記録・cache判定を確認。実GPU/Jetsonのbuildは別途検証する。

### INT8校正とFP32比較benchmark

GUIの「ONNX export」→「NotebookでTensorRT build・速度計測」で元のFP32モデルと
「INT8 · 校正＋FP32との比較」を選ぶ。校正用・比較評価用データセットを選択し実行すると、
校正、FP32/INT8 engine build、同一入力のnative TensorRT比較を一括実行する。
RGB・20ch EVSに対応する。EVSはdatasetとモデルのmodel_spec一致を必須とする。
学習に使ったデータを校正に使ってよい。評価との重複も許可し、入力ファイルパスの重複を表示する。
未知データでの精度を評価する場合は別走行を使う。画像内容が同じ別ファイルは重複判定されない。

校正はONNX Runtime MinMax static Q/DQ、signed INT8・zero-point=0、activation per-tensor、
weight per-channel。静的重みを持つConv/Gemm/MatMulを対象とし、head、attentionのactivation間MatMul、
正規化、Softmaxは対象外。全layerのINT8化や速度向上は保証しない。
INT8 buildはQ/DQを使い、浮動小数点層に`--fp16`を許可する。比較元は`--noTF32`のFP32。
どちらも外部入出力はFP32。`--int8`だけで未校正モデルを量子化する経路は用意しない。

| CLI引数 | 既定値・意味 |
|---|---|
| `--model` | 必須、元のFP32 ONNX bundle |
| `--calibration` | 必須、校正用dataset |
| `--evaluation` | benchmarkでは必須、比較用dataset |
| `--output` | 必須、新規出力ディレクトリ（上書き不可） |
| `--calibration-samples` / `--evaluation-samples` | 各256、seedによる非復元抽出の上限、1〜100000 |
| `--seed` | 42、抽出seed、0〜2147483647 |
| `--warmup` | 10、各engine各passの事前実行、1〜1000 |
| `--repeats` | 3、同じ入力を反復、1〜20 |

```bash
# GPUのkart Docker。Pythonにはtorch/Pillow/onnx/onnxruntime/TensorRT/CUDA runtimeが必要。
python -m kart_e2e.int8_benchmark \
  --model /workspaces/models/fp32-policy \
  --calibration /workspaces/e2e/datasets/2026-10-19/10-00-00/train \
  --evaluation /workspaces/e2e/datasets/2026-10-19/11-00-00/eval \
  --output /workspaces/models/int8-policy
```

`ros2 run kart_e2e e2e_int8_benchmark`も同じ引数。ROSノードではなくオフラインCLIで、topic入出力なし。
CLIはJetsonでも利用可能だが、RGB前処理には学習と同じtorchが必要。通常はNotebookで校正する。
GPUがなければ失敗し、CPU速度へ暗黙fallbackしない。
校正のみなら`python -m kart_e2e.int8_quantize --model ... --calibration ... --output ...`
（または`ros2 run kart_e2e e2e_int8_quantize`）。追加引数`--samples=256`、`--seed=42`。
こちらはCPUで実行可能だがTensorRT精度・速度比較は行わない。

benchmarkはbatch=1で同じ実入力を順に与える。passごとにFP32/INT8の順を入れ替え、
一度に一つのengineをGPUへロードする。前処理・warmup・buildを除き、
CUDA eventによる演算時間と、H2D＋実行＋D2H＋同期のwall時間の平均・中央値・P95を保存する。
指令と生出力の教師MAE/RMSE/P95/MSE、INT8−FP32誤差差分、予測差、速度比、engineサイズを記録。
steer-onlyのthrottleはモデルmetadataの固定値。結果は閉ループ走行評価ではない。

UI出力は`e2e/models/<日付>/<時刻>/<名前>/`。元ONNXを残し、rootに校正済み`model.onnx`とmetadata、
`benchmark/report.json`・`predictions.csv`・`fp32/model.plan`・`int8/model.plan`・layer情報を保存する。
入力・モデルのSHA256、抽出行、GPU/driver/TensorRT情報も保持する。失敗・入力変更時は成果物を公開しない。
比較結果はUIに自動表示し、同じモデルライブラリからごみ箱へ移動できる。
Jetson転送にはrootのONNXとmetadataだけを使い、Notebookのengineは転送しない。
`bash scripts/e2e_trt.sh`で校正済みモデルを選ぶとINT8 buildを自動選択する（明示`--int8`も可）。
校正済みモデルに`--fp32`/`--fp16`を指定すると拒否する。元モデルは従来のFP32/FP16選択のまま。

検証範囲：CPUで実Q/DQ生成・ORT推論、契約、比較集計とCUDA呼出しのmockテスト。
実NVIDIA GPUでのTensorRT build・校正済み学習モデルの精度/速度・実車走行は別途確認する。

### 学習ボトルネックの診断ログ

RGB/20ch EVSの制御学習は、通常のGUI実行でもepoch・loss・速度を1行の進捗表示にまとめ、
実験ディレクトリに`timing.jsonl`（schema=`kart.training_timing.v1`）を保存する。
学習中はUI下部の「ログ・履歴」から確認・コピー可能。既存実行への後付けや過去runへの遡及記録は行わない。
新規ROS node/topicはなく、オフライン学習CLIのみの変更。

| 引数 / GUI設定 | 既定値 | 意味 |
|---|---|---|
| `--profile-batches` / 各phaseのCUDA計測batch数 | 20 | 各epochのtrain/validation・cache抽出で先頭N batchをCUDA Event計測。0〜1000、0でEventのみ無効 |
| `--log-interval` / 詳細計測ログの間隔 | 50 | 初回batch・N batchごと・phase終了時に詳細JSONLを保存。1〜100000。画面の進捗更新は最大0.5秒ごとで独立 |

学習画面の「学習速度の診断ログ」で変更する。CPU側時間は全batchを計測し、CUDAがなければ
GPU時間・VRAMを捏造せず空欄にする。Tiny第1段階の特徴蒸留は対象外。
通常の学習CLIにも上記引数を追加可能（`python -m kart_e2e.train ... --profile-batches 20 --log-interval 50`）。

JSONLには`configuration`（device/torch/CUDA runtime/GPU/CPU thread数）、`operation`、
`phase_start`、`progress`、`phase_end`を逐次追記する。
phaseごとのbatch数、実sample数、wall秒、samples/s、host各stageのms統計、
先頭N batchの`sampled_cuda_ms`、CUDA allocated/reserved/peak allocated bytesを含む。
統計はcount/total/mean/median/P95/maximumに加えfirst/mean_after_firstを保持し、初回と以後を区別する。
GPUが未完了のsampleはprogress集計に含めず、phase終了時に一度同期して確定する。
CUDAの計測device/streamは学習と同じ。stageごと・batchごとの同期は追加しない。
計測自体のEvent・ログ・phase終端同期の負荷はあり、計測なしの速度と厳密に同一とは限らない。

| ログのstage/項目 | 読み方 |
|---|---|
| `iterator_start` / `iterator_end` | DataLoader iterator作成・終端、worker起動/終了待ちの一部。epoch 1と以後を比較 |
| `data_wait` | consumerが次batchを受け取るまでの待ち。I/O・decode・resize・worker queue/pin処理を含み、workerのCPU時間やディスク速度そのものではない |
| `transfer` | CPU→device。host値はenqueue/待ちを含む。CUDA Eventのstream時間と別に見る |
| `encoder` / `head` | backbone特徴抽出 / 制御head。特徴量キャッシュ使用時は学習中のencoderを実行しない |
| `loss` | loss計算と既存のfiniteチェック。host時間には先行GPU処理の完了待ちも含む |
| `backward` / `optimizer` | 逆伝播 / AdamW・GradScaler。validationでは実行しない |
| `metrics` | loss.item/MSE.item/MAE集計。host時間には既存のGPU同期・転送待ちを含む |
| `cache_*_hash_inputs` / `cache_*_lock_wait` / `cache_*_total` | 入力ハッシュ、lock待ち、キャッシュ全体。totalのcache_hitで再利用を識別 |
| `cache_*_extract` の `download` / `write_features` | 特徴量をCPUへ戻す時間 / NPY mmap書込。flush・整合確認はcache totalにも含む |
| `checkpoint_save` | best.pt更新の保存時間とbyte数。best更新epochのみ |

host時間とCUDA時間は重複するため足し合わせない。CUDA Eventはstream上の経過時間で、
kernel単位のprofilerやGPU稼働率ではない。先頭N batchにはwarmupが含まれるため、firstと
mean_after_firstや次epochを比較して定常時を判断する。
例えばdata_waitが継続して大きければ入力供給側が候補、encoderのCUDA時間が大きければ特徴抽出側が候補。
loss/metricsのhostだけが大きい場合はGPU待ちの可能性があり、そのCPU処理が原因だとは断定しない。
実際のPNG/NPY読み込みとresizeの内訳や個別CUDA kernelの調査には追加のworker計測・profilerが必要。

`metrics.jsonl`には`train_seconds` / `validation_seconds` と各`*_samples_per_second`も保存する。
これらはphase時間で、モデル初期化・cache準備・checkpoint保存を含まない。
CPU学習・CUDA Event mockで検証し、実CUDAテストは対応GPUがある場合のみ実行する。

### RAM全量読み込み（学習の既定）

制御学習・Tiny第1段階の蒸留ともに`--ram-preload`が既定ON。
GUIの「学習データを最初にすべてRAMへ読み込む」で切り替える。
大きいデータセットではチェックを外すか、CLIに`--no-ram-preload`を指定し、従来の逐次読み込みを使う。
この切替は実行開始前の設定であり、実行中にモードは変えない。

- 全体fine-tuning／scratch／特徴量キャッシュOFF：前処理済みRGB FP32 tensor（3×128×224）または
  EVS FP32 counts（20×H×W）と教師をRAMに保持し、各epochの画像decode・resize・正規化・ディスク読込を省く。
  EVSのpolicy内固定正規化/paddingは従来どおりGPUで行う。
- 凍結encoder＋特徴量キャッシュON：必要なら最初に従来の逐次読込で特徴量を作り、
  学習に使う特徴量と教師をRAMへ全量コピーする。学習で使わない画像は重複保持しない。
- Tiny蒸留：学生入力と教師特徴量（教師cache ON時）をRAMへ保持する。
- 完全に同じtrain/validation入力・教師・前処理契約は同じRAM tensorを共有する。
  別datasetの部分重複画像までは統合しない。

RAMモードのDataLoaderは実効`num_workers=0`。前処理済みtensorをbatch化するだけのため、
複数spawn workerへ全量データを複製せず、全量分の`/dev/shm`拡張も要求しない。
GUI/CLIの`num_workers`要求値（既定8）は逐次モードと初回特徴量cache抽出に使う。
`run.json`へ`ram_preload`、`effective_num_workers`、`ram_preload_bytes`（共有分は一度だけ）を保存する。
進捗ログに全量読込のsample数・推定MiB・利用可能MiB・実効worker数を表示する。
制御学習では`timing.jsonl`にも`ram_preload` / `ram_preload_reuse`の準備時間を残し、
学習phaseのsamples/sとは分ける。

RGBは前処理後の1画像あたり約0.328 MiB（教師を除く）。ディスク上のPNG圧縮サイズからRAM量を見積もらない。
利用可能RAMとLinux cgroupの残容量の小さい方を確認し、割当予定がその80%を超える場合は停止して
RAM OFFを案内する。20%はモデル・batch・OS等の余地。確認できないOSでは容量チェックはbest effortで、
他processとの競合やOOMを完全には防げない。自動で逐次モードへは切り替えない。

CPUでPNG/NPY/EVS・特徴量の逐次/RAM入力一致、後続I/O不要、同一dataset共有、容量チェックと
設定ルーティングをテストする。64GB実環境での最大容量・GPU速度向上は別途実測する。

### 学習進捗を1行にまとめる

制御学習・Tiny蒸留の画面進捗はepoch/総epoch、train/validation、batch進捗bar、
累積平均loss、samples/s、経過・ETAを1行で更新する。GUIではMSEと最新data waitも表示する。
TTYでは改行せず上書きし、狭いterminalでは幅内に省略する。追加のGPU同期やloss.itemは行わない。
epoch完了後にtrain/validationのloss・MSE・速度、epoch時間、LR（制御学習）、best保存の
要約を1行残す。従来の大きいmetrics辞書や`TIMING ...`の詳細stage列は画面に繰り返さない。

非TTYの標準出力は`[progress] `で始まる読みやすい進捗snapshotを出力する。
GUIはこの行を履歴から分離し、現在の進捗だけを固定欄で更新する。epoch要約後は固定欄を消す。
警告・例外・通常ログは省略せず、raw job logも保持する。コピーは表示中の要約＋最新進捗を対象とする。
診断用`timing.jsonl`、学習指標`metrics.jsonl`は従来どおり保持し、詳細はそちらで確認する。
画面更新は最大0.5秒ごと。`--log-interval`は詳細JSONLの記録間隔で、画面更新間隔とは分離する。

### TensorRT向けINT8 biasの扱い

ONNX RuntimeのQ/DQ変換は`QuantizeBias=False`を明示し、Conv/Gemmのbiasを元のFP32のまま保持する。
ORT既定のINT32 bias＋DequantizeLinearはORTで実行できてもTensorRT 10.16.1がparse時に拒否するため使用しない。
生成後にQ/DQのzero-pointがINT8かつ0、scaleが正の有限FP32、定数DQ入力がINT8であること、
元のFP32 biasが値を含めて保持されることを検査する。metadataのquantization.biasへ`fp32_unquantized`を保存する。
これはFP32入出力やactivation/weightのINT8校正を変更せず、biasだけを浮動小数点に残す。
旧形式のINT8 ONNXは元のFP32モデルから再校正する。再学習・元ONNXの再exportは不要。
CPUで旧INT32 bias DQの生成・早期拒否と修正後のConv/ViT-S/Tiny＋ORT推論をテストする。
TensorRTのparse/build成功は対象NVIDIA GPU上で別途確認する。

### 学習パラメータの一括比較

project rootの`bash scripts/e2e_sweep.sh --train <dataset>`でbatch/loss/schedulerの組合せを逐次学習する。
既定12条件・各100epoch、RGB ViT-S全体fine-tuning、同じ初期重み/seed/LR1e-5。
validation省略時はtrainと同じ。ONNX以降は実行しない。
条件ごとのbest.pt・metrics・timingと、最良epoch/MSE/時間のsummary、epoch予算別CSVを保存する。
`metrics.jsonl`の`epoch_seconds`・`elapsed_seconds`は当該epochのcheckpoint保存前までの時間。
通常の制御学習もSmooth L1・100 epoch・Plateau ONを既定とする。
Tiny特徴蒸留は独立した工程で既定10 epochのまま。比較手順は[モデル比較計画](../../../docs/e2e_model_comparison.md)を参照。
サイズ・pretrained/scratch・freeze/fine-tuning・Tiny蒸留予算の一括比較には
`bash scripts/e2e_model_compare.sh --train <dataset> --hours 6`を使用する。
既定は`--preset sizes`（S/B/Tiny × freeze/fine-tuningの6条件）。
Tinyは50 epoch蒸留した共通encoderを一度だけ作り、それぞれ新しい制御headから学習する。
従来のscratch・蒸留予算比較を含む11条件は`--preset full`。
`size_comparison.md/csv`にはサイズとMSE、同じ更新範囲のViT-Bに対する比率を記録する。
既存train/distill CLIを共用し、試走なしで各制御条件を直接100 epoch実行する。
時間試走が必要な場合だけ`--pilot-epochs 2`で有効化できる。
時間切れの途中結果と未実行条件を区別し、全条件の6時間内完了は保証しない。
実行・resume・出力仕様は[scripts README](../../../scripts/README.md#モデルサイズ事前学習蒸留の時間予算付き比較)を参照。
実行例・resume・全設定は[scripts README](../../../scripts/README.md#学習条件の一括比較)を参照。


## 共通復帰の任意接続

bringupの`config/recovery/launch.yaml enabled: true`かつdrive_enabled=trueの場合のみ、
RGB/EVS共通のinference launchがdecoderの/auto/control_cmdを/recovery/input_cmdへremapする。
/recovery経由で既存muxへ接続し、decoderの入力鮮度・STOP要求は維持する。
観測専用drive_enabled=falseでは復帰nodeを起動しない。
復帰はVSLAM不要。使用画像とIMUはE2E入力とは別に有効化しておく。
[復帰設定と検証範囲](../kart_recovery/README.md)を参照。

bringup.shでは`--recovery off|infra_imu|rgb_imu|evs_assisted`または対話メニューで今回の復帰構成を選択する。
RGB/EVS E2E launchの`recovery`引数（既定空文字）はinferenceへ渡り、未指定時はYAMLを保持する。

## Section別head（共有backbone・同時学習）

section→head対応は多対一。既存HDMapの基準ラインの距離[m]で区間を明示指定する。
`config/examples/section_assignments.json`をコピーし、heads、sections、距離を実コースに合わせる。
サンプル例の15mは仮の値であり、そのまま実コースには使わない。
`[start_m,end_m)`、閉路のみstart>endで始点を跨げる。重複sectionは拒否し、未定義区間は不採用。
`base_frame`はoffline・実走行で一致させる。HDMapとVSLAM地図は事前に同じ座標へ登録しておく。

ROS/Docker環境で、既存の教師データセットを`--clock header`で作成した後:

```bash
python3 -m kart_e2e.section_dataset routing \
  --map /workspaces/map/course/map.json --localization-map /workspaces/map/course/bundle \
  --lane lane_001 --line customline --assignments /workspaces/e2e/assignments.json \
  --output /workspaces/e2e/routing.json

ros2 launch kart_bringup e2e_sections_offline.launch.py \
  bag:=/workspaces/record/対象bag map_file:=/workspaces/map/course/map.json \
  map_dir:=/workspaces/map/course/bundle routing_file:=/workspaces/e2e/routing.json \
  trajectory_output:=/workspaces/e2e/poses.jsonl
# bag再生終了後Ctrl-C。保存先は新規ファイルを指定する。

python3 -m kart_e2e.section_dataset split \
  --dataset /workspaces/e2e/datasets/元データセット --trajectory /workspaces/e2e/poses.jsonl \
  --routing /workspaces/e2e/routing.json --output /workspaces/e2e/datasets/section付きデータセット

python3 -m kart_e2e.train \
  --train /workspaces/e2e/datasets/section付きデータセット \
  --validation /workspaces/e2e/datasets/section付き検証データセット \
  --section-routing /workspaces/e2e/routing.json \
  --repo /workspaces/python_ws/dinov3 --weights /workspaces/weights/dinov3/使用する重み.pth \
  --mode steer_only --finetune --learning-rate 1e-5 --epochs 50 \
  --output /workspaces/e2e/training/section_trial
```

学習は通常と同じAMP・RAM・凍結時の特徴キャッシュに対応。
headごとの逆頻度で復元抽出し、各epochの総サンプル数は元データ数を保つ。
全headのtrain/validationサンプルが必要。`sections/<section_id>/`にも個別manifestを生成する。
画像／countsは可能ならhard link、別filesystemならcopyして独立したデータセットを作る。
固定AUTO／通常MANUAL等の教師採用条件は元データセットを継承し、区間分けで復活させない。
`metadata.json`に元manifest、trajectory、routingのハッシュ・採用／除外数を記録する。
`metrics.jsonl`の`train_heads` / `validation_heads`にhead別サンプル数・MSE・ステアMAEを保存する。
通常の`validation_mse`は複数head時のみhead別MSEの平均、`validation_micro_mse`は画像数加重。

通常の`kart_e2e.export_onnx`で、共有encoderと全headを一つのONNXに出力する。
FP32入出力は`[1, head数 × (steer_onlyなら1、steer_throttleなら2)]`、head順はrouting.heads。
推論時もbackboneを一回だけ実行し、decoderが対応するheadの出力を採用する。
通常のTensorRT FP32/FP16 build経路を使う。INT8の複数head比較・GUIでのsection編集は未対応。
RGBは実走行launchまで接続。EVSは同時学習／exportまで対応するが、実走行の時刻対応は未対応で
launchが拒否する。RGBモデルもROS/GPU/実bag/実車での動作確認は未実施。

### `e2e_section_selector`ノード

Python独立プロセス。重い画像tensorは購読せず、CameraInfoのheader時刻でTFを照会する。

| 方向 | topic既定値 | 型 | QoS / 内容 |
| --- | --- | --- | --- |
| 入力 | `/realsense/color/camera_info` | `sensor_msgs/msg/CameraInfo` | SENSOR_DATA、画像時刻 |
| 入力 | `/localization/vslam/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable/volatile/10、局在確認 |
| 入力 | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | tf2標準、画像時刻のmap→base |
| 出力 | `/e2e/head_selection` | `kart_interfaces/msg/E2EHeadSelection` | reliable/volatile/10、画像毎の選択または無効理由 |

ROS標準のparameter_events/rosout以外の独自service/actionなし。
全parameterは`config/section_selector.yaml`、実運用はbringupの同名YAML。
`model_dir=''`（export内routing）、`routing_file=''`（明示時はoffline用routingを優先）、
`trajectory_output=''`（空は記録なし、指定は新規JSONL）、
`camera_info_topic='/realsense/color/camera_info'`、
`diagnostics_topic='/localization/vslam/diagnostics'`、`output_topic='/e2e/head_selection'`、
`diagnostics_timeout_s=1.5`（steady time秒）、`use_sim_time=false`。全て起動時設定。

routing JSONの閾値は`max_distance_m`（ラインからの最大距離）、`max_heading_rad`（方向差）、
`ambiguity_distance_m`（別区間が同程度に近いと不採用）、`max_jump_m`（距離上の飛びと曖昧区間の分離）、
`max_pose_age_s`（TF待ち上限・decoder受信watchdog）、`max_pose_skew_s`（offline画像との最大過去pose差）。
例の値は1m、1.2rad、0.15m、3m、0.3s、0.05s。実コースに合わせて固定して学習から使用する。
TFは最新値で代用せず画像時刻で問い合わせ、診断が未局在／異常／古い場合はvalid=false。

decoderの追加parameterは`head_count=1`、`routing_sha256=''`、`head_timeout_s=0.3`。
export metadataからlaunchが指定する。空ハッシュなら従来の単一head動作。
選択の到着前は直前の一致済み推論を既存の150ms watchdog範囲内で保持する。
無効選択・鮮度切れ・未一致のままwatchdog超過は停止し、正常モードへの切替で既存ラッチを解除する。
CameraInfoと画像のstampが一致すること、TF遅延がwatchdogを超えないことは実機で確認が必要。

portable検証: section境界・方向・地図不一致、dataset分割、混合head勾配、CPUのCNN 1epoch、
ONNX Runtime比較、画像stamp一致／境界を跨ぐ遅延推論／期限切れのC++テスト。

既存mapのcustomline速度区間をそのまま境界に使う場合、assignmentsのsectionを
`{"id":"s01","map_section":0,"head":"curve"}`のように指定できる。
`map_section`はmapの`custom_speeds`（または生成customlineの`speed_sections`）の0始まりindex。
距離の再入力は不要。headの意味は速度値から自動推定せず明示割当とする。
customline以外ではこの参照を拒否する。既存区間の重複や未定義の隙間は、routing作成時に確認する。

### 別走行から必要なsectionだけを採用する

固定スロットル＋人間のステアで収集した別bagから、カーブ区間と直線区間を選択できる。
通常の教師抽出を先に行い、固定収集のMANUAL復帰区間を除外する方針は維持する。

```bash
# 低スロットル走行から、必要なsectionだけを区間分け時に採用
python3 -m kart_e2e.section_dataset split \
  --dataset /workspaces/e2e/datasets/slow --trajectory /workspaces/e2e/slow_poses.jsonl \
  --routing /workspaces/e2e/routing.json --sections s02 s04 \
  --output /workspaces/e2e/datasets/slow_curves

# 区間分け済みの高速走行から、再度VSLAMを実行せず直線sectionだけを抽出
python3 -m kart_e2e.section_dataset extract \
  --dataset /workspaces/e2e/datasets/fast_labeled --sections s01 s03 \
  --output /workspaces/e2e/datasets/fast_straights
```

`split --sections`を省略すれば従来どおり全section。指定はsection IDでありhead名ではない。
未知のsectionは処理開始前にエラー。抽出件数0もエラー。
指定区間にデータがなかった場合は`metadata.json.missing_selected_sections`に記録する。
`selected_sections`、採用／除外件数、元manifestのハッシュを記録し、画像・countsと教師の
steering/throttle・head対応はそのまま保持する。元データは編集しない。
`extract`の出力は選択区間をまとめた一つのmanifestで、さらに`sections/`を再生成しない。

学習の`--train`は複数datasetを指定できるため、例えば
`--train .../slow_curves .../fast_straights`として混ぜる。
全データセットで同じroutingを使い、trainとvalidationのそれぞれで全headのデータを揃える。
`--mode steer_throttle`は実際に記録された固定スロットル値も教師にする。
現行missionの実走行は固定スロットルを維持する。
この抽出機能によって走行時のスロットル設定を自動変更しない。

### ワイルドカード割当 `*`：全周用head

専門headに加え、全周用headを`sections`へ距離指定なしで追加できる。

```json
{
  "heads": ["straight", "curve", "stable"],
  "sections": [
    {"id": "s01", "start_m": 0, "end_m": 5, "head": "straight"},
    {"id": "s02", "start_m": 5, "end_m": 10, "head": "curve"},
    {"id": "*", "head": "stable"}
  ]
}
```

上は割当部分の抜粋。完全な例は`config/examples/section_assignments_wildcard.json`。
`*`は一つだけ指定でき、距離／map_sectionは指定しない。
専門sectionの境界とは重複しない特別な割当として扱う。

全周用は通常の教師抽出済みdatasetを入力し、offline VSLAM／trajectoryなしで作成する。

```bash
python3 -m kart_e2e.section_dataset assign \
  --dataset /workspaces/e2e/datasets/全周の人間操舵データ \
  --routing /workspaces/e2e/routing.json --sections '*' \
  --output /workspaces/e2e/datasets/stable_full_lap
```

`assign`は各サンプルを`section_id='*'`とwildcardのheadへ割り当てる。
元dataset内の教師サンプルを全て採用し、ステア・スロットル・教師採用情報は保持する。
停止・復帰・調整の除外は前段の通常抽出で行う。raw bag全メッセージを教師にする意味ではない。
画像／countsを可能ならhard linkし、元datasetは編集しない。
header時刻／poseは不要で、metadataに`section_assignment=wildcard_no_localization`を記録する。
既存poseに基づく割当情報は採用せず、出力にtrajectoryの出典を付けない。

専門head分は従来どおり`split`と指定sectionの抽出で作成する。
同じroutingを使い、学習に`--train .../専門dataset .../stable_full_lap`を指定する。
validationにも全headのデータを用意する。head名とindexは全datasetで共通。

`split --sections '*'`と`extract --sections '*'`は「全ての既存区間を選択」であり、
既存headの割当を保持する。全サンプルをstableへ割り当て直す操作は`assign`を使う。
シェル展開を防ぐため`'*'`は引用符で囲む。個別sectionとの混在指定は拒否する。
位置判定の曖昧・局在失敗をwildcardへ自動fallbackする動作は追加していない。

### 学習UIからのsection工程

Map Studioの「学習 → Section head」で割当、offline pose、専門／全周dataset作成を行える。
既存の学習画面で複数datasetを選ぶとroutingを自動で渡し、全headのtrain/validationの存在を検証する。
`section_workflow`はUI内部workerであり、固定のrouting保存／offline起動のみを処理する。
独自ROSノードやtopicは追加しない。詳細なUI操作は[Map Studio README](../../../tools/app/README.md#section-headの学習ui)を参照。
