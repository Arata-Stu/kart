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

学習と同じ前処理・教師対応付けでMANUALの有効ラベル付き画像を抽出し、ONNX RuntimeのCUDAを優先して推論する（利用不可・初期化失敗は理由をログに出してCPUへfallback）。ONNX内部にpaddingが含まれるため入力の二重paddingはしない。モデルmetadataのmode・SHA256・固定/最大スロットルを検証し、不正値や範囲外出力は失敗する。steer-onlyは固定スロットルを使用、steer+throttleは予測値へmetadataの最大スロットルを適用する。AUTO切替・watchdog等のオンライン状態機械は再現しない。

`report.json`にMAE/RMSE/P95絶対誤差（正規化操作値）、抽出件数、スキップ数、モデルSHA256、実際のprovider一覧・CUDA fallback理由・推論平均・中央値・P95・最小・最大（前処理・最初の10回のwarmupを除外）、最大500点の表示用サンプルを保存。`predictions.csv`には全評価フレームの時刻・教師・生出力・上限適用値・推論時間を保存する。データは`e2e/evaluations/<評価名>/`へ公開し、一時画像は終了時に削除する。学習と同じbagでの評価も許可するが、未知のコースへの性能評価ではない。運転成功率やTensorRT実機レイテンシは別途検証する。

### Notebook TensorRT build

UI「rosbag評価」の「NotebookでTensorRT build・速度計測」は選択ONNXを`python -m kart_e2e.build_engine --model <bundle> --output <新規ディレクトリ>`で処理する。ROSノード・topicは追加しない。x86_64限定、`trtexec`はPATHまたは`/usr/src/tensorrt/bin/trtexec`から利用し、ONNXのSHA256/入出力契約を検証する。精度はtrtexec既定、形状は固定1×3×120×212。`--warmUp=500 --duration=3 --exportTimes=<file>`で合成入力の計測を行い、engine・timings.json・report.jsonを`e2e/engines/<name>/`へ保存する。reportはONNX/engineバイト数、ビルド＋計測合計秒、GPU情報、利用可能なcomputeMs/latencyMs/h2dMs/d2hMs統計、再現コマンドを含む。詳細TensorRTバージョン・GPU計測ログはUIジョブログに残る。未対応オプションやGPU不足は失敗として表示し、engineを成功公開しない。

Notebook engineはJetsonへ転送しない。既存の転送機能は引き続きONNXとmetadataのみを送信する。合成入力のTensorRT計測はrosbagでの精度検証ではなく、engineの数値一致も別途必要。

## E2E TensorRT事前build

Jetsonのkartコンテナ内で`bash /workspaces/scripts/e2e_trt.sh`を実行する。`/workspaces/models`以下の転送済みONNX bundleをfzfで選択する。直接指定は`bash /workspaces/scripts/e2e_trt.sh /workspaces/models/<モデル名>`、探索先変更は`--models-root <dir>`。`KART_TRT_PYTHON`既定は`/opt/inference/bin/python`。ROS起動・車両指令publishは行わない。

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
  状態は`/operation_mode/state`（`kart_interfaces/msg/OperationModeState`）。MANUAL限定、brake/reverse除外。
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
|--mode-topic|/operation_mode/state|MANUAL判定|
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
`fixed_throttle`、`drive_enabled`、`use_sim_time`（既存decoderと同じ意味）。
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
TUIの「AUTOスロットル」で通常／固定を選ぶ。collect/drive/e2eで利用可能、evalは不可。
固定AUTOのbagはAUTOとして記録されるため、MANUAL限定の既存教師抽出には含まれない。
ROS結合・実車動作は未確認。portable testで固定出力、直接切替、異常ラッチ、HOST handshakeを検証する。
