# E2E imitation learning設計

## 出力と既存制御への接続

DINOv3 ViT-S/16を共通encoderとして、ステア＋スロットルの2出力と
steer-onlyの1出力を選択する。modeは学習時に確定しcheckpointへ記録する。
steer-onlyの固定スロットルは推論時設定。固定スロットルは固定車速ではない。
ステアはROSの左正[-1,1]、スロットルは[0,1]。brake/reverse教師を混ぜない。
E2Eは直接actuator指令なので速度PIDを通さず、既存AUTO command muxへ接続する。

rule-basedとE2Eを同じAUTO topicへ並列publishしない。現在の切替はSTOP中の
launch入替で行う。中央の動的source selectorは後続タスク。
推論既定は`drive_enabled=false`で専用topicへ出力する。車両接続は明示指定。
AUTO判定・入力／mode鮮度・異常ラッチ・元入力stamp保持は両モード共通。
復旧時に自動再開しない。中立と物理停止は区別する。

## 責務・依存

`kart_e2e`内でデータ抽出、model/preprocess、training、ROS adapter、gateを分離する。
encoderは公式DINOv3の固定commitを`e2e.repos`で取得し、重みは別途用意する。
JetPilotの小型encoder／head構成を参考にし、RGB/EVSの入力契約はkart側で明示管理する。
TorchはPCでの学習・ONNX変換だけに使い、Jetsonの推論にTorchを導入しない。
ROS Pythonと異なるinterpreterを暗黙使用せず、同じinterpreterで依存を検証する。
RGB走行時は公式Isaac ROS画像encoder群・TensorRTNode・kart C++ decoderを同一containerへloadする。
moduleはload専用、親がcontainer所有を決める。外部containerはmultithreaded必須。
RGBは画像とCameraInfoの同一timestampを使い、制御指令まで元時刻を保持する。
RGB前処理はantialiasなしresizeへ統一し、パッチpaddingはONNX内の正規化後へ置く。
旧PIL antialias前処理のcheckpointはschema不一致として拒否し、再学習を要求する。
ONNX exportはchecker・ORT比較後にhash入りmetadataを保存し、launchで契約を検証する。
vehicle・sensor・localizationの所有は既存launchに維持する。

## 再現性と検証の境界

画像前処理を学習・推論で共有し、mode・前処理schema・encoder commitをcheckpointへ保存する。
公式重みはstrict loadし、重み欠落時のrandom fallbackは禁止する。
bag/header timestampは混用せず、対応許容時間とbagの収集意図で教師抽出する。fixed_configのみheaderがないためbag受信時計を使う。
train/validationは同一bag・同一データセットの使用を許可する（自動分割はしない）。
同一画像を共有する場合、validation指標は未知データへの汎化評価として扱わない。
推論・学習の単体テストに使うrandom weightsは
モデル配線確認のみで、走行精度やpretrained性能の検証ではない。
ROS結合・実bag・GPU性能・車両走行はそれぞれ別の検証段階として報告する。


## Notebook UI

Map Studioの独立「学習」画面から、dataset作成・学習・ONNX export・Jetson転送を行う。
CLIを正本としてUIは固定引数の組み立て・ジョブ管理・成果物一覧だけを担当する。
成功した成果物だけを原子的に公開する。同一bagを理由に学習開始を拒否しない。
転送はhash検証付きのONNX bundleのみ。TensorRT engineはtargetで生成し、UIで走行を起動しない。

## 起動モデルの契約

学習時のruntime.fixed_throttle/max_throttleをcheckpoint→ONNX metadata→decoderへ継承する。
TUIはoutput_mode・modality・model_specを検証し、RGBモデルではRGBなしを拒否する。
RGBはセンサとencoder/TensorRTが同一container、EVSは既存OpenEBのtensor_containerを使う。
E2EモードではVSLAM/VGLを起動せず、AUTOは引き続き利用者の操作で選択する。

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
新しいROSノードやtopicは追加しない。以下の2026-10-18実装で制御headとONNX/TensorRTへ接続する。
DSEC学習は整列RGBに合わせrectified座標、kart既定driverはraw sensor座標。
同じ画素座標イベント集合の集計規約は合わせるが、実カメラの校正差は別途評価する。


## EVS policy経路（2026-10-18）

RAW＋bagの操作指令／mode／TensorTimingから未正規化counts datasetを生成する。
既定40ms・10bin・[start,end)・positive→negative・整数sum pooling。
EventState固定統計はhistogram_version=2を必須とし、解像度と係数をdataset/checkpoint/ONNXに継承する。
RAW readerはtime shiftingを無効化してセンサ時刻を保持する。Tensor stampを教師時刻として
過去の人間の操作指令を対応し、stream/epoch不明・reset・重複・不完全窓を推測で補わない。
単一stream/epochは--rawで対応、複数の場合はmanifestでRAWごとの対応を明示する。

counts padding→固定正規化→EVS encoderのpatch tokens平均→制御headを同じpolicyとして学習/export。
蒸留encoderの既定は凍結、finetuneは明示指定。制御教師なしのencoderを走行policyとして扱わない。
EVS ONNX入力はinput_tensor FP32 [1,20,120,160]または[1,20,240,320]、制御出力はRGBと共通。
model metadataでmodalityを識別し、RGBのSPECを緩めず別契約で検証する。
JetsonはOpenEBの既存/event_camera/tensor_containerへ公式TensorRT＋既存C++ decoderをloadする。
RGB encoder・Torch・別の制御gate・別のAUTO producerは追加しない。
全OpenEB設定はモデルから明示生成して起動時に照合する。TensorRT buildは引き続き明示操作。
EVS E2EでRGB/Infraともに無効ならRealSense/IMUと空のsensor containerを起動しない。

CLIを実装の正本とし、EVS dataset/学習のWeb UI操作は今回追加しない。
RAW再構成はオンラインのpacket dropを再現できず、offsetは受信遅延を含む。
合成データ・ランダム重み・ORT比較を、実RAW/公式蒸留重み/実TensorRT/ROS/車両の確認と区別する。
操作手順・全CLI引数・追加launch構成はkart_e2e READMEのEVS節へ記載する。


## AUTO固定スロットルの記録

bringupのauto_fixed_throttleはAUTOで手動ステア＋固定スロットルを合成する。
MANUALは通常操作。正常なAUTO/MANUAL切替はSTOPを挟まず、異常後はSTOP再armを要求する。
E2Eモデルmetadataは変更せず、固定設定中はmuxがteleop入力をAUTOにも使用する。
録画と走行切替は独立。条件はteleop/fixed_config、操作はteleop/control_cmdへ記録する。
RGB/EVS抽出はbag全体のfixed_configを先に走査する。
kart.auto_fixed_throttle.v1のenabled=trueが1件でもあれば固定収集と判定し、
固定AUTOの人間ステアだけを採用する。MANUALの復帰・調整はbag全区間で除外する。
有効固定設定がない従来bagはMANUAL抽出を維持する。
固定AUTO候補は最新過去300ms以内のenabled/armed/AUTO設定と操作throttle一致を要求し、
AUTO連続観測のentry_neutral_s以内を除外する。未知・不正設定は拒否し、意図を推測しない。
判定policy、教師候補の採用・除外内訳、サンプルのteacher_sourceを出力に保存する。

## 収集用固定スロットルとE2E走行の分離

- collect/driveの`--auto-fixed-throttle`（Joy parameter `auto_fixed_throttle`）は手動ステア収集用。
- `--mode e2e`はAUTOでモデルのステア＋固定スロットル。収集用Joy固定は必ず-1へ上書きし、
  muxの`auto_use_teleop=false`で`/auto/control_cmd`を車両へ通す。MANUALは通常の手動操作。
- E2E専用CLI `--e2e-fixed-throttle`／mission引数 `e2e_fixed_throttle` は未指定なら
  metadataのruntime.fixed_throttleを使い、明示時は0..runtime.max_throttleの有限値に限る。
  TUIもE2E専用の入力欄を使い、収集用の固定選択を表示しない。
- missionはRGB/EVSの双方に`use_fixed_throttle=true`を渡す。モデルがsteer_throttleでも
  ステアだけを操舵に用い、駆動量は固定値。モデルの出力数・形式は書き換えない。
- decoderの新しい起動時parameter `use_fixed_throttle` はbool、ノード/YAML既定false。
  単独launchにも同名の任意引数を提供する。既存topic・QoS・watchdog・STOP処理は同じ。

```bash
bash scripts/bringup.sh --mode e2e --e2e-model-dir /workspaces/models/policy \
  --e2e-fixed-throttle 0.10
```

0.10は車速ではなく正規化した固定要求。起動だけでは走行せず、□でAUTO、△でMANUAL、×でSTOP。
C++ gateとlaunch転送・CLIをportable testで確認する。ROS結合・GPU推論・実車操舵は未確認。
