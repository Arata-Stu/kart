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
AUTO判定・画像／mode鮮度・異常ラッチ・元画像stamp保持は両モード共通。
復旧時に自動再開しない。中立と物理停止は区別する。

## 責務・依存

`kart_e2e`内でデータ抽出、model/preprocess、training、ROS adapter、gateを分離する。
encoderは公式DINOv3の固定commitを`e2e.repos`で取得し、重みは別途用意する。
JetPilotの小型encoder／head構成を参考にするが、巨大な学習UIやイベント入力を移植しない。
TorchはPCでの学習・ONNX変換だけに使い、Jetsonの推論にTorchを導入しない。
ROS Pythonと異なるinterpreterを暗黙使用せず、同じinterpreterで依存を検証する。
走行時は公式Isaac ROS画像encoder群・TensorRTNode・kart C++ decoderを同一containerへloadする。
moduleはload専用、親がcontainer所有を決める。外部containerはmultithreaded必須。
画像とCameraInfoの同一timestampを使い、制御指令まで元時刻を保持する。
前処理はantialiasなしresizeへ統一し、パッチpaddingはONNX内の正規化後へ置く。
旧PIL antialias前処理のcheckpointはschema不一致として拒否し、再学習を要求する。
ONNX exportはchecker・ORT比較後にhash入りmetadataを保存し、launchで契約を検証する。
vehicle・sensor・localizationの所有は既存launchに維持する。

## 再現性と検証の境界

画像前処理を学習・推論で共有し、mode・前処理schema・encoder commitをcheckpointへ保存する。
公式重みはstrict loadし、重み欠落時のrandom fallbackは禁止する。
bag/header timestampは混用せず、対応許容時間とMANUAL条件で教師抽出する。
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
TUIはoutput_modeとmodel_specも検証し、RGBなしを拒否する。センサとencoder/TensorRTは同一container。
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
新しいROSノードやtopicは追加しない。ONNX/TensorRT接続と制御headは後続。
DSEC学習は整列RGBに合わせrectified座標、kart既定driverはraw sensor座標。
同じ画素座標イベント集合の集計規約は合わせるが、実カメラの校正差は別途評価する。
