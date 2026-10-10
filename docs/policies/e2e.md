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
