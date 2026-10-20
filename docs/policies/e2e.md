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

E2E画像stampの未来ずれは、明示したimage_future_tolerance_s（既定5ms、許容範囲0..50ms）で
小さな時刻差だけを許容する。画像stampの上書きや過去期限の延長は行わない。
期限切れ・AUTO前の画像・非有限値・受信途絶・異常ラッチは従来どおり拒否する。

## RGBデータセットの保存形式

容量より抽出・読み込み速度を優先し、既定は圧縮なしuint8 HWC RGBのNumPy形式とする。
保存時に縮小・量子化・非可逆圧縮は行わず、既存の教師判定とモデル入力前処理を維持する。
CLIの明示指定でlossless PNGも提供し、既存PNGデータセットの読み込み互換を保持する。
学習・特徴量キャッシュ・オフライン可視化・ONNX評価は同じRGB loaderを使用する。
NPYのpickle読み込みは禁止し、dtype・shapeを検証する。可視化用PNGと学習入力の保存形式を区別する。


## RGB Tinyと特徴蒸留

ViT-Sと同じ固定DINOv3実装を用い、Tinyは12層・192次元・3 heads・patch16・storage token4とする。
教師はViT-Sをeval/frozenで固定し、同じ画像からのCLS特徴へ192→384の学習用射影で近づける。
第1段階は特徴蒸留、第2段階はTinyの192次元へ制御headを付け、人の操作で学習する。
蒸留の射影は圧縮のための学習補助であり、制御モデル・ONNX・実行モデルには含めない。
教師を動かすことで損失を下げない。教師と生徒の異なる次元の完全一致を前提にしない。
蒸留済みencoderのSPEC・教師hash・設定を保存し、モデル契約でSmall/Tinyを区別する。
既存Small checkpointの互換を保持し、EVS仕様へ暗黙にTinyを適用しない。

EVS 20chのNotebook GUIは既存CLIと同じ前処理・学習・export経路を使用する。
RGB/EVSを明示選択し、データセットのmodality不一致を学習開始前に拒否する。
EVS RAWはbag直下のRAWが1件かつ記録timingのstream/epochが1組なら自動対応する。
複数RAW・複数stream/epochは名前や時刻から推測せず、明示manifestで対応づける。
EVSの評価は作成済みデータセットを入力し、RGBのbag直接評価とは入力を分ける。

EVS RAW抽出はOpenEB Pythonバインディングに合わせたNumPy 1.26専用環境を用い、
NumPy 2の学習・ONNX環境から分離する。import成功だけをイベント読出しの互換性の証拠にしない。

RAWの局所的順序乱れは既定5msのbounded reorderで処理する。
窓終端＋許容幅まで先読みし、元時刻のまま窓を確定する。chunk内だけのsortでは確定しない。
許容幅を超える遅れ・負時刻は停止し、resetを並べ替えで隠さない。設定と観測最大遅れを保存する。

EVS特徴量可視化は学習済みcheckpointの実encoder/headから生成し、countsの固定正規化とpaddingを共有する。
表示用log圧縮をモデル入力へ適用しない。PCA・感度の表示座標からpaddingを除き、20ch一覧も保持する。
入力感度は未正規化countsに対するステア勾配としてRGBの正規化入力勾配と区別して記録する。

制御学習はSmooth L1（既定β=0.1）と従来MSEを選択可能とする。
実目的関数のlossと比較用MSEを混同しない。best checkpointは検証MSE、
既定ONのPlateau schedulerは通常モデルで検証loss、Multi-headでhead平均の検証MSEを使用する。使用LRと次epoch LRを明示し、
損失・scheduler設定を成果物へ保存する。Tiny特徴蒸留の目的関数と分ける。

RGB baseline比較は事前学習済みとscratchを初期化方法で明示する。
scratchはencoderも更新し、既定100 epoch・LR1e-4、重み読込と特徴量キャッシュを禁止する。
事前学習済みも既定100 epochとする。ViT-Bの公式factory・専用重みパスを使用し、
CNNは4段stride2 Conv/GroupNorm/GELU＋global meanと既存headを使う。
前処理・損失・評価契約を共通化し、architecture/初期化/seed/学習parameter数を保存する。
モデル間の比較はbag分割とテスト条件を揃え、学習途中と収束後の比較を区別する。

制御学習のAMPは要求精度と実効精度を分けて記録する。
autoはCUDA native BF16を優先、非対応時FP16＋GradScaler、CPUではFP32。
明示BF16/FP16の非対応環境を暗黙fallbackしない。forwardのみautocastし、loss/指標/保存parameterはFP32。
キャッシュはFP32で構築し、学習精度を変えても同じキャッシュ契約を保つ。
速度と精度の効果は対象GPUで計測し、CPU互換テストからGPU性能を推定しない。

TensorRTの内部演算精度は学習AMPと独立して明示選択する。
Notebook build UIはONNX export工程に配置し、FP16/FP32をreportと履歴に保存する。
Jetson build CLIはモデル→精度の順に選択し、明示--fp16/--fp32では精度メニューを省略する。
FP16でもFP32の入出力契約を維持・検証し、モデルごとの既存engineを精度差で誤再利用しない。

INT8は代表データによる対称signed Q/DQ静的校正とし、FP32入出力を維持する。
静的重みのConv/Linearを対象とし、制御head・activation間attention MatMul・正規化・Softmaxは除外する。
UIのINT8操作は校正と同一GPU・同一評価入力のFP32/INT8 native TensorRT比較を一体で実行する。
校正用と評価用を独立選択し、重複は禁止せず入力パスの重複をreportに明示する。
元モデルを上書きせずQ/DQモデルを別保存し、校正済みモデルをFP32へ戻したと誤表示しない。
精度差は教師とのMAE/RMSE/P95と予測差、速度はwarmup・前処理を除くCUDA event演算時間と
同期した転送込み時間を記録する。FP32はTF32無効、INT8の浮動小数点fallbackはFP16許可と明示する。
CPU ORTの数値preflightをGPU速度比較の代用にしない。対象GPUでのbuild/benchmark成功と
閉ループ実走行の成否を区別し、自動走行開始や量子化モデルへの自動切替は行わない。

制御学習の速度診断は通常実行から逐次ログを保存する。
DataLoader consumer待ちとCPU dispatch/同期待ち、CUDA stream Event時間を区別する。
CUDA sample数は各phase先頭20batchを既定とし、stageごとの同期は挿入せずphase終端で確定する。
初回worker/warmup、cacheのハッシュ・hit/miss・抽出、検証、checkpoint保存を学習本体と分ける。
host/CUDA時間を合算せず、consumer待ちからI/Oやresizeの原因を断定しない。
CPU/mockテストから実GPUのボトルネックや速度向上を推定しない。

学習用tensorは既定でRAMへ全量preloadし、GUI/CLIで明示的に逐次読み込みへ切替可能にする。
RGB/EVSの前処理済み入力、凍結時は特徴量キャッシュを読み込み、学習中のディスク読込を省く。
全RAMモードはDataLoader worker=0としてspawnごとの複製や巨大な/dev/shm領域を要求しない。
逐次モードと初回特徴量抽出は従来のworker設定を使い、要求値と実効値をprovenanceへ残す。
同一train/validation datasetはRAMを共有する。必要byte数と利用可能RAM/container headroomを事前確認し、
不足の予測時は逐次モードへの切替方法を表示して停止する。暗黙のモード切替は行わない。

学習画面の進捗と詳細診断を分離する。terminalは1行上書き、GUIは最新phase進捗を固定欄で更新し、
履歴へはepoch要約を残す。phase/epoch/batch/loss/速度/ETAを同じ行に明示する。
詳細stage統計はtiming.jsonl、全学習指標はmetrics.jsonlに保存する。
進捗表示のための追加GPU同期を行わず、警告・例外は隠さない。

TensorRT向けINT8校正ではbiasのINT32 DequantizeLinearを生成しない。
ORTのQuantizeBias=Falseで元のFP32 biasを保持し、Q/DQ dtype/scale/zero-pointとbias保持を生成直後に検査する。
ORTでの推論成功だけをTensorRT対応の根拠にせず、parser非対応形式をbuild前に拒否する。

学習条件の一括比較は同じ初期重み/seedと固定データを条件ごとに新しいprocessで学習する。
batch/loss/schedulerを独立した軸とし、異なるloss同士の比較は共通MSEを使用する。
同じtrain/validationも許可し、その事実を結果に明示する。最良epochとepoch予算別の到達精度・時間を残し、
上限近傍の最良値から収束を断定しない。学習のみでONNX以降へ自動進行しない。

## 位置に基づくsection別head

共有backbone＋複数制御headを一つのcheckpointに保存する。sectionとheadは多対一で対応し、
straight/curve等のheadを複数sectionから共有できる。headごとの逐次finetuneは行わず、
headを均等にサンプリングした混合バッチで、対象headと共有backboneへ損失を逆伝播する。
検証は全サンプルを一度ずつ使い、best.ptはhead別MSEの単純平均で選ぶ。

sectionは登録済みmap上の基準ライン距離[start,end)で定義する。既存の速度区間を自動でheadへ
読み替えず、明示したsection→head対応とライン形状をrouting JSONへ固定する。
HDMapとVSLAM DBのハッシュを持ち、offlineラベル作成と実走行で同じ地図を使用する。
頭出し地点が違うodomをmap座標と見なさない。cuVSLAMの既存map局在成功・VO正常・
診断鮮度と、画像時刻のmap→base TFが確認できたサンプルのみ使う。
距離・向き・近接別区間の曖昧性・進行距離の飛びで不確かな判定は不採用とする。

ONNXはbackbone一回＋全headを出力する。decoderは画像と完全一致するstampの
E2EHeadSelectionとroutingハッシュで出力を選択する。選択不能・古い自己位置・地図不一致を
通常headへ暗黙fallbackしない。既存watchdogとSTOPラッチを維持する。
RGB sectionモデルのmission起動だけは、既存地図のVSLAM localizationを追加し、Pure Pursuitを
起動しない。単一headモデルのE2Eは従来構成のまま。EVSの複数headは学習可能だが、
RAW時刻とRGB CameraInfo時刻が一致しないため現段階の実走行launchでは明示拒否する。

section抽出は、複数の固定スロットル＋人間操舵の走行から採用区間を選ぶ用途に対応する。
`split --sections`はpose判定後に区間を絞り、`extract`は既存sectionラベルを再利用する。
教師採用条件・steering/throttle値は変更しない。section→headのroutingも元の全体定義を保持し、
選択sectionに応じてhead番号を振り直さない。複数走行を学習入力で合流させる。
画像gate・確率差による切替・安定policyへのfallbackは現段階では設計案であり未実装。
共有backboneの更新によって既存headの出力も変わるため、headの重み凍結だけを
安定policyの保存と見なさない。

全周用の追加headは`sections`の`{"id":"*","head":"stable"}`で明示する。
位置区間を持たないwildcard割当は一つに限り、通常sectionとの幾何的な重複として扱わない。
`section_dataset assign`は既存の人間教師datasetをpose不要で全採用し、そのheadへ割り当てる。
専門headのdatasetと共通routingで同時学習する。教師の有効区間の判定は前段の抽出を維持する。
通常split/extractにおける選択`*`は既存ラベルを保持した全区間選択とし、headの再割当はassignで明示する。
wildcard割当は実走行時の位置不明を許容する機能ではなく、gateのfallback実装とは分ける。

制御学習の共通既定はSmooth L1（β=0.1）・100 epoch・Plateau ONとし、
CLI・HTTP省略値・UIを揃える。epochは学習予算であり、収束の保証やearly stoppingではない。
Tiny特徴蒸留は別工程のepoch・目的関数を保持する。

モデル比較の時間予算は全datasetでの短い試走を含むwall timeとして扱う。
不足時に制御学習のepochを暗黙に減らさず、未実行・時間切れ・失敗を成功と区別して記録する。
蒸留と後段head学習は合計時間で開始可否を判断し、試走encoderを本番の初期重みへ流用しない。
比較は既存のtrain/distillを共用し、全条件の予算内完了や同一train/valの走行精度を保証しない。
サイズ優先比較は同じfreeze/finetune内でS/B/Tinyを比べ、Tinyは共通の本番蒸留encoderから独立に開始する。
初期重みの由来を表示し、蒸留Tinyと公式S/Bの差を純粋なarchitectureの効果と断定しない。
サイズ比・MSE比は成功した同じ更新範囲の基準モデルだけから計算する。
