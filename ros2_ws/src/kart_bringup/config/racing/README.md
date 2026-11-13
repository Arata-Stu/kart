# Racing shadow

RCカーと障害物の2クラスモデルを追加学習する場合は、Studioの「RCカー・障害物」を使う。
RGB bag抽出・手動bbox/negative確認・YOLOv8s学習・kart用ONNX exportの
手順は[RCカー・障害物検出の追加学習](../../../../../docs/perception_training.md)を参照。
完成した`opponent.onnx`は`racing`profile（num_classes=2、class 0=rc_car／1=obstacle、tracked_class_id="0"）を使う。
障害物は走路上の静止ダンボールや落とし物等。背景ダクトや常設境界を一律に障害物へ分類しない。
Studioからの転送後はJetson側で`bash scripts/racing_trt.sh --model racing --model-root <転送モデルdirectory>`を実行する。
入力ONNXと生成engineのcacheはこのdirectory配下へ置き、公式COCOや他の学習版と分離する。
`--model-root`未指定は従来の`weights/racing/yolov8s`。finetuned配下の生成planもTUIで選択できる。
2クラスでもByteTrack・既存の制御用路面投影はRCカーだけを扱う。
perception_groundは両クラスをmapへ投影する。認識起動だけで停止・回避制御は有効化しない。
live追従・追越しは別途`--overtake`で明示する（既定OFF）。モデルの選択・追加学習だけでは有効化しない。

`predictive_spliner.yaml`は`kart_racing::PredictiveSplinerNode`（node名predictive_spliner）の全独自parameter。
全既定値・意味・topic契約は[kart_racing README](../../../kart_racing/README.md)を参照。
`racing_shadow.launch.py`で明示起動する。既存drive/collectには自動追加しない。
独自launch引数なし。地図、認識、車両、制御は起動しない。
入力Frenet観測は下記observerまたは外部が供給し、出力はshadow専用。
実運用に接続する前に時刻付き軌道adapter、衝突監視、制動経路と実機検証が必要。

横移動距離は `transition_min_distance`（既定3.0 m）と `transition_time`（既定1.0 s）で調整する。
Studioでは「横移動の最小距離」「横移動の速度係数」。往路・復路共通で、
固定下限・速度上限×係数・曲率由来の下限の最大値から候補を生成する。
短縮しても車体掃引・境界・曲率の検査を省略しない。成功サンプルは既定値へ戻す。

## 公式YOLOv8s + ByteTrack（2D認識）

ホストで `bash scripts/repos.sh import` → `./scripts/dev.sh --build-local` で
公式decoder（ros-lyrical-isaac-ros-yolov8）とEigenを導入し、通常の
`bash scripts/build.sh --packages-up-to kart_bringup` でtrackerをビルドする。

既定は公式のCOCO80クラスモデル。追加学習・独自annotationなしで検出を開始できる。
検出は全80クラスを`/racing/detections`へ出し、ByteTrackへはcar（class_id="2"）だけを渡す。
RCカーをcarとして検出できる精度は未検証。人などの検出を相手車両観測へ混ぜない。

重みの正本は `weights/racing/yolov8s/`。公式の
[yolov8s.pt](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8s.pt)をその名前のまま配置する。
この配布物のSHA-256は`1f47a78bf100391c2a140b7ac73a1caae18c32779be7d310658112f7ac9aa78a`。
ホストのkartルートからの取得例：

```bash
curl -fL https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8s.pt \
  -o weights/racing/yolov8s/yolov8s.pt
```

Ultralytics導入済みの変換用Python環境で、固定640×480・NMSなしのONNXをexportする。
公式checkpointの重みは追加学習しない。`/opt/inference`にはUltralyticsを追加せず、変換用環境を分ける。
CPUでも変換できる基準コマンド（検証用Ultralytics 8.4.209、ONNX 1.21.0）：

```bash
yolo export model=weights/racing/yolov8s/yolov8s.pt format=onnx imgsz=480,640 \
  half=False dynamic=False nms=False batch=1 opset=17 simplify=False device=cpu
```

出力`yolov8s.onnx`を同じフォルダーへ置き、対象GPUのkartコンテナ内で下記を実行する。
Docker内の配置先は`/workspaces/weights/racing/yolov8s/`。
前処理はRGB8→float32/255→NCHW。入出力はFP32。
prepareは入出力名・shape・型を検査し、FP32の公式exportとFP16 exportの両方を受け付ける。
FP16入出力には境界Castを追加する。TensorRTの型付きbuildはONNXの型に従うため、
上記FP32 exportから生成したengineをFP16推論済みとは扱わない。

```bash
cd /workspaces
ENGINE=$(bash scripts/racing_trt.sh --model coco) || exit 1
ros2 launch kart_bringup racing_perception.launch.py engine:="$ENGINE"
```

既存イメージにByteTrack依存があれば、ROS workspaceを
`bash scripts/build.sh --packages-up-to kart_bringup`で再ビルド・sourceする。trackerの更新も必要。

引数engineは必須、image_topic既定は/realsense/color/image_raw
（sensor_msgs/msg/Image、best effort、640×480固定）。カメラまたはbagは別途起動する。
perception.yamlのformatはRGB8変換、to_tensorは0..1へのscale、planar/reshapeは
[1,3,480,640]へ変換する。trtのbindingsはimages→output0、batch=1、
厳密なshape照合、外部生成engineを使用。COCO decoderはoutput0 [1,84,6300]、
num_classes=80、confidence_threshold=0.1、nms_threshold=0.45、入出力queue=1。
topicは/racing/detections（vision_msgs/msg/Detection2DArray、DEFAULT/reliable、
volatile、depth 1）と/racing/tracks（同型、best effort/volatile/depth 1）。
全ノードの採用値は起動時に表示。tracker.yamlの全項目は
[tracker README](../../../kart_opponent_tracker/README.md)参照。表示用map投影は既定ON、
既存の制御用地面投影とFrenet状態生成は別launchで有効にする。

build入力は同じディレクトリの `yolov8s.onnx`。`engines/coco/<GPU UUID>-<TensorRT版>/yolov8s.plan`へ
engineを保存し、GPU・driver・ONNX・buildスクリプト・engineハッシュとbindingsで再利用を確認する。
`--force` で再生成。buildログはplan隣のlast-build.log。TensorRT 10はstronglyTyped、
11以降は既定の型付きbuild。ByteTrack本体はDocker作成時に固定版をビルド・インストールする。

| launch引数 | 既定 | 意味 |
| --- | --- | --- |
| engine | 必須 | 対象GPU用のTensorRT plan |
| image_topic | /realsense/color/image_raw | 640×480の入力RGB topic |
| target_container | 空文字 | 空なら専用containerを作成、指定時は既存containerへload |
| model_profile | 空文字 | plan隣のbuild.jsonからcoco/racing/opponentを取得。metadataなしはcoco。明示指定とmetadataが不一致なら拒否 |
| tracking_enabled | 空文字 | 未指定はtracker.yamlのbool値を保持（既定true）。falseは検出のみ、trueは検出＋選択車両クラスのByteTrack追跡 |
| map_projection | true | 表示専用perception_groundを起動。falseでも検出・追跡・既存制御用の投影は変更しない |

モデルに由来するnum_classesとtracked_class_idは選択したprofileから決める。
静的なdecoder設定はperception.yaml、tracker全項目はtracker.yamlに保持する。
build.jsonとengineのhash不一致は起動前に拒否する。新旧モデルのengineは同じcacheへ混ぜない。

### RCカー・障害物2クラスの学習済みモデル

Studioの新しいexportは`racing`profile。入力ファイル名は互換性のため`opponent.onnx`だが、
raw Detect出力は`output0=[1,6,6300]`で、rc_car（class_id="0"）とobstacle（"1"）を検出する。
`--model racing`でbuildし、`engines/racing/<GPU UUID>-<TensorRT版>/racing.plan`へ保存する。
launchはbuild.jsonからnum_classes=2とRCカーだけのtracked_class_id="0"を選び、
デバッグ画像にも両クラス名を渡す。旧1クラスの同名ONNXは別directoryに保持する。

### 相手車両1クラスの既存モデル

専用の学習済み`opponent.pt`も使用できる。同じ固定入力・NMSなし条件で`opponent.onnx`へexportし、
`ENGINE=$(bash scripts/racing_trt.sh --model opponent)`でbuildする。
出力は`engines/opponent/<GPU UUID>-<TensorRT版>/opponent.plan`。
同じlaunchへengineを渡すとmetadataからnum_classes=1・tracked_class_id="0"を選ぶ。
旧build.jsonでprofile情報がない場合は再buildするか、単独launchに`model_profile:=opponent`を明示する。
公式の80クラス重みを`opponent.pt`へ改名して専用モデルとして使うことはできない。

### rosbagで検出結果を見る

ROS workspaceをsourceした同じコンテナで、次を実行する。入力bagにカメラの生RGBまたは
CompressedImageがあればよく、Depth・CameraInfo・教師ラベルは不要。

```bash
cd /workspaces
ENGINE=$(bash scripts/racing_trt.sh) || exit 1
/usr/bin/python3 scripts/evaluate_racing_bag.py /path/to/bag analysis/racing-run01 --engine "$ENGINE"
```

評価器はローカルの未使用DDS domain（既定91、変更は--domain）で推論launchを起動する。
--topicはbag側のRGB名、既定/realsense/color/image_raw。先頭から--stride間隔で最大--frames枚
（既定1、120枚）。各フレームの撮影stampとframe_idに一致するdecoder出力を待つ。
新規出力ディレクトリのindex.htmlを開くと未検出フレームを含むbbox・score付き画像を確認できる。
report.jsonにbag、engine SHA、model_profile・class_names、撮影時刻、bbox・class ID・クラス名・scoreを保存する。
画像とHTMLにも実際のクラス名を表示する。`--model coco|racing|opponent`で外部engineのprofileを明示できる。
目視評価用。GT照合によるAP計算なし。

### 車体座標

以下のopponent_groundは追跡済みRCカーを扱う既存の制御・shadow用経路。
RCカーと障害物の地図表示には次節のperception_groundを使い、両者の入力・出力を混同しない。

`ros2 launch kart_bringup racing_ground.launch.py ground_z_m:=<実測値[m]>`
を認識launchと別に起動する。入力・出力とパラメータは
[opponent_ground](../../../kart_opponent_tracker/README.md)参照。
既存のカメラ校正に加え、base_linkからRGB光学frameまで連結した実測取付TFが必要。
flat-road推定点を/racing/ground_pointsへ出力する。2Dのみのbag評価はこのノードを起動しない。

### RCカーと障害物のmap表示

通常の認識ONでは`perception_ground`（package/executable: kart_opponent_tracker/perception_ground、
node: /racing/perception_ground）を別プロセスで起動する。`tracking_enabled=false`でも表示できる。
全クラス検出の`/racing/detections`（vision_msgs/msg/Detection2DArray）と
`/realsense/color/camera_info`（sensor_msgs/msg/CameraInfo、両入力sensor data QoS/depth 5）を撮影stamp完全一致で同期し、
選択した2種類のbbox下端をbase_linkの平面へ投影して、同じstampのTFでmapへ変換する。
画像自体は投影ノードにもFoxgloveにも送らない。topicのQoS・空配信・診断詳細は
[perception_ground README](../../../kart_opponent_tracker/README.md)参照。

| 出力topic | 型 | QoS | 内容 |
| --- | --- | --- | --- |
| /racing/map_points | kart_interfaces/msg/GroundDetections | best effort / volatile / depth 1 | header.frame_id=map、正常結果の撮影stamp、valid/reason、objectsに検出ID・元class_id・class_name・score・位置[m] |
| /racing/map_markers | visualization_msgs/msg/MarkerArray | best effort / volatile / depth 1 | map上の表示用marker。rc_carは緑#39dcb0、obstacleは桃#d16ba5 |
| /racing/map_diagnostics | diagnostic_msgs/msg/DiagnosticArray | reliable / volatile / depth 1 | 1Hz、入力・同期・校正・TF・対象0件・投影成否の診断 |

同期入力ごとに位置・markerを更新する。対象0件・全除外・入力不正・TF欠落では空の点とmarker削除を出す。
正常な結果だけ`valid=true`とし、空配列の`zero_detections`／`zero_selected`は正常な0件。
失敗・watchdogによる消去は`valid=false`と同梱`reason`を出し、1Hz診断の到着前でも0件と区別する。
元の撮影stampがないwatchdog消去では通知時刻を使い、撮影画像の結果として扱わない。
完全同期しない・入力途絶も0.1秒周期のsteady watchdogで監視し、鮮度0.5秒を超えた表示を消す。
markerのlifetimeは0.5秒。以前の成功位置を新しいstampへ付け替えて再配信しない。

`map_projection.yaml`が全運用parameter。独自parameterは起動時固定で、変更後は認識を再起動する。

| parameter | 運用値 | 意味 |
| --- | --- | --- |
| use_sim_time | false | 通常は実時計。bag時計を使う場合は関連ノードと揃える |
| projection_frame | base_link | 路面平面と距離上限の基準frame |
| target_frame | map | 表示用出力frame |
| ground_z_m | -0.033 | base_link原点から路面までのz[m]。後車軸高さ33 mmに基づく暫定値、実測調整必須 |
| max_range_m | 10.0 | 投影可能な距離上限[m] |
| confidence_threshold | 0.25 | 表示用投影に採用する検出score下限 |
| rc_car_class_id | "2" | 静的COCO基準値。launchはcoco="2"／racing・opponent="0"へ上書き |
| obstacle_class_id | "" | 静的値は無効。launchはracingのみ"1"、coco・opponentは空文字で無効 |
| max_objects | 100 | 1同期画像で扱う物体数上限 |

CameraInfoの校正・元画像の640×480 shape・取付TF・撮影時刻のmap〜base_link TFが必要。
COCOのbicycle=1を障害物へ読み替えない。map TFがないcollect等では点を作らず診断理由を出す。
単独認識launchの`map_projection:=false`で表示専用ノードを無効化できる。
2D画像ギャラリー評価はこの値をfalseにし、CameraInfoやmap TFを要求しない。
既存`/racing/ground_points`・`/racing/tracks`や停止・回避planner入力は変更しない。

### Foxglove と取付高さ

同一ROS domainで `ros2 launch kart_bringup foxglove.launch.py` を起動し、Raw Messagesパネルで
`/racing/detections`と`/racing/tracks`を選ぶ。class ID・score・bbox・追跡ID・撮影時刻を確認できる。
Tableパネルでは`/racing/detections.detections`と`/racing/tracks.detections`で物体ごとに1行を表示する。
帯域を抑えるため、通常は画像を配信せず、元RGB画像・CameraInfo・ImageMarkerはFoxgloveへ公開しない。
`/racing/rgb`は元の`/realsense/color/image_raw`をRGB8へ形式変換した640×480のPC内YOLO入力。
YOLO用の画像入力は維持し、Bridgeの公開対象から画像を除外する。
Raw Messages／TableにはCameraInfoは不要なので、旧Imageパネルは閉じるか削除する。
3DパネルのFixed frameとDisplay frameをmap、topicを`/racing/map_markers`にし、HDMapも有効化する。
通常は画像を送らず、緑のRCカー・桃色の障害物を表示する。Raw Messages／Tableの
`/racing/map_points.objects`でclass_name・元class ID・score・座標を確認できる。
表示されなければ`/racing/map_diagnostics`でCameraInfo・同期・校正・TF・全投影拒否を切り分ける。
既存groundを別起動した場合の`/racing/ground_markers`はbase_link出力でRC追跡のみ。
この旧経路をmap表示する場合は撮影時刻のmap〜base_link TFが必要で、旧groundのtarget_frameはbase_linkのままにする。
評価器と一緒に見る場合はbridgeも評価domain（既定ROS_DOMAIN_ID=91）で起動する。

base_link原点は後車軸中央、zは上向き。平坦路上で走行時の荷重・タイヤの状態にして、
地面から後車軸中央までの鉛直距離Hを測り、ground_z_m=-H[m]を指定する。
現リポジトリの取付設定は後車軸高さ33 mmを採用しており、その値なら -0.033。
表示用map_projection.yamlの初期値はこの採用値に基づく暫定値で、車両ごとの実測確認に代わるものではない。
車体→camera_linkの現値はxyz=(0.200,0,0.067)m、RPY=(0,0,0)で、向きは仮定値。
RGBレンズまでの並進・回転はcamera_link→RGB光学frameを含むTF連鎖を再利用する。
CameraInfo.header.frame_idを確認し、
`ros2 run tf2_ros tf2_echo base_link <RGB光学frame>` で確認する。
車両が水平なとき、光学中心の予測対地高さは H + TF並進z。床上で車体基準の前後左右距離が既知の点を
複数撮影し、投影誤差を確認する。遠方ほどpitch誤差の影響が大きい。

### 通常bringupからの認識

`mission.yaml`のperception_engine=""で既定OFF。collect/environment/drive/e2eの
`scripts/bringup.sh --perception /absolute/path/yolov8s.plan` またはTUI「認識 / 物体検出」で有効化。
`--perception off` で明示無効。RGB有効かつ640×480が必要。
起動時の`--perception-mode detect|track`で検出のみ／検出＋車両追跡を選ぶ。
未指定・空文字は`mission.yaml`の`perception_mode: track`を保持する。
COCOモデルではどちらも全80クラスを`/racing/detections`へ出し、trackだけcar（class_id="2"）を
`/racing/tracks`へ出す。detectではtracksは空配列で、検出枠の表示は継続する。
racingモデルでは両クラスを検出し、trackだけrc_car（class_id="0"）を追跡する。
地図走行の参照ライン追従を表すmissionの`tracking`とは独立した選択である。

対象GPUのkartコンテナ内、kartルート`/workspaces`からの例：

```bash
cd /workspaces
ENGINE=$(bash scripts/racing_trt.sh --model coco) || exit 1
bash scripts/bringup.sh --perception "$ENGINE" --perception-mode detect
```

用途に「地図走行」を選び、既存の地図・VGLモデル・追従ラインを選択する。
設定一覧の「認識 / 物体検出」でOFF／engineを、「認識 / モード」で検出のみ／検出＋車両追跡を
変更できる。`--mode drive`で非対話起動する場合も、通常の地図・モデル引数に
`--perception "$ENGINE" --perception-mode track`を追加する。
この選択は起動時に反映し、記録開始やAUTO要求は自動送信しない。
missionはRealSenseと同じsensor_containerをtarget_containerとして渡し、encoder・TensorRT・decoder・
ByteTrackの全componentをそこにロードする。単独起動でtarget_container未指定時のみ専用containerを作る。
表示用map投影は別プロセスで自動起動し、既存の制御用車体座標投影は別起動（上記参照）。
認識の有効化だけで車両への制御指令は生成されない。

### 推論と投影を切り分ける画像デバッグ

`mission.yaml`の`perception_debug_image: false`が既定。
`--perception-debug-image`／`--no-perception-debug-image`で明示ON／OFF、未指定はYAMLを保持する。
engine指定が必須。TUI「認識 / bbox画像」、Studioの起動設定でも切り替えられる。
ON時だけ`opponent_debug_image`を別プロセスで起動し、`config/racing/debug_image.yaml`を適用する。
認識launch単独では`debug_image:=true`を指定できる。

既に認識が動いているときは、同じROS環境の別ターミナルから次だけでも確認できる。

```bash
ros2 launch kart_bringup racing_debug.launch.py
```

このlaunchはカメラ・YOLO・車両制御を起動せず、デバッグノードのみを起動する。
通常bringupのdebug ONとの同時起動はしない。単独launch引数は`model_profile=coco`（`racing`・`opponent`も可）、
`image_topic=/realsense/color/image_raw`。統合起動では認識に指定したimage_topicを同じように渡す。
class_namesは選択したモデルの名前一覧で上書きする。
ImageパネルのTopicを`/racing/debug_image/compressed`にし、別annotation・Sync timestamps・RectifyをOFFにする。
CameraInfo不要のbbox描画済みJPEGで、緑は検出・青は追跡。既定640×480・最大5Hz・quality 70。
同じ撮影stamp/frameの結果だけを描画し、`DETECTIONS: 0`と`MISSING`を区別する。
画像入力待ち・途絶・不正画像では撮影画像と区別した`NO SOURCE IMAGE`画面になる。
ノード・全parameter・QoSは[kart_opponent_tracker](../../../kart_opponent_tracker/README.md)を参照。

路面投影を起動すると`/racing/ground_diagnostics`（diagnostic_msgs/msg/DiagnosticArray、1Hz）も出る。
`reason`と件数で次を切り分ける。未検出だけから推論失敗とは断定しない。

| reason | 意味・確認箇所 |
|---|---|
| waiting_tracks / tracks_stale | 追跡入力がない／途絶。認識・追跡起動と入力を確認 |
| waiting_camera_info / camera_info_stale | 車体内のCameraInfo未受信／途絶 |
| not_synchronized | tracksとCameraInfoの撮影stamp完全一致なし |
| zero_tracks | 追跡対象0件。画像の検出結果・car選別・track開始scoreを確認 |
| invalid_stamp / invalid_frame / invalid_image_shape / invalid_calibration | 撮影header・640×480/ROI/ビニング・内部校正条件不一致 |
| tf_unavailable | 撮影時刻のRGB optical→base_linkのTFがない |
| all_rejected | trackはあるが全投影を除外。画像下端の切れ・取付姿勢・路面高さ・距離上限等を確認 |
| projected | 接地点を生成済み。表示されなければmap〜base_linkのTFとFoxglove設定を確認 |

最新入力・同期の鮮度はmonotonicで1秒、入力停止時に前の成功を維持しない。
topic自体がない場合は投影ノード未起動または起動エラーも確認する。

### 追従・停止車両回避のshadow評価

コース地図・自己位置・認識を起動した状態で
`ros2 launch kart_bringup racing_ground_shadow.launch.py ground_z_m:=-0.033`
を起動する（後車軸中心の対地高さ33 mmの場合）。groundとracing_shadowの単独launchに代えて使う。
閉じた/planning/reference_line、撮影時刻TF、/racing/tracks、RGB CameraInfoが必要。
observer.yamlはracing_observerの位置誤差下限。停止・回避設定はpredictive_spliner.yaml。
出力は/racing/observationと/racing/shadow_plan。通路情報なしでは停止車両の回避許可を出さない。

### 検証範囲

`test_racing_models.py`は3種のモデル選択・metadata/hash不一致・2クラスの検出/追跡/デバッグ名の配線・既定設定を検証する。
`scripts/tests/test_racing_trt.py`は実ONNX graphの変換とshape検査、模擬TensorRTによるcache・build失敗保持を検証する。
`scripts/tests/test_evaluate_racing_bag.py`はCOCO/racing名・ID・scoreとレポートの時刻保持を検証する。
trackerのクラス選別はpackageのportable C++テストで検証する。
2026-11-13には公式PTのhashと80クラス名を照合し、上記FP32 ONNXを実際に生成した。
Ultralytics同梱bus.jpgを640×480へresizeしたCPU smoke testでperson・busを計5件検出し、
同じRGB/255入力のPyTorch出力とONNX Runtime出力を比較した。
bboxの最大絶対差は0.00156 pixel、class scoreの最大絶対差は6.98e-6。
環境はUltralytics 8.4.209／PyTorch 2.15.0／ONNX 1.21.0／ONNX Runtime 1.27.0。
ローカル結果は`analysis/racing/coco_cpu_smoke_2026-11-13/validation.json`。学習・AP評価は行っていない。
ROS結合・実bagでのdecoder推論・Jetson上のTensorRT build/速度・実車のRCカー検出精度は別途確認が必要。

## Live追従・追越しの起動

mission.yamlの`overtake_mode: "off"`が既定。driveで`--overtake off|shadow|active`を明示する。
TUIの「走行 / 追従・追越し」、StudioのJetson起動でも同じ選択を扱う。
有効化には認識engineと`perception_mode=track`が必要。ON選択時は車両追跡へ連動し、
認識OFF・検出のみへ変更するとovertakeをOFFに戻す。active選択は自動復帰をOFFにしてレビューへ示す。

- off: behaviorを起動せず、従来のHDMap→planning/reference_line→PP/PIDを保持する。
- shadow: 従来の参照配信を保持し、behaviorは同じ基線を購読して候補・判断だけを出す。
  COCO／旧opponentも使用できるが、障害物クラスはなく、activeの2クラス判断には使わない。
- active: racing2クラスengineだけを許可。HDMapをplanning/course_referenceへ変更し、
  behaviorだけがplanning/reference_lineを配信する。E2E・parking・固定AUTO・自動復帰との併用は起動前に拒否する。

認識やactiveを選ぶだけでコース全体を追越し許可にはしない。Studioで選択lineごとの
`passing_zones`（開始/終了WPとboth/left/right）を設定する。既定は許可ゼロ。
許可は生成形状のSHA-256に紐付け、再生成・編集後は旧許可を失効させて設定し直す。
exportと調整snapshotにも保持し、不一致はROS normalizeで拒否する。
TUI／Studioの起動レビューではovertakeモードと選択lane/lineを確認し、許可の設定は地図編集で行う。
schemaと周期跨ぎの意味は[kart_hdmap README](../../../kart_hdmap/README.md#ラインごとの追越し許可)参照。
Foxgloveの既存`/hdmap/markers`にも許可区間の太い色帯と区間ID/側ラベルを表示する
（both=水色、left=黄緑、right=橙）。型は従来のMarkerArrayのまま。

既存の地図・VGLモデル・lane・line指定へ、次を追加する（下記は候補表示）。

```bash
--perception /absolute/path/racing.plan --perception-mode track --overtake shadow --recovery off
```

`racing_behavior.launch.py`は`kart_racing/racing_behavior`（node名/racing_behavior）と
`kart_opponent_tracker/opponent_ground`（/racing/opponent_ground）を起動する。
通常missionはtracking.launch.pyから一度だけincludeし、別途ground/behaviorを重複起動しない。
behaviorの全運用parameterは`behavior.yaml`、confirmed RC groundは`ground.yaml`。
ground_z_mはmap_projection.yamlと同じ暫定-0.033 mで、両方を実測調整する。
behaviorの閾値・寸法仮定・停止条件・動的parameterの一覧は[kart_racing README](../../../kart_racing/README.md)を参照。
STOP・1秒停車中に調整できる設定は33項目。候補範囲`lookahead_m`は既定20 m（3..50 m）。
新しい`simple_pass_speed=2.0` m/s以下は周回学習を待たず候補評価し、採用後は同じ側の並走区間を保持する。
`prediction_hold_s=6.0`秒、速度不確かさ0.10 m/s、加速不確かさ0.05 m/s²、横方向拡大率0.03 m/s、
前後不確かさ上限1.5 m、区間方位変化上限0.25 radが初期値。
対象の見失い中は`COAST_PASS`、合流余裕待ちは`HOLD_SIDE`として表示し、合流は別途確認する。
予測保持は入力全体の通信断を許容せず、通路不成立や許可区間外への無制限な延長も行わない。

| behavior launch引数 | 既定 | 意味 |
| --- | --- | --- |
| mode | shadow | off/shadow/active。missionからovertake_modeを渡す。起動時固定 |
| map_file | 必須 | 選択済みHDMap。nodeへmodeと共に明示上書きする |
| perception_engine | 空文字 | 前提検証用engine path。shadow/activeで必須、activeはracing profileを要求 |
| use_sim_time | 空文字 | 未指定は各YAML維持。明示時はbehavior/groundへ共通適用 |

behaviorのbaseline_referenceはactiveで/planning/course_reference、shadowで/planning/reference_lineへremapする。
観測は/racing/map_points（全対象の有効なmap接地点）と/racing/ground_points（確認済みRC追跡）を分ける。
画像をネットワークへ送らず、元class ID・撮影stamp・接地点の推定という意味を保持する。

| 出力topic | 型 | QoS | 内容 |
| --- | --- | --- | --- |
| /racing/candidate_reference | kart_interfaces/msg/ReferenceLine | reliable / transient local / depth 1 | 候補参照。shadowでは車両追従へ接続しない |
| /planning/reference_line | kart_interfaces/msg/ReferenceLine | reliable / transient local / depth 1 | activeのみ、PP/PID用の唯一の参照配信 |
| /racing/behavior_path | nav_msgs/msg/Path | reliable / volatile / depth 1 | 選択経路の観測用表示 |
| /racing/behavior_markers | visualization_msgs/msg/MarkerArray | reliable / volatile / depth 1 | 候補・判断用marker |
| /racing/behavior_status | std_msgs/msg/String | reliable / volatile / depth 1 | 20Hz、最大128KiBの有界JSON状態 |

Foxgloveの3Dでbehavior_path/behavior_markers、Raw Messagesでbehavior_status/candidate_referenceを確認する。
通常の画像公開はOFFのまま。Studioのlive表示とparameter調整は同じ状態を使う。
`/racing/behavior/reset`（std_srvs/srv/Trigger）はSTOP・1秒停車・基線から0.15 m以内でfaultを解除し、AUTOを要求しない。
Foxgloveのclient write許可は追加しない。

activeだけ`control_overrides.yaml`をPPへ追加し、line_timeout_s/wall_timeout_sを0.4秒にする。
通常のpose_timeout_s=0.2秒は維持する。動的参照stampは新しい観測の最古stampで、再配信で入力期限を延ばさない。
開始後の入力異常は採用経路の速度0を保持し、起動待ちや不正経路では空参照を使う。
behavior停止時はPPの0.4秒timeoutで旧経路の保持を止める。
STOP要求は物理停止の確認ではない。起動でAUTO・録画を開始しない。
構成・前提拒否・単一参照配信・active専用timeoutとUI連動はportable testsで確認し、ROS結合・Jetson・実車は未確認。
