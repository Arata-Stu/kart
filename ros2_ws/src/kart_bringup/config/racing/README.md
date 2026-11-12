# Racing shadow

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
[tracker README](../../../kart_opponent_tracker/README.md)参照。地面投影とFrenet状態生成は別launchで有効にする。

build入力は同じディレクトリの `yolov8s.onnx`。`engines/coco/<GPU UUID>-<TensorRT版>/yolov8s.plan`へ
engineを保存し、GPU・driver・ONNX・buildスクリプト・engineハッシュとbindingsで再利用を確認する。
`--force` で再生成。buildログはplan隣のlast-build.log。TensorRT 10はstronglyTyped、
11以降は既定の型付きbuild。ByteTrack本体はDocker作成時に固定版をビルド・インストールする。

| launch引数 | 既定 | 意味 |
| --- | --- | --- |
| engine | 必須 | 対象GPU用のTensorRT plan |
| image_topic | /realsense/color/image_raw | 640×480の入力RGB topic |
| target_container | 空文字 | 空なら専用containerを作成、指定時は既存containerへload |
| model_profile | 空文字 | plan隣のbuild.jsonからcoco/opponentを取得。metadataなしはcoco。明示指定とmetadataが不一致なら拒否 |
| tracking_enabled | 空文字 | 未指定はtracker.yamlのbool値を保持（既定true）。falseは検出のみ、trueは検出＋選択車両クラスのByteTrack追跡 |

モデルに由来するnum_classesとtracked_class_idは選択したprofileから決める。
静的なdecoder設定はperception.yaml、tracker全項目はtracker.yamlに保持する。
build.jsonとengineのhash不一致は起動前に拒否する。新旧モデルのengineは同じcacheへ混ぜない。

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
画像とHTMLにも実際のクラス名を表示する。`--model coco|opponent`で外部engineのprofileを明示できる。
目視評価用。GT照合によるAP計算なし。

### 車体座標

`ros2 launch kart_bringup racing_ground.launch.py ground_z_m:=<実測値[m]>`
を認識launchと別に起動する。入力・出力とパラメータは
[opponent_ground](../../../kart_opponent_tracker/README.md)参照。
既存のカメラ校正に加え、base_linkからRGB光学frameまで連結した実測取付TFが必要。
flat-road推定点を/racing/ground_pointsへ出力する。2Dのみのbag評価はこのノードを起動しない。

### Foxglove と取付高さ

同一ROS domainで `ros2 launch kart_bringup foxglove.launch.py` を起動し、Raw Messagesパネルで
`/racing/detections`と`/racing/tracks`を選ぶ。class ID・score・bbox・追跡ID・撮影時刻を確認できる。
Tableパネルでは`/racing/detections.detections`と`/racing/tracks.detections`で物体ごとに1行を表示する。
帯域を抑えるため、通常は画像を配信せず、元RGB画像・CameraInfo・ImageMarkerはFoxgloveへ公開しない。
`/racing/rgb`は元の`/realsense/color/image_raw`をRGB8へ形式変換した640×480のPC内YOLO入力。
YOLO用の画像入力は維持し、Bridgeの公開対象から画像を除外する。
Raw Messages／TableにはCameraInfoは不要なので、旧Imageパネルは閉じるか削除する。
ground起動時は3Dパネルでframeをbase_link、topicを/racing/ground_markersにする。
地図に重ねる場合はFixed frameとDisplay frameをmapにし、HDMapも有効化する。
撮影時刻のmap〜base_linkまでのTFが必要。投影のtarget_frame自体はbase_linkのままにする。
評価器と一緒に見る場合はbridgeも評価domain（既定ROS_DOMAIN_ID=91）で起動する。

base_link原点は後車軸中央、zは上向き。平坦路上で走行時の荷重・タイヤの状態にして、
地面から後車軸中央までの鉛直距離Hを測り、ground_z_m=-H[m]を指定する。
現リポジトリの取付設定は後車軸高さ33 mmを採用しており、その値なら -0.033。
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
車体座標投影は別起動（上記参照）。認識の有効化だけで車両への制御指令は生成されない。

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
通常bringupのdebug ONとの同時起動はしない。単独launch引数は`model_profile=coco`（`opponent`も可）、
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

`test_racing_models.py`はモデル選択・metadata/hash不一致・既定設定を検証する。
`scripts/tests/test_racing_trt.py`は実ONNX graphの変換とshape検査、模擬TensorRTによるcache・build失敗保持を検証する。
`scripts/tests/test_evaluate_racing_bag.py`はCOCO名・ID・scoreとレポートの時刻保持を検証する。
trackerのクラス選別はpackageのportable C++テストで検証する。
2026-11-13には公式PTのhashと80クラス名を照合し、上記FP32 ONNXを実際に生成した。
Ultralytics同梱bus.jpgを640×480へresizeしたCPU smoke testでperson・busを計5件検出し、
同じRGB/255入力のPyTorch出力とONNX Runtime出力を比較した。
bboxの最大絶対差は0.00156 pixel、class scoreの最大絶対差は6.98e-6。
環境はUltralytics 8.4.209／PyTorch 2.15.0／ONNX 1.21.0／ONNX Runtime 1.27.0。
ローカル結果は`analysis/racing/coco_cpu_smoke_2026-11-13/validation.json`。学習・AP評価は行っていない。
ROS結合・実bagでのdecoder推論・Jetson上のTensorRT build/速度・実車のRCカー検出精度は別途確認が必要。
