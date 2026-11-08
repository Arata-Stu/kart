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

## YOLOv8s + ByteTrack（2D認識）

ホストで `bash scripts/repos.sh import` → `./scripts/dev.sh --build-local` で
公式decoder（ros-lyrical-isaac-ros-yolov8）とEigenを導入し、通常の
`bash scripts/build.sh --packages-up-to kart_bringup` でtrackerをビルドする。

重みの正本は `weights/racing/yolov8s/`。相手車両1クラス（index 0）の学習済みcheckpointを `opponent.pt` として置く。
学習用Ultralytics環境のGPUで固定入力・NMSなしのFP16 ONNXをexportする例：

```bash
yolo export model=weights/racing/yolov8s/opponent.pt format=onnx imgsz=480,640 half=True dynamic=False nms=False batch=1 device=0
```

対象GPUのkartコンテナ内で下記を実行する（スクリプトのパスはkartルート基準）。
前処理はRGB8→float32/255→NCHW。入出力はFP32、重みはFP16。
prepareは入出力名・shape・型・FP16重みを確認し、FP16入出力のexportには境界Castを追加する。
FP32入出力を保持したFP16 exportも使用可。TensorRT 11では型付きONNXからengineを生成する。

```bash
cd /workspaces
ENGINE=$(bash scripts/racing_trt.sh) || exit 1
ros2 launch kart_bringup racing_perception.launch.py engine:="$ENGINE"
```

引数engineは必須、image_topic既定は/realsense/color/image_raw
（sensor_msgs/msg/Image、best effort、640×480固定）。カメラまたはbagは別途起動する。
perception.yamlのformatはRGB8変換、to_tensorは0..1へのscale、planar/reshapeは
[1,3,480,640]へ変換する。trtのbindingsはimages→output0、batch=1、
厳密なshape照合、外部生成engineを使用。decoderはoutput0 [1,5,6300]、
num_classes=1、confidence_threshold=0.1、nms_threshold=0.45、入出力queue=1。
topicは/racing/detections（vision_msgs/msg/Detection2DArray、DEFAULT/reliable、
volatile、depth 1）と/racing/tracks（同型、best effort/volatile/depth 1）。
全ノードの採用値は起動時に表示。tracker.yamlの全項目は
[tracker README](../../../kart_opponent_tracker/README.md)参照。地面投影とFrenet状態生成は別launchで有効にする。

build入力は同じディレクトリの `opponent.onnx`。対象GPUのUUIDとTensorRT版ごとに
engineを保存し、GPU・driver・ONNX・buildスクリプト・engineハッシュとbindingsで再利用を確認する。
`--force` で再生成。buildログはplan隣のlast-build.log。TensorRT 10はstronglyTyped、
11以降は既定の型付きbuild。ByteTrack本体はcolcon時のadd_subdirectoryで同時にビルドされる。

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
report.jsonにbag、engine SHA、撮影時刻、bbox一覧を保存する。目視評価用。GT照合によるAP計算なし。

### 車体座標

`ros2 launch kart_bringup racing_ground.launch.py ground_z_m:=<実測値[m]>`
を認識launchと別に起動する。入力・出力とパラメータは
[opponent_ground](../../../kart_opponent_tracker/README.md)参照。
既存のカメラ校正に加え、base_linkからRGB光学frameまで連結した実測取付TFが必要。
flat-road推定点を/racing/ground_pointsへ出力する。2Dのみのbag評価はこのノードを起動しない。

### Foxglove と取付高さ

同一ROS domainで `ros2 launch kart_bringup foxglove.launch.py` を起動し、Imageパネルの
画像に/racing/rgb、Annotationsに/racing/detection_boxesを選ぶ。Sync timestampsを有効にすると
撮影時刻が同じ枠を確認できる。生検出のscoreは/racing/detections、追跡IDは/racing/tracksにある。
ground起動時は3Dパネルでframeをbase_link、topicを/racing/ground_markersにする。
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
`scripts/bringup.sh --perception /absolute/path/opponent.plan` またはTUI「認識 / 相手車」で有効化。
`--perception off` で明示無効。RGB有効かつ640×480が必要。
missionはRealSenseと同じsensor_containerをtarget_containerとして渡し、encoder・TensorRT・decoder・
ByteTrackの全componentをそこにロードする。単独起動でtarget_container未指定時のみ専用containerを作る。
車体座標投影は別起動（上記参照）。認識の有効化だけで車両への制御指令は生成されない。

### 追従・停止車両回避のshadow評価

コース地図・自己位置・認識を起動した状態で
`ros2 launch kart_bringup racing_ground_shadow.launch.py ground_z_m:=-0.033`
を起動する（後車軸中心の対地高さ33 mmの場合）。groundとracing_shadowの単独launchに代えて使う。
閉じた/planning/reference_line、撮影時刻TF、/racing/tracks、RGB CameraInfoが必要。
observer.yamlはracing_observerの位置誤差下限。停止・回避設定はpredictive_spliner.yaml。
出力は/racing/observationと/racing/shadow_plan。通路情報なしでは停止車両の回避許可を出さない。
