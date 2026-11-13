# Racing YOLOv8s

既定は追加学習なしの公式COCO80クラスモデル。
[公式yolov8s.pt](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8s.pt)を
`yolov8s.pt`として置き、固定640×480・NMSなしでexportした`yolov8s.onnx`をengineビルドに使う。
FP32・FP16のONNXを受け付け、ROSとの入出力はFP32に揃える。
公式PTのSHA-256: `1f47a78bf100391c2a140b7ac73a1caae18c32779be7d310658112f7ac9aa78a`。

Studioの追加学習は`rc_car`（class 0）・`obstacle`（class 1）の2クラス。
障害物はコース上の静止ダンボール・落とし物等で、背景ダクトや常設コース境界を一律に含めない。
ファイル名は`opponent.onnx`のまま、`racing_trt.sh --model racing --model-root <モデルdirectory>`でbuildする。
raw Detect出力は`[1,6,6300]`、検出は両クラス、追跡はRCカーだけ。
モデルの選択だけでは回避制御を有効化せず、live追従・追越しは別途`--overtake`で明示する（既定OFF）。
認識起動時の表示専用map投影は両クラスを扱い、RCカーは緑、障害物は桃色で描画する。
校正・同時刻CameraInfo・取付TF・撮影時刻のmap TFと実測路面高さが必要。詳細は下記の認識設定を参照。
旧相手車両1クラスの`opponent.pt`／`opponent.onnx`は`--model opponent`で維持する（出力`[1,5,6300]`）。
同名の新旧ONNXは別directoryに保持し、誤ったprofileでのbuildはshape検査で拒否する。
いずれもGit管理外。コンテナの標準パスは `/workspaces/weights/racing/yolov8s/`。
`engines/<coco|racing|opponent>/<GPU UUID>-<TensorRT版>/` 以下は対象機で生成される。
2クラスのengine名は`racing.plan`。COCOの`yolov8s.plan`・旧1クラスの`opponent.plan`とは混用しない。
engineと隣の`build.json`を一緒に保持する。launchとbag評価はこのmetadataからクラス構成を判定する。
[export・build・rosbag評価手順](../../../ros2_ws/src/kart_bringup/config/racing/README.md)を参照。
