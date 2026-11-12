# Racing YOLOv8s

既定は追加学習なしの公式COCO80クラスモデル。
[公式yolov8s.pt](https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8s.pt)を
`yolov8s.pt`として置き、固定640×480・NMSなしでexportした`yolov8s.onnx`をengineビルドに使う。
FP32・FP16のONNXを受け付け、ROSとの入出力はFP32に揃える。
公式PTのSHA-256: `1f47a78bf100391c2a140b7ac73a1caae18c32779be7d310658112f7ac9aa78a`。

専用モデルは相手車両1クラスの`opponent.pt`／`opponent.onnx`を置き、`racing_trt.sh --model opponent`で選択する。
いずれもGit管理外。コンテナの標準パスは `/workspaces/weights/racing/yolov8s/`。
`engines/<coco|opponent>/<GPU UUID>-<TensorRT版>/` 以下は対象機で生成される。
engineと隣の`build.json`を一緒に保持する。launchとbag評価はこのmetadataからクラス構成を判定する。
[export・build・rosbag評価手順](../../../ros2_ws/src/kart_bringup/config/racing/README.md)を参照。
