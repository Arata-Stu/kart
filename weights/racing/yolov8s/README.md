# Racing YOLOv8s

このディレクトリへ相手車両1クラスの学習済み重みを配置する。
`opponent.pt` は学習・export用、`opponent.onnx`（FP16、640×480、NMSなし）はengineビルドの入力。
いずれもGit管理外。コンテナの標準パスは `/workspaces/weights/racing/yolov8s/`。
`engines/<GPU UUID>-<TensorRT版>/` 以下は対象機で生成される。
[export・build・rosbag評価手順](../../../ros2_ws/src/kart_bringup/config/racing/README.md)を参照。
