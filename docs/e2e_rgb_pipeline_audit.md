# RGB E2E pipelineの確認（2026-10-19）

対象: kartの現行ソースと、設定コメントに固定したIsaac ROS 5.0のNVIDIA公式ソース。
症状は「走行は継続するがステアが合わずコースを外れる」。実機接続・実機bag・使用中重みは未確認。

## 確認結果と限界

|項目|学習側|ROS推論側|判定|
|---|---|---|---|
|色順序|RGB|ImageFormatConverterでrgb8|設定上一致|
|サイズ|212×120、bilinear、half-pixel、antialiasなし、uint8丸め|ResizeNode linear、aspect保持なし、paddingなし|補間の意図は一致、CV-CUDA実出力は要計測|
|値域|uint8 / 255|ImageToTensorNodeのscale=trueは1/255を一度だけ適用|固定公式ソースと一致|
|正規化|(x-mean)/std、ImageNet RGB値|HWCをImageTensorNormalizeNodeで正規化|固定公式ソースと設定は一致|
|配置|CHW→batch|HWC→NHWC→NCHW→reshape [1,3,120,212]|固定公式ソースと設定は一致|
|padding|正規化後に左右6・上下4のzero pad|ONNX内で一度だけ同じpad|PyTorch/ONNX試験で一致|
|出力|tanh steering、sigmoid throttle|control tensorを読み、steeringはそのまま使用|ソース上の二重tanhや追加steering倍率なし|

車両bridgeには別途steering_scaleやtrimがあるため、decoder出力と実舵角の一致は別途確認する。
学習・export・GPU推論は同じ処理を意図している。現時点で二重正規化やRGB/BGRの設定ミスは見つかっていない。
ただし、正規化済み入力からのONNX数値一致だけではROS前処理を検証できない。
実機のaptバイナリが以下の固定ソースと同じ実装であることも未確認。単体テストを実機保証としない。

## 一次ソース

- [ImageToTensorNode](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference/blob/c16ac2c8669051ae1f3c6d77684f91fb25756f21/isaac_ros_tensor_proc/src/image_to_tensor_node.cpp): scaleは1/255。
- [ImageTensorNormalizeNode](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference/blob/c16ac2c8669051ae1f3c6d77684f91fb25756f21/isaac_ros_tensor_proc/src/image_tensor_normalize_node.cpp): rank3 HWCとして計算後NHWCを公開。
- [InterleavedToPlanarNode](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference/blob/c16ac2c8669051ae1f3c6d77684f91fb25756f21/isaac_ros_tensor_proc/src/interleaved_to_planar_node.cpp): NHWC→NCHWを実際に並べ替え。
- [ResizeNode](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline/blob/375311f26670455843a3b1f690a1ad54079c6be0/isaac_ros_image_proc/src/resize_node.cpp): 設定どおりCV-CUDA linear resize。

## JetPilotとの差の解釈

JetPilotのINTER_AREAとkartのlinearは、標準424×240→212×120の正確な1/2縮小では、
理想的にはどちらも2×2画素平均になる。方式名の違いだけを大きな失敗原因と断定しない。
JPEGによる画素変化や実装の丸めは別問題。backbone独自実装と公式実装、学習率・epoch、
教師対応（nearestと過去のみ）も異なるため、比較は同じデータ・重み・実画像で行う必要がある。

## 実機での切り分け

まず同一画像stampの元RGB・リサイズ後RGB・推論入力・推論出力を短時間記録する。
[package READMEの診断手順](../ros2_ws/src/kart_e2e/README.md) の `e2e_rgb_pipeline_check` を使用する。

1. 実リサイズ後RGB→再正規化と実tensorが不一致なら、ROS正規化/layout/インストール版を調べる。
2. ここが一致し、元RGBからの学習前処理と差が大きければ、resize/色順序/画像経路を調べる。
3. 同じ実tensorを入力したONNXとTensorRTの予測が違えば、engine・精度・モデル一致を調べる。
4. オフラインでも同じ誤操舵なら学習・教師データを調べる。一致して実車だけ外すなら遅延、
   camera姿勢、スロットル条件、操舵補正などを実測する。

実機版確認:

```bash
dpkg-query -W ros-lyrical-isaac-ros-image-proc ros-lyrical-isaac-ros-tensor-proc ros-lyrical-isaac-ros-tensor-rt
ros2 param dump /e2e_resize
ros2 param dump /e2e_to_tensor
ros2 param dump /e2e_normalize
```

推測に合わせて正規化やリサイズを変更せず、比較結果から修正箇所を選ぶ。
