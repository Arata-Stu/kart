# E2E runtime configuration — Isaac ROS release-5.0

運用正本。静的parameterはYAML、構成はlaunch.json、明示launch引数だけで上書きする。
各moduleは既存containerへloadするだけ。親e2e.launch.pyが必要に応じてcontainerを所有する。

|ファイル|対象|主な設定|
|---|---|---|
|launch.json|起動構成（ROS parameterではない）|kart_e2e_container / multithreaded / create=true、model_dir空、RealSense RGB topic|
|image_encoder.yaml|公式6 Component|212×120 resize、RGB、scale=1/255、ImageNet正規化、NCHW [1,3,120,212]|
|tensor_rt.yaml|公式TensorRTNode|image→control bindings、FP32入出力、batch1、1GiB workspace、strict dimensions、queue1|
|control_decoder.yaml|kart_e2e::ControlDecoderNode|固定スロットル0、上限0.2、期限0.15秒、50Hz、drive=false|

使用公式ソースはpackage version 5.0.0。対象apt binaryの実revisionは未確認。
- [DNN inference](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference/tree/c16ac2c8669051ae1f3c6d77684f91fb25756f21)
- [Image pipeline](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline/tree/375311f26670455843a3b1f690a1ad54079c6be0)

全ノード独自parameter＋QoSをYAMLへ列挙。ROS標準の自動宣言parameterはuse_sim_time以外省略。
旧版のNITROS format引数やnum_blocksなど、5.0で宣言されないparameterは追加しない。
公式image encoder launchの処理を組み立てるがcropは不要なので省略。
ImageTensorNormalizeNodeの出力はNHWC、PlanarでNCHWにし、Reshapeで名前／形状を固定する。
ResizeはImageとCameraInfoをExactTime同期し、keep_aspect_ratio=false / disable_padding=true。
パッチ用paddingは正規化後にONNX内で実施する。resize結果のGPU/学習側の画素差は未確認。

QoSはResize入力SENSOR_DATA best effort depth2、出力DEFAULT reliable depth1。
残りのencoderノードはDEFAULT reliable depth10（上流固定、queue parameter非公開）。
TensorRTは入出力DEFAULT reliable depth1。すべてvolatile。
encoder内部topicは`/e2e/resize/image`, `/e2e/rgb/image` (sensor_msgs/msg/Image)、
`/e2e/{image_tensor,normalized_tensor,planar_tensor,tensor_input}` (isaac_ros_tensor_msgs/msg/TensorList)。
`/e2e/resize/camera_info` (sensor_msgs/msg/CameraInfo)も発行する。
TensorRTは`/e2e/tensor_input`入力、`/e2e/tensor_output`出力。同じTensorList型。
旧`isaac_ros_tensor_list_interfaces`や`NitrosTensorList`ではない。

`force_engine_update=false`が既定。launchではtrueを拒否し、検証済みengineがない場合は案内して停止する。buildは別途`scripts/e2e_trt.sh`で実行する。
falseでcache再利用する際はGPU/TensorRTの互換性を確認する。
この上流commitの`enable_fp16`は名前と異なりkTF32を設定するため、既定false。
FP16最適化済みと主張しない。精度・latency検証後に別途最適化する。

launch引数（既定はすべて空、設定を保持）:
`model_dir`, `container_name`, `create_container`, `image_topic`, `camera_info_topic`,
`fixed_throttle`, `drive_enabled`, `force_engine_update`, `use_sim_time`。
model_dirからONNX/hashを確認し、model/engine pathと出力modeを明示的に解決・表示する。
画像前処理契約の破壊、metadata不一致、未知の起動構成キーはエラー。

[ノード全parameter・起動手順](../../../../kart_e2e/README.md)。


## EVS policy

- `evs_launch.json`: load先`/event_camera/tensor_container`、入力`/event_camera/tensor`。
  model_dirとevs_tensor_configは空で、実行時の必須指定。container/topicは現行direct pipelineに固定。
- `evs_tensor_rt.yaml`: 上記tensor_rt.yamlと同じ全parameter、入力tensor/binding名だけ`input_tensor`。
  FP32 raw counts [1,20,120,160]または[1,20,240,320]をmodel metadataで検証する。
  normalization/paddingはONNX内。image_encoder.yamlは使わない。
- decoderは既存control_decoder.yamlを共有し、runtime/output_modeをモデルから継承。
- sensorの全OpenEB YAMLは`python -m kart_e2e.evs_contract`で明示生成し、
  `evs_tensor_config`で指定する。通常の既定212×120 tensor設定へEVS policyを接続しない。
- `evs_e2e.launch.py`は既存OpenEB processへTensorRT/decoderだけをloadする。
  全引数はmodel_dir、evs_tensor_config、container_name、input_topic、fixed_throttle、use_fixed_throttle、drive_enabled、use_sim_time。
  既定空＝JSON/YAML保持。model_dirとevs_tensor_configは必須。
- 内部inference.launch.pyのinput_topicは未指定時RGBの/e2e/tensor_input、EVSでは/event_camera/tensor。
  出力/e2e/tensor_outputとdecoderのtopic/QoSは共通。decoderのuse_fixed_throttle（bool、既定false、起動時固定）はmission e2eでtrueにする。service追加なし。
- 詳細手順・検証範囲は[kart_e2e README](../../../../kart_e2e/README.md)を参照。

decoderの`image_future_tolerance_s`は0.005秒（起動時固定、範囲0..0.05）。
数msの画像stampの未来ずれを許容し、input_timeout_s=0.15の過去期限は維持する。
