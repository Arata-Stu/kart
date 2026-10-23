# E2E notebook dependencies

ROS Pythonと同じバージョンの専用venvに`requirements.txt`を導入する。
PyTorchのCUDA wheel/indexは使用するPCのCUDA環境に合わせる。
例（ROSをsource済み、プロジェクトルートで実行）:

```bash
/usr/bin/python3 -m venv --system-site-packages .venv/e2e
source .venv/e2e/bin/activate
python3 -m pip install -r python_ws/e2e/requirements.txt
python3 -c 'import rclpy, torch, numpy, PIL; print(torch.__version__, torch.cuda.is_available())'
```

上記は完全lockではない。実験時の解決結果は`pip freeze`で記録する。
Jetsonの推論はIsaac ROS TensorRTノードを使用し、Torch・ONNX Runtimeを必要としない。
このrequirementsはPCでの学習とONNX export用で、Jetsonには導入しない。
学習／ROS推論のコード・実行手順は[kart_e2e](../../ros2_ws/src/kart_e2e/README.md)。
encoderの取得は`vcs import . < e2e.repos`、公式モデル重みは別途準備する。


EVS学習も同じTorch環境で`kart_e2e.train --modality evs`を使用する。
RAW dataset作成だけはOpenEB/Metavisionの`metavision_core.event_io`とROS Pythonが必要。
EVS Dockerで取得・構築したSDKをsystem-site-packages venvからimportできることを確認する。
PyYAMLはOpenEB全設定の生成用（ROS側python3-yamlも利用可）。
入力重みはEventStateのv2 histogram encoder bundle。RGB公式重みを直接指定しない。
EVS PilotNet風CNNは`--modality evs --architecture simple_cnn --initialization scratch`を使い、
重み・DINOソースは不要。dataset内の160×120用固定正規化metadataを使用する。
OpenEBの既定tensorとCNN入力はともに`[1,20,120,160]`。RGB版CNNは212×120を維持する。
[EVS実行手順](../../ros2_ws/src/kart_e2e/README.md#evs-rawから制御policyまで2026-10-18)。
