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
