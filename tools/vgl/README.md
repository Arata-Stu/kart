# kart VGLモデル生成

JetPilotのcheckoutを必要としない独立した入口は`scripts/vgl_model.sh`。
公開ALIKED-TensorRTの固定commit `96f1f9e7a9932a5da4f0d1aa62017c0fd48141ce`から
`aliked-n16`を取得して再exportする。TopK=2048、FP32入力、opset=17。
ソースとライセンスはvendorへ保存し、重みSHA256を検証する。
コードは既存JetPilotのALIKED実験ツールから移植し、kartの配置・設定検証へ接続した。

## Notebookのkart Dockerで424×240を生成

```bash
cd /workspaces
source ros2_ws/install/setup.bash
bash scripts/vgl_model.sh prepare --source-only
bash scripts/vgl_model.sh doctor --stage export
bash scripts/vgl_model.sh export --name 424x240
bash scripts/vgl_model.sh build --name 424x240
bash scripts/vgl_model.sh inspect --name 424x240
```

`KART_VGL_PYTHON`既定は`/opt/inference/bin/python`。exportはCUDA対応torchと互換torchvision、
ONNX/NumPy/Pillowが必要。doctorは不足依存を表示し、PyTorchを自動で置換しない。
buildはROS環境とisaac_ros_visual_mappingの公式exporter、TensorRT Pythonが必要。

exportの幅/高さは既定424/240。`--width`/`--height`で変更できる。
`--image /path/to/frame.png`があればRGB bilinear resizeを使用し、省略時は固定seedの合成入力で変換する。
合成入力でのexport成功は数値精度や実際のcuVGL動作の検証ではない。
配布ONNXの入力名・shapeだけを書き換えず、サイズに依存する内部計算をソースから再出力する。

完成したUIのモデル指定先:

```text
/workspaces/models/vgl/424x240/runtime_models
  aliked_lightglue/
    aliked.onnx
    aliked_*.engine
    lightglue_aliked_*.engine
```

models以下なのでVGL地図追加生成のモデル候補へ入る。
manifestにはソース/重み/ONNXハッシュ・入力形状・export環境を記録する。
engine-inspection.jsonにはALIKEDの入力形状・TRT版・context memoryを保存する。
buildはALIKED入力`[1,3,240,424]`を検査してからLightGlueを生成し、最後に両engineのdeserializeを検査する。
既存export/buildは上書きしない。再試験は`--name 424x240-v2`など別名を使う。
失敗した生成フォルダは診断用に残す。LightGlue失敗後に再buildする場合も別の名前でexportする。

## NotebookとJetson

ONNXは転送できるがengineは実行するGPU側でbuildする。
Jetsonでは`models/vgl/424x240/`へ`aliked.onnx`と`manifest.json`のみを同じ配置で転送し、
kart Dockerで`build --name 424x240`を実行する。再export用のPyTorchはこのbuildには不要。
元の地図、配布モデル、`/opt/ros`を上書きしない。

## 任意の元サイズ比較

`prepare --reference <NVIDIA aliked.onnx>`は照合済みSHA256と一致する参照モデルを保存する。
未照合の新しい配布モデルは拒否する。通常の424×240生成では参照モデル不要。
元サイズ1920×1200を別名でexportし、`compare --name baseline --image <実画像>`で
参照ONNXと比較できる。比較はCUDA優先指定が既定で、不在時はエラーにする。
CPUを明示する場合は`--provider CPUExecutionProvider`。
特徴点は座標対応を取って比較する。単一画像比較だけでcuVGL互換性を保証しない。

## 検証

```bash
python3 -m unittest discover -s tools/vgl -p test_lab.py
```

パス逸脱拒否・ハッシュ・ソース改変拒否・buildの入力選択などを単体テストする。
GPU export/buildとbagによるVGL位置推定は実行先で別途確認する。
