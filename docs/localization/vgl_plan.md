# VGL軽量モデルの移植方針

2026-10-08、JetPilotの現在のコード・文書を確認した結果。VGL runtimeはkartにはまだ未実装。

## 確認できたこと

- `scripts/mapping/create_map.sh`はFoundationStereoを既定low_resとする。これは深度推定でVGLモデルではない。
- VGLはALIKED + LightGlue。従来既定は1920×1200、軽量profileは424×240を明示指定する。
- `docs/vgl_tensorrt_profiles.md`には、配布ONNXが固定1920×1200で、設定のみの変更では軽量化しなかったという記録がある。
- `tools/aliked_workspace`はaliked-n16を424×240でONNX再出力し、TensorRT生成とshape検査を行うツール。
  READMEはGPU統合を未検証としている。今回も実機の配置モデル・速度・精度は確認していない。
- `scripts/lib/create_map_with_vgl.py`は公式姿勢生成後、明示モデルでVGL特徴・語彙・indexを新規生成する。
- `vgl_profile.json`にONNX identity、幅・高さ、生成状態を保存し、位置推定時に対応するモデルを選ぶ。
  x86 map生成とJetson runtimeは同じONNXを使うが、TensorRTエンジンは各GPUで生成する。

## kartへの適用

1. VSLAM単独の点群取得を先に完成させる。FoundationStereo・VGLをその必須依存にしない。
2. 次に保存済み公式出力の画像・姿勢からVGL生成を追加し、モデルprofileを地図と同梱する。
3. localizationではVGLが初期位置／再位置合わせhint、VSLAMが連続追跡とmap→odomを担当する。
4. ONNX identityと入力shapeの一致を検査し、モデル不在時は停止する。大きい既定モデルへfallbackしない。
5. 実機で再位置合わせ成功率・処理時間・GPUメモリを比較して軽量profileを採用する。

参照（JetPilot checkout）:
- `docs/vgl_map_model_profiles.md`
- `docs/aliked_model_compatibility.md`
- `docs/vgl_tensorrt_profiles.md`
- `tools/aliked_workspace/README.md`
- `ros2_ws/src/launch/jetpilot_system_launch/launch/vgl_model_profile.py`

## 走行時とJoy連携（後続開発）

VGLをオフライン地図作成だけでなく走行時の初期位置・再位置合わせにも使う。
Joy managerはボタンの立ち上がりでlocalization要求を送り、専用の位置推定側が処理する。
ボタン割当は既存MANUAL／STOP／録画操作と競合させず、実装時に決める。
連打の重複要求を拒否し、受付・実行中・成功・失敗・timeoutを状態として返す。
要求だけで走行モードを変えない。map→odomの更新時のplanner／制御側の扱いは結合時に設計する。
ユーザー指定によりVGL実装は後回し。実bag／実機検証はユーザー側で実施予定。
