# sim bagとVSLAM推定の比較

`evaluate_vslam.py`はROSを起動せず、元のsim bagの`/sim/odometry`
(nav_msgs/msg/Odometry)と推定bagの`/visual_slam/tracking/odometry`
(nav_msgs/msg/Odometry)を比較する。必要なPython依存はNumPyとrosbags
（kart Dockerに導入済み）。任意で推定bagの`/visual_slam/status`も集計する。

## 記録と評価

Docker内で両ターミナルのROS environment / domainを合わせる。
他のsim、localization、bag playerは終了する。入力bagと出力bagは別directory。
録画名は毎回新しくする（ros2 bag recordは既存directoryへ書かない）。

ターミナル1で推定出力を記録開始:

```bash
source /workspaces/ros2_ws/install/setup.bash
ros2 bag record --use-sim-time -o /workspaces/record/vslam_sim_diagnostic_01 \
  /visual_slam/tracking/odometry /visual_slam/status
```

ターミナル2で対象bagを先頭から再生（VO）:

```bash
source /workspaces/ros2_ws/install/setup.bash
bash /workspaces/scripts/bringup.sh --mode eval --eval-localization fresh \
  --bag /workspaces/record/sim/sim_20261011_100606_3e4bd720 \
  --tracking-mode 0 --rate 0.5
```

入力bag終端到達後、ターミナル1をCtrl-Cで停止しmetadataを書き出す。
ターミナル2もCtrl-Cで終了。再生を巻き戻して同じ出力bagへ記録しない。

```bash
python3 /workspaces/tools/sim/evaluate_vslam.py \
  --truth-bag /workspaces/record/sim/sim_20261011_100606_3e4bd720 \
  --estimate-bag /workspaces/record/vslam_sim_diagnostic_01 \
  --output /workspaces/record/vslam_sim_diagnostic_01_report
```

出力は`summary.json`、時刻別の`samples.csv`、等縮尺XY軌跡の`trajectory.svg`。
既存の同名report fileは上書きする。入力bagは読み取り専用。

## 比較条件と解釈

- 使用時刻はheader stamp。保存/受信時刻は使わず、半速再生でも同じsim時刻で比較する。
- 両方のchild_frame_idが一致することを要求する。base_linkとcamera_linkの軌跡を混ぜない。
- 初回の共通時刻で完全なSE(3)（位置と3軸姿勢）を合わせ、scaleや全軌跡best fitは行わない。
- 真値を線形位置・短区間の正規化quaternion補間で推定時刻へ合わせる。
  真値gapが50msを超える補間は除外する。header時刻の重複・逆行は拒否する。
- 速度は両軌跡の同じ時刻での位置差分から比較する。Odometry twistの座標系の違いを持ち込まない。
  推定gapが100msを超える区間は速度集計から除外する。
- 位置RMSE/P95/max/終端誤差、姿勢RMSE、走行距離比、速度RMSE、最大推定gap、status件数を集計。
- 走行距離比はscaleの参考値。位置ノイズでも増えるため、それだけでscale誤差と断定しない。
- 比較は両bagの共通時間だけ。未対応件数・共通durationを必ず確認する。
  初回推定時点ですでに生じた誤差は初期整合で除かれる。
- status件数は推定bag全体。Successでも真値に一致する保証はない。
- 真値や録画clockを再配信せず、推定TFへ影響を与えない。

## 検証範囲

合成軌跡で初期座標変換の除去・2倍scale誤差の保持・時刻補間と非重複拒否を確認。
実際に生成したSQLite3 rosbagでheader時刻読み出しを確認（3テスト）。
対象ユーザーbag、Dockerでの出力記録、CUDA/cuVSLAM推定の実行は未検証。

```bash
python3 -m unittest discover -s tools/sim -p 'test_*.py' -v
```

## 模様付きmapの再生成

`python3 tools/sim/generate_feature_map.py`で`minicar_2026.json`を元に
`minicar_2026_features.json`と`maps/sim/textures/features_*.png`を再生成する。
NumPyと標準ライブラリのみ。PNG encoderも標準ライブラリで、追加画像依存はない。
固定seed 20261011、複数サイズのランダム長方形/L形パターンを各壁へ静的に割り当てる。
実行中の画像へ点を書き足す処理ではなく、世界の面へ模様を貼る。
