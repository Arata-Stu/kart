# E2Eの操舵・走行条件分布の解析

単画像モデルの構造を変えず、教師データの偏りと評価走行の条件を比較するCLI。
衝突前の条件が未観測・少数だったか、指定した分布閾値を満たしていたかを調べる。
**閾値を満たすことは「学習に十分だった」「データ不足が原因ではない」の証明ではない。**
画像の見た目・障害物・照明の類似性やモデル推論誤差は本ツールでは評価しない。

## UIから実行

Map Studioの「分布解析」で入力の選択・実行・ヒートマップ/時系列の確認まで行える。
[UI手順](../app/README.md#e2eデータ分布のui解析)を参照。以下はCLIを利用する場合の手順。

## 入力

`manifest.example.json`をコピーして各bagと抽出済みdatasetを指定する。
パスはmanifestからの相対パスまたは絶対パス。bagは閉じたmetadata.yaml付きディレクトリ。

- `training`: bagと、そのbagから抽出したRGB/EVSのdataset（samples.jsonl、metadata.json、画像/counts）が必須。
  指定datasetの採用timestamp・教師操舵を使用し、bag全体の操作を教師だと仮定しない。
  train/validation splitやsamplingを自動再現しないため、実際の学習集合を調べるときは
  samples.jsonlをその集合に限定したdatasetを指定する。bagとの対応は利用者が明示する。
- `reference`: 良好な走行のbag。学習データとは別の比較基準であり、学習済みとは扱わない。
- `evaluation`: 評価走行bag。`events`で衝突等の時刻を明示できる。
- `mode`: `manual`または`auto`を各bagに必ず指定。固定AUTO教師もauto。
- `events`: `name`、ROS header時計の整数`stamp_ns`、`before_s`（既定3秒）。
  比較対象は `[stamp_ns-before_s, stamp_ns)`。イベント表には有効な連続時間も示し、
  指定窓の80%未満ならpartial、連続有効時間0ならunknownとする。衝突後を含めず、衝突を自動検知しない。
  まずイベントなしで解析し、`evaluation_coverage.csv`のtimestampと映像を照合して時刻を決めてもよい。
- 同じbagパスの二重登録は拒否。コピー・別名bagや学習/評価の重複は利用者も確認する。

## offline VSLAM込みの実行（kart Docker）

```bash
cd /workspaces
source /opt/ros/lyrical/setup.bash
source ros2_ws/install/setup.bash
bash scripts/e2e_coverage.sh \
  --manifest /workspaces/analysis/e2e_coverage_manifest.json \
  --output /workspaces/analysis/e2e_coverage_run01
```

pose指定がないbagは、既存`kart_mapping.build_map`を呼び出し、公式の
`edex compute_poses`を実行する。stereo/CameraInfo/静的TF検査、camera設定生成を再利用する。
GPU・ROS・公式Isaac mapping実行環境が必要。depthやoccupancyは実行しない。
既存runnerの都合でcuVSLAM地図も保存されるが、分布解析は連続`odom_poses.tum`のみ使う。
オンラインVSLAMの有無にかかわらず、比較時の経路はoffline poseに統一する。

mapping設定は`kart_bringup/config/mapping/workflow.json`。manifestの`workflow`で
別の完全なJSONを指定可能。dense_reconstruction=trueは拒否する。
`ROS_DOMAIN_ID`は子プロセスだけworkflow値へ設定する。車両ノードや走行制御は起動しない。
`KART_COVERAGE_PYTHON`でPythonを指定できるが、VSLAM実行時はROS依存をimportできる環境が必要。

計算済みposeを再利用する場合、該当recordingへ以下を追加する。

```json
"poses": "/workspaces/map/MAP/official/RUN/poses/odom_poses.tum",
"pose_frame": "camera_link"
```

pose_frameは出力のbody frameを確認して指定する。元bagと同じ時計・同じ収録由来が必須。
自動時刻シフトはしない。静的TFで車体へ変換して速度・yaw rate・曲率を求める。
欠落・非平面・pose跳躍・不正な車両状態は判定不能として扱う。
pose再利用だけならROS不要（NumPy/PyYAML/rosbags、vehicle_tuningと同じ依存）。

## 比較と成果物

- 教師操舵分布、操舵×速度、軌跡曲率×速度を別々に表示する。
- 速度はm/s、曲率はyaw rate/前進速度（1/m）、操舵は正規化指令。
  左右の符号は記録値のまま。AUTOの指令は望ましい操舵・教師とはみなさない。
- 既定bin幅は操舵0.1、速度0.5、曲率0.1。floor(value/幅)で分類し、各binは下端を含み上端を含まない。
- `observed_support`: 合計1秒以上、2bag以上、0.3秒以上の連続滞在3回以上。
  `sparse`: 観測済みだが閾値未満。`unobserved`: 未観測。`unknown`: 無効/未対応。
  閾値はmanifestのsettingsで変更でき、完全な値をsummaryへ保存する。
- 時間加算は同じbin・modeの連続点のみ（gap上限0.12秒）。フレーム数を独立試行数としない。
  学習側はdataset時刻から最大50ms以内の最寄りposeへ対応する（offline解析のみ）。
- bag品質選別はvehicle_tuningの検査を再利用するが、一定指令区間への採用とは分離する。
  旋回も含めるためyaw rate上限2rad/s・横速度上限0.5m/s。低速0.15m/s未満、制動、後退、
  HOSTでない状態等は除外。これは対象範囲であり走行上限の推奨ではない。全実効値を保存する。

`report.html`に分布、イベント直前の支持状態、追加収集/品質確認の候補を表示する。
`evaluation_coverage.csv`には評価走行の各timestamp・操舵・速度・曲率・支持状態を保存し、
映像と照合できる。`summary.json`には設定、出典hash、品質、教師データの例を保存する。
各recordingの`samples.csv`には除外理由と元画像/countsのパス（教師のみ）を保存する。

各bagの座標系を同一と仮定しないため、コース上の同じ場所や周回は自動対応しない。
入力bias・環境差・横位置差・モデル構造や制御遅延は別の要因として残る。
良好な走行を追加収集する候補を出すもので、衝突走行のAUTO出力をそのまま教師に追加しない。

出力先は新規directory限定。失敗時もstatus.json、job.json、vslam.log、途中pose等を保持する。
再実行は別の出力先へ行い、完了済みposeは明示指定で再利用する。既存成果物は上書きしない。

## 検証

```bash
PYTHONPATH=tools/e2e_coverage:tools/vehicle_tuning \
  python3 -m unittest discover -s tools/e2e_coverage -p 'test_*.py' -v
```

分布・時間集計、欠落、衝突前区間、独立bag条件、教師timestamp対応、pose再利用、
runner呼出し・失敗成果物保持をportable testで検証。GPU runnerはmock検証。
実機bag、公式offline VSLAM実行、実際の衝突場面での妥当性は別途確認が必要。
