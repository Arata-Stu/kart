# VSLAMを使う簡易な車両特性・速度制御チューニング

追加の計測器なしで、手動直線走行のrosbagから加速・惰行・制動を整理する。
地図なしの局所odometryを速度の基準にする。VSLAMの絶対速度精度、推定器の遅延と
物理応答の分離、限界グリップ、旋回中の制動はこのツールでは検証しない。

1. `collect`: RealSense・手動vehicle・録画管理＋地図なしcuVSLAMを起動。
2. `check`: 必須topicの存在・型・件数を確認。
3. `analyze`: 時刻・TF・状態・直線条件を検査し、特性表・概算モデル・kff候補を作成。
4. `tune`: 指定したPIDを同定モデル上で試算し、レビュー用の完全な設定YAMLを出力。

解析CLIはROSノードを起動せず、topic publish・走行・実車parameterの変更を行わない。
HTMLはネット接続不要で、ポインター／スライダーによる時刻別数値確認ができる。
Map Studioとは独立した小さなCLI＋レポート。新しいROS packageは追加しない。

## 実行環境

Jetsonのkart Dockerで記録し、Notebookのkart Docker `/workspaces`で解析する。
DockerにはNumPy/PyYAML/rosbagsが既に含まれる。解析はROSのsource不要で、
SQLite3/MCAPのrosbag2を`rosbags`で読む。MCAP/SQLite3を直接渡さず、metadata.yamlがあるbag directoryを指定。

Mac等で解析だけする場合は専用venvへ`requirements.txt`を導入し、
`KART_TUNING_PYTHON=/path/to/venv/bin/python`を指定する。
custom msgはbag内の定義を優先し、定義がないbagにはrepoの`kart_interfaces`定義を補う。

## 記録入口（Jetson Docker）

```bash
cd /workspaces
# 既存手順でkart_bringupをビルド済みにする。直接ビルドする場合:
source /opt/ros/lyrical/setup.bash
cd /workspaces/ros2_ws
colcon build --symlink-install --packages-up-to kart_bringup
source install/setup.bash
cd /workspaces
bash scripts/vehicle_tuning.sh collect run_name:=straight_01
```

他のmission/drive/e2e/localization launchを終了してから起動する。
同じsensor containerへRealSenseとcuVSLAMをloadする。カメラ／vehicleは各1回だけ起動。
地図、HDMap、VGL、Pure Pursuit、速度PIDは起動しない。保存地図なし、mapping無効、
動的map/odom TFの出版なし。既定VO。VIOを選ぶ場合だけ`tracking_mode:=1`。

起動引数（空文字が既定、YAML保持）:

| 引数 | 意味 |
|---|---|
| `run_name` | bagの実験名。実directoryは最初のSTART時に作成 |
| `record_dir` | 録画root。既定は録画YAMLの`/workspaces/record` |
| `device` | bridgeポートの明示上書き。既定`/dev/ttyACM0` |
| `rgb_fps` / `infra_fps` | カメラHzの明示指定。stereo無効は起動前拒否。RGBは記録推奨 |
| `tracking_mode` | 0=stereo VO、1=VIO。既定はtuning/cuvslam.yamlの0 |
| `multicam_mode` | 0=Moderate、1=Performance、2=Precision。既定は1 |

車体取付TFは`kart_bringup/config/vehicle/transforms.yaml`を使う。現在のcamera_mount値は
ユーザー指定の暫定値で、実測校正済みではない。nullの場合は起動を拒否し、仮のidentityを補わない。

### 試験操作

- 推定が安定してからR1で録画START。録画と走行開始は別操作。
- MANUALで短い直線を走る。小さい一定スロットルから段階を変え、各条件を繰り返す。
- スロットルを0へ戻す惰行区間を記録。制動は別試行で低い指令から記録する。
- 戻り・旋回区間も記録されるが、解析では操舵／yaw rate等で除外される。
- 停止後L1で録画STOPし、metadata.yamlの確定を確認。AUTOへの自動切替はしない。
- `session.example.json`をコピーし、路面、タイヤ、電池、搭載物、試験メモを記入する。
  同一条件で少なくとも加速3区間・惰行3区間を目安にする。制動モデルは制動3区間以上も必要。
  数値は解析上の最低条件であり、十分な同定精度を保証しない。
- 1つの指令区間は0.8秒以上必要。最初の0.3秒と平滑化の端は使用しない。
  kff用にはさらに速度が落ち着いた末尾0.6秒以上が必要。直線が短い場合は加速度表だけでもよい。

## データ記録要件

標準`config/recording/bag_manager.yaml`とOpenEB版は以下を既に含むため、録画topicは変更していない。
列挙されているだけでは実データがある保証にならない。`check`と`analyze`で確認する。
通常の`bringup.sh collect`だけではVSLAMは起動しないので、本専用入口を使う。

| 区分 | topic | 型 | 用途・条件 |
|---|---|---|---|
| 必須 | `/visual_slam/tracking/odometry` | `nav_msgs/msg/Odometry` | twist、header/child frame、header stamp。gap上限既定0.12秒 |
| 必須 | `/vehicle/control_cmd` | `kart_interfaces/msg/ControlCommand` | mux後の要求値。実PWM・駆動力ではない。対応鮮度0.15秒 |
| 必須 | `/operation_mode/state` | `kart_interfaces/msg/OperationModeState` | 既定MANUALだけ。鮮度0.3秒 |
| 必須 | `/localization/vslam/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | hardware_id=`visual_slam`、level=0、vo_status=OK。鮮度2秒 |
| 必須 | `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | hardware_id=`JPBB-01`。host_active、esc_command_state、esc_brake_rejectedを検査。鮮度2秒 |
| 必須 | `/vehicle/active_path` | `std_msgs/msg/UInt8` | 2=HOST、鮮度0.2秒。headerなしのためbag受信時刻 |
| 座標変換時に必須 | `/tf_static` | `tf2_msgs/msg/TFMessage` | odom childから車体frameへの剛体変換。childが車体frame自身なら不要 |
| 推奨 | `/realsense/imu` | `sensor_msgs/msg/Imu` | 加速度ノルムを応答の照合に表示。重力除去や積分速度推定はしない |
| 推奨 | `/realsense/color/image_raw` | `sensor_msgs/msg/Image` | 振動・滑り等の目視照合。解析はmetadata件数のみ、画素展開／動画生成なし |
| 推奨 | `/vehicle/output_channels` | `std_msgs/msg/Int32MultiArray` | 基板報告PWMの追加確認用。実測PWMではない |
| 推奨 | `/vehicle/trim/state` | `kart_interfaces/msg/ControlTrim` | トリム値を保存。途中変更／不正値を検出したら解析を拒否 |
| 自動制御の評価時 | `/control/speed_diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | active時のtarget_mps。AUTOを解析対象にしたとき速度RMSEを計算 |

stereo画像・CameraInfoも標準bagへ保存される。画像の欠落や露光条件は必要に応じて別途再生して確認する。
本ツールは画像を再生しない。`/vehicle/vbec`はBEC電圧であり走行電池電圧ではない。

推奨記録: camera/VSLAM/vehicle/controlの実効parameter dump、試験時のgit revision、
設定YAMLコピー。実行中のノードは`ros2 param dump /visual_slam`等で保存できる。
ソースYAMLと実効parameterは同一とは限らない。bagのheaderが同一ROS clockに属することを確認する。

## 記録チェックと解析（Notebook Docker）

```bash
cd /workspaces
bash scripts/vehicle_tuning.sh check \
  --bag /workspaces/record/2026-10-15/14-00/straight_01 \
  --output /workspaces/record/analysis/straight_01_check

bash scripts/vehicle_tuning.sh analyze \
  --bag /workspaces/record/2026-10-15/14-00/straight_01 \
  --session /workspaces/record/straight_01_session.json \
  --output /workspaces/record/analysis/straight_01
```

日付・時刻は実際のbagへ置き換える。`--session`は省略可能。
`check`は存在・型・件数の確認だけで、品質の合格判定ではない。必須不足ならレポートを残しexit 2。
`analyze`は時刻／フレーム不整合をエラーにする。有効区間0件なら理由付きレポートを残しexit 2。
モデルが不成立でも特性表が作れればexit 0。`summary.json`の`model.usable`でモデル可否を区別する。
入力bagは読み取り専用。出力は新規directoryに限り、完了時に公開、既存出力は上書きしない。

成果物:

| ファイル | 内容 |
|---|---|
| `report.html` | 速度・指令・加速度・IMU、除外理由、品質、区間、モデル検証を表示 |
| `summary.json` | 使用設定とhash、source、時刻品質、TF、kff候補、検証誤差、試験メモ |
| `samples.csv` | 全odometry点。除外理由と解析に採用したsegment番号。空segmentはモデル対象外 |
| `segments.csv` | 一定指令区間、速度範囲、加速度、定常速度 |
| `characteristics.csv` | 速度×指令binごとの加速度中央値・P10/P90・区間数 |
| `model.json` | `a=A*throttle-B*brake-C*v-D`の係数と適用範囲、学習／検証区間ID |
| `normalized.json` | 画像を含まない抽出データ。`--normalized-json`でROS/bag再読込みなしに再解析可能 |

### 時刻・TF・データ選別

- 既定`clock=bag`は同一の実時間収録bagの受信時刻を全streamへ使う。推定器の処理遅延を含む概算。
  headerとbagを保存し、その差の中央値・P95、gap、header重複数を表示する。
- `clock=header`は同一時計・元stampが保存されたデータで明示選択。headerなしactive_pathだけは
  受信時刻を使う。再生wall timeと元header時刻の混在、別bagの自動同期は対応しない。
  bag/header差の絶対値が2秒超なら拒否。source commandの古さも別途確認する。
- 過去の最新指令／状態のみを鮮度上限付きで対応させる。未来の指令を補間しない。
- `twist`はchild frameの速度として、静的TFで`vehicle_frame`へ回転し、
  `v_vehicle=R*v_child-(R*omega_child)×p`で原点差を補正して前方x成分を使用する。
  **map→odomの動的TFとmap位置の差分は使わない**。固定TFは時刻補間不要。
  固定TF値の変更／競合・欠落・frame変更は拒否。車体取付が途中で動く試験は対象外。
- MANUAL、HOST経路、bridge正常、VO正常、前進、低操舵／低yaw rate／低横速度を採用。
  STOP/PROPO/reverse/同時throttle+brake、異常値、gapは除外。
- 同じ指令に近い連続区間だけで局所直線近似し、速度平滑化と加速度推定を行う。
  指令変化は区間先頭値と比較するので、ゆっくりした指令ランプを定常と誤認しない。
  欠落・異常・指令変更を跨いだ平滑化はしない。
- 1Hz程度のbridge診断では短いESC拒否を全て検出できない。ブレーキ表は指令に対する
  観測応答であり、ESCの実制動／停止を証明しない。

### モデルと候補値

特性表はモデルの成立に関係なく出力する。kffは、定常末尾がある3区間以上かつ2段階以上の
スロットルから、`u=kff*v`を原点通過で近似する。切片・摩擦・始動特性は表現できないため
残差と速度範囲も表示する。これは現在のspeed_controllerに対応する簡易値で、加速度逆モデルではない。

加減速モデルは最低6区間を要求。種類ごとに末尾約1/4の区間（3区間以上ある種類）を検証へ残し、
残りで係数を推定する。長い区間が支配しないよう区間ごとに等重みとする。
制動が3区間未満なら制動行はfitから除外し、ブレーキモデルは未同定にする。
励起不足・rank不足・条件数>1000・非物理的な符号は拒否。係数を都合よくclipしない。
検証区間の加速度RMSE、定数予測との比較、区間内の速度rollout RMSEでモデル可否を判定する。
これは同一bag内の検証であり、別日・路面・電池での汎化検証ではない。

## PID候補の試算

まずkff候補とモデルの検証誤差を確認する。モデル成立時だけ次を実行できる。
下記のgain／速度は**構文例であり、実車推奨値ではない**。実測範囲と解析結果に合わせて選ぶ。

```bash
bash scripts/vehicle_tuning.sh tune \
  --analysis /workspaces/record/analysis/straight_01 \
  --output /workspaces/record/analysis/straight_01_candidate_a \
  --kp 0.05 --ki 0 --kd 0 --kff 0.2 \
  --targets 0.3,0.6,0.3 --stage-seconds 4 --delays 0.1,0.2,0.3
```

`--base-config`の既定は実機用`kart_bringup/config/control/speed_controller.yaml`。
指定しなかったgain/limitと全parameterは保持。`--throttle-limit`、`--brake-limit`も明示指定できる。
目標はbase configのmax_target_speed_mpsを超えられない。制動モデル未成立ならbrake_limit>0の試算は拒否。
現在の実機設定のbrake_limit既定は0。データがあるという理由で制動を自動有効化しない。

`SpeedPid`と同じP/I/D/FF、積分制限・anti-windup、出力制限、目標0の処理を再現する。
車両モデルの積分周期は20ms。初期速度は最初の目標速度で、停止発進を検証したことにはならない。
遅延は既定0.15秒の**仮定**。`--delays`で感度を比較する。指令・速度が測定範囲外に出た割合を示す。
目標0への減速も、低速域を測定していなければ外挿になる。実bagを別gainで再走行した結果ではない。
ROS状態機械、AUTOのarming/watchdog、操舵、センサノイズ、ESCの状態遷移は試算しない。

出力は`report.html`、条件／変更前後を持つ`tuning.json`、条件別`simulation_N.csv`、
全parameterを保持した`speed_controller.candidate.yaml`。試算が良くても自動適用しない。
レビュー後、STOP中に必要なgainだけ運用YAMLへ反映して再起動する。
wheelbase／舵角などの未校正値を候補生成で埋めない。現在のmax_steering_rad=0のままでは
実車AUTO制御の準備は完了していない。実車再試験のbagを別出力へ解析して比較する。

自動制御bagはconfig.jsonをコピーし`allowed_modes: [1]`に変更して`--config`へ指定。
`/control/speed_diagnostics`のactive目標とVSLAM速度のRMSEを表示する。
同じ直線選別後の区間だけの指標であり、全周回追従性能ではない。

## 全解析設定

`config.json`が正本。CLIの`--config`でコピーを指定する。設定のSHA256と全値を結果へ保存する。
独自ROS parameter／ROS node／publish topicはない。

| 設定 | 既定 | 意味 |
|---|---|---|
| clock | bag | bag/headerの時刻選択 |
| vehicle_frame | base_link | 前方x、左y、上zの車体frame |
| allowed_modes | [2] | 2=MANUAL。AUTO評価は[1]。STOP/PROPOは不可 |
| command_max_age_s / mode_max_age_s | 0.15 / 0.3 | 過去指令／modeの鮮度 |
| diagnostic_max_age_s / path_max_age_s | 2.0 / 0.2 | 診断／HOST経路の鮮度 |
| odom_max_gap_s | 0.12 | odom連続性、IMU対応の期限 |
| max_header_bag_offset_s | 2.0 | 時計ずれ／大きい遅延の拒否閾値 |
| max_steering | 0.08 | 正規化操舵の絶対値上限 |
| max_yaw_rate_rps | 0.2 | 車体yaw rateの絶対値上限 |
| max_lateral_speed_mps | 0.15 | 車体横速度の絶対値上限 |
| min_speed_mps / max_speed_mps | 0.15 / 8.0 | 解析する前進速度域。走行指令上限ではない |
| max_abs_acceleration_mps2 | 8.0 | 推定加速度の外れ値閾値 |
| smoothing_window_s | 0.3 | 局所一次fitの全幅 |
| settle_after_command_s | 0.3 | 区間開始後に捨てる時間。さらに窓の半幅を除外 |
| command_change_threshold | 0.015 | 区間先頭指令からの変化許容値 |
| min_segment_s | 0.8 | 一定指令区間の最短時間 |
| steady_acceleration_mps2 / steady_min_duration_s | 0.12 / 0.6 | 定常末尾の加速度上限／最短継続 |
| speed_bin_mps / command_bin | 0.25 / 0.025 | 特性表の速度／指令bin幅 |
| min_fit_segments | 6 | モデル同定の最低区間数 |
| max_validation_rmse_mps2 | 0.5 | 別区間の加速度RMSE上限。定数予測より良いことも必要 |
| max_validation_speed_rmse_mps | 0.3 | 別区間の速度rollout RMSE上限 |
| assumed_delay_s | 0.15 | PID試算の遅延仮定。実測同定値ではない |
| topics | config.json参照 | 上の記録要件のodom/command/mode/vslam/bridge/path/trim/imu/image/speed_diagnostics |

これらは初期の解析閾値で、実車の校正値ではない。閾値を緩める前に除外理由を確認する。

## 検証

```bash
python3 -m unittest discover -s tools/vehicle_tuning -p 'test_*.py' -v
PYTHONPATH=ros2_ws/src/kart_bringup python3 -m unittest discover \
  -s ros2_ws/src/kart_bringup/test -p test_tuning.py -v
```

既知の合成加減速モデルの係数回復、区間分離検証、TF回転／原点差、異常除外、
時刻不整合、因果的な対応、PID出力制限、CLI成果物と上書き拒否、実SQLite3 bagの読出しを確認する。
実車bag、JetsonのCUDA/cuVSLAM、ROS componentロード・結合、車両の走行は別途確認が必要。
