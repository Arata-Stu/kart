# kart_sim

rc-simのTT-02/CAD・Ackermann操舵・4輪トルク駆動・センサ実装を移植し、
室内JSONマップ生成とROS 2 bridgeを同じpackageに収める。外部rc-sim checkoutへ依存しない。
Python独立process（既定node `/kart_sim`、実行ファイル`kart_sim_node`）を使う。
MuJoCo/GL contextを単一threadで所有するためComponent化しない。
`ament_cmake_auto` + `ament_cmake_python`。MuJoCoはROS system Pythonに導入する。

## マップ

編集の正本はproject rootの`maps/sim/*.json`。ビルド時にshare/kart_sim/mapsへインストールする。
`minicar_2026`は大会PDF △3（2026-09-16）の寸法と写真による**暫定再構成**。
横10.30 m、赤白板の障壁、駐車枠、黒幕、狭路、段差、摩擦区間、近似凹凸を含む。
実測モデルではない。図に矛盾・未記載寸法があり、採用条件はJSONの`assumptions`に記載。
奥行5.60 mは仮のモデル配置。資料のカーペット560+αの確定値とは扱わない。
板は図の赤白区間と接合位置に合わせて分割し、継ぎ目4mmと黒い支柱・灰色固定具を追加。
隙間と支持具寸法は仮値。図の接合位置も座標へ近似しており、支柱46個の厳密配置は未再現。
芝の毛、マットの正確な凹凸、動的な矢印、ライトかく乱は未再現。
板は接触あり、駐車テープ・スタート線は描画のみ。
トンネル入口を塞いでいた横断黒幕は撤去し、外周・内側境界の黒幕だけ残す。
大会マップの駐車枠はwalls=trueで外側3辺に壁を配置。
コースに接する1辺は開口し、入口を横切るコース障壁は撤去したまま。
開口に幅5cmの白線、内側に色付きの4辺テープ枠とP1（緑）/P2（赤）/P3（青）を描く。
床の文字・線は表示だけで走行を妨げない。マップschema詳細はmaps/sim/README.md。
ショートカットは高さ1cmの平台を四方向の幅5cm斜面で囲み、
四隅は5×5cm領域を対角線で接合した三角斜面にして段差なくつなぐ。
ショートカットは平台・斜面とも黒色。右上カーブは輪郭に沿う一体の緑色面と、
その上に白い低摩擦面を配置し、床の露出をなくす。室内の壁・天井・照明も仮値。
空室で車体を検証する`indoor_empty`も用意する。

新規JSONをmaps/simに置き、`scripts/sim.sh --list`で一覧、`--map STEM`で選択する。
ROS launchには`map_dir:=/workspaces/maps/sim`を渡せばビルド後も直接編集が反映される。
実行中のマップ交換はしない。停止して別mapで再起動する。

## TF・車体

+X前、+Y左、+Z上。rc-simのchassis中心からkartの**後輪軸中心base_link**へ変換する。
`publish_truth_tf=true`では`sim_world -> map -> odom`がidentity、
`odom -> base_link`が真値動的TF。`base_link -> rear_axle`はidentity。
VSLAM評価では`publish_truth_tf=false`にし、map/odom/base_linkの動的TFをVSLAMに任せる。
真値pose/odometryは独立した`sim_world`座標のまま出版し、推定軌跡とは初期姿勢で位置合わせする。
odometryのtwistは後輪軸原点のbase_link座標。COM速度へ角速度×原点差を加えて変換する。

仮想D455は`assets/d455.json`で定義。`base_link -> camera_link`はkartの暫定mount値
[0.23385, 0.04750, 0.10000] m、rpy=[0,0,0]、camera_linkは左IR原点。
左/右IR間隔95 mm（右はcamera_linkの-Y）、公称画角はIR水平87°/垂直58°、RGB90°/65°。
[公式D400 datasheet](https://www.realsenseai.com/download/21345/)を参考にした理想pinholeで、
424×240の実機profileの校正を再現した値ではない。
RGB位置[0,-0.060,0] m、IMU位置[0,0,0] mは**仮値**。実機SDKのextrinsicsで差し替える。
`camera_link -> camera_{infra1,infra2,color,gyro,accel}_frame`から各optical frameへ接続する。
カメラの床からの高さはユーザー承認の仮値133mm。後輪軸高さ33mmを引いて取付z=100mm。
カメラの高さは車体に固定され、走行中の車体の上下・姿勢に従う。
光学座標は+X右/+Y下/+Z前、IMUもcamera_gyro_optical_frameへ統一。
描画姿勢とROS TFは同じrigを参照し、CameraInfoのK/Pも描画と同じ内部値を使う。
右CameraInfoのP[0,3]は-fx×baseline、R=identity、D=0。

左右は同じ物理状態・同じstampのmono8（描画RGBの輝度変換）。RGBは単眼rgb8。
IR投光、実赤外線反射、ノイズ、歪み、露出、ブラー、ローリングシャッターは未再現。
校正JSONはmount_xyz/rpy、baseline_m、rgb_xyz/rpy、imu_xyz/rpy、画角を編集できる。
任意のstereo_intrinsics/rgb_intrinsics=[fx,fy,cx,cy]を指定すると画角計算を置き換える。
解像度は424×240固定。IMUはサイト原点の角速度とspecific force（重力の支持反力を含む）を
MuJoCoから取得する。静止時の加速度大きさは約9.81 m/s²、自由落下は約0。
Imu.orientationはsim_worldに対するIMU光学座標の真値、誤差/バイアスなし。
車体姿勢はPoseStamped/Odometryの真値quaternionで取得できる。

質量・摩擦・車輪トルクはrc-simの仮値。搭載物の正本は`assets/vehicle.json`。
CADは表示メッシュ＋外接箱で接触/慣性を近似する。
印刷フレームは前端を27mm短縮したYMax=85.5mm版へ更新。
ローワーデッキの床との隙間11mm、暫定厚み9mm、支柱45mm、印刷板5mmで
フレーム上面を床から70mmに近似（ユーザー情報60〜70mm）。
旧モデルのデッキ厚24mmでは上面85mmとなっていた。
これらはasset JSONの幾何設定でありROS parameterではない。
サスペンション、ESC状態遷移、モータ/ギア/デフ特性、実機遅れは未再現。
brakeは各車輪回転に逆らう散逸トルク。reverseとの混同を避けた簡易モデルで、実ESCの保証ではない。

## ROS入出力

表はroot namespace時。相対topic/service名、`/clock`のみ絶対名。
ROS標準rosout/parameter_events/parameter servicesとtf2の標準通信詳細は省略。

| 方向 | 既定名 | 型 | QoS / 内容 |
|---|---|---|---|
| 入力 | /vehicle/control_cmd | kart_interfaces/msg/ControlCommand | reliable/volatile/depth1。steering左正[-1,1]、他[0,1]、駆動/制動/後退排他 |
| 入力 | /operation_mode/state | kart_interfaces/msg/OperationModeState | reliable/volatile/depth1。AUTO/MANUALのみ許可 |
| 出力 | /clock | rosgraph_msgs/msg/Clock | reliable/volatile/depth10。100Hz既定、sim時刻 |
| 出力 | /sim/odometry | nav_msgs/msg/Odometry | sensor data best effort/volatile/depth5。100Hz既定、真値 |
| 出力 | /tf | tf2_msgs/msg/TFMessage | tf2標準。publish_truth_tf=trueでodom→base_link、100Hz既定 |
| 出力 | /tf_static | tf2_msgs/msg/TFMessage | tf2標準transient local。固定変換 |
| 出力 | /sim/ground_truth/pose | geometry_msgs/msg/PoseStamped | sensor data、100Hz既定、sim_worldでの後輪軸姿勢 |
| 出力（任意） | /realsense/infra1/image_rect_raw | sensor_msgs/msg/Image | sensor data、mono8、424×240、stereo_hz |
| 出力（任意） | /realsense/infra2/image_rect_raw | sensor_msgs/msg/Image | sensor data、mono8、左と同stamp |
| 出力（任意） | /realsense/color/image_raw | sensor_msgs/msg/Image | sensor data、rgb8、424×240、rgb_hz |
| 出力（任意） | /realsense/{infra1,infra2,color}/camera_info | sensor_msgs/msg/CameraInfo | sensor data、対応画像と同stamp/frame、歪みなし |
| 出力（任意） | /realsense/imu | sensor_msgs/msg/Imu | sensor data、imu_hz、角速度・加速度・姿勢の真値 |
| 出力（任意） | /sim/ground_truth/imu | sensor_msgs/msg/Imu | /realsense/imuと同内容、真値評価用 |

| サービス入力 | 型 | 意味 |
|---|---|---|
| /sim/reset | std_srvs/srv/Trigger | spawnへ戻す。時刻0、指令/モードを無効化。下流も時刻巻戻しに対応が必要 |
| /sim/pause | std_srvs/srv/SetBool | trueで物理/clock出版を停止、falseで再開。指令を無効化 |

指令は受信steady時刻とsim stampの両方で期限を判定（時刻丸めの未来側1µsのみ許容）。異常・期限切れは中立（停止保証ではない）。
require_mode=trueではmode heartbeatも必要、モード変更時は新しい指令を要求する。
全接続ノードでuse_sim_time=trueを使う。mode manager/muxの起動は本launchの範囲外。
`/sim/odometry`は真値でありVSLAM推定出力ではない。センサ入力をVSLAMへ接続する
専用入口はkart_bringup/sim_vslam.launch.py。既存実車のvehicle TF launchを併用しない。
移植したLiDAR/深度の計算APIはsensors.pyに残すが、ROS出版は上記画像/IMU/真値のみ。

## 全パラメータ

独自parameterは起動時read_only。package configは単体基準、運用は
kart_bringup/config/sim/sim.yamlを使う。

| 名前 | node既定値 | 意味 |
|---|---|---|
| map_file | 空文字 | 必須JSON絶対パス。launchが選択mapから設定 |
| asset_dir | 空文字 | 空ならinstalled kart_sim/assets |
| camera_enabled | false | 左右IR/RGB取得・出版。GL context必須 |
| camera_rig_file | 空文字 | 空ならassets/d455.json。任意校正JSONへの絶対パス |
| stereo_hz | 60.0 | 左右同時撮影のsim時刻Hz、30..90 |
| rgb_hz | 30.0 | 単眼RGB撮影のsim時刻Hz、30..90 |
| imu_enabled | true | 真値IMU取得・出版。GL不要 |
| imu_hz | 200.0 | IMUのsim時刻Hz、30..1000 |
| publish_truth_tf | true | 真値map/odom/base_link TFを出版。VSLAM評価はfalse |
| viewer_enabled | false | MuJoCo GUI。overviewでは天井のみ非表示、camera描画には残る |
| monitor_enabled | false | localhostの画像・走行操作・録画monitor。ROS nodeでの起動時固定 |
| record_dir | record/sim | rosbag2保存親directory。相対パスは起動cwd基準、開始操作時のみ作成 |
| require_mode | true | modeのAUTO/MANUAL・heartbeatを必須にする。falseは明示的な単体指令テスト用 |
| command_timeout_s | 0.2 | 指令のsim stamp・steady受信期限、正値 |
| mode_timeout_s | 0.5 | modeのsim stamp・steady受信期限、正値 |
| step_count | 10 | wall timer一回の物理step数、1..100。各step 0.001秒 |
| max_steering_rad | 0.45 | 正規化舵角1の中心舵角rad、0より大きく0.6以下、仮値 |
| wheel_torque_nm | 0.025 | 正規化駆動1の各輪トルクNm、正値、仮値 |
| use_sim_time | ROS共通false | YAMLはtrue。自身の物理更新timerはsteady clockを使う |

物理1000Hz、出版/GUIは100Hz既定。処理が追いつかない場合sim時刻が壁時計より遅れる。
画像/IMUは物理step内で取得し、timerごとにその間のサンプルを出版する。
撮影時刻は指定周期を1ms刻みへ切り上げ（誤差1ms以内）。左右は完全同時刻。
最新値と1回分のstep batchを保持し、レンダリングとROS callbackは同一thread。
30〜90Hzはsim時刻の設定であり、壁時計でのリアルタイム達成はGPU/描画負荷に依存する。

## 起動・検証

Dockerfileに[MuJoCo 3.3.7](https://pypi.org/project/mujoco/3.3.7/)、GL/EGL/GLFWライブラリとrosgraph_msgsを追加済み。イメージ再ビルド後、通常のビルドを行う。

```bash
cd /workspaces
scripts/build.sh --packages-up-to kart_sim kart_bringup
source ros2_ws/install/setup.bash
scripts/sim.sh --map minicar_2026
# GUI/カメラが使えるLinux描画環境の場合
ros2 launch kart_bringup sim.launch.py map:=minicar_2026 map_dir:=/workspaces/maps/sim viewer_enabled:=true camera_enabled:=true stereo_hz:=60.0 rgb_hz:=30.0
# 別マップ
scripts/sim.sh --map indoor_empty
```

実機bridge、カメラdriver、vehicle TF、別の/clock publisherを含めない専用入口。
本launchは走行指令やAUTO要求を送信しない。

ROS不要の確認（NumPy・MuJoCoが必要）:

```bash
python3 -m venv /tmp/kart-sim-env
/tmp/kart-sim-env/bin/python -m pip install -r ros2_ws/src/kart_sim/requirements.txt
SIM_PYTHON=/tmp/kart-sim-env/bin/python scripts/sim.sh --check
# macOS GUIはmjpythonを使う。W/S駆動増減、A/D操舵増減、X制動、R reset。
# キーを離しても入力保持。monitorのAuto lapなら手動運転不要。
SIM_PYTHON=/tmp/kart-sim-env/bin/mjpython scripts/sim.sh --preview --sensors --stereo-hz 60 --rgb-hz 30
# 表示された http://127.0.0.1:PORT/ をブラウザで開くと3画像・真値姿勢・IMUと操作ボタン。
# モニタ表示は5Hz、センサ取得Hzとは独立。
PYTHONPATH=ros2_ws/src/kart_sim /tmp/kart-sim-env/bin/python -m unittest discover -s ros2_ws/src/kart_sim/test -v
```

VSLAM評価（Isaac ROS/cuVSLAMとNVIDIA GPUのあるLinux環境、別terminal）:

```bash
ros2 launch kart_bringup sim.launch.py camera_enabled:=true publish_truth_tf:=false stereo_hz:=60.0 rgb_hz:=30.0 imu_hz:=200.0
# VO。tracking_mode:=1でVIO（200Hz IMUを推奨）
ros2 launch kart_bringup sim_vslam.launch.py tracking_mode:=0
```

VSLAM nodeは/visual_slam、設定全項目はbringup/config/sim/vslam.yaml。
既存実車設定と同じplugin/出力を使用し、入力画像/CameraInfo/IMUのみ仮想センサへremapする。
保存済みマップは読み込まず、実車を起動しない。MAC上のpreview自体にはROS/Isaac ROS不要。
外部ROS指令は既存ControlCommand/mode heartbeatを使う。monitor操作時の制御は下記参照。
VO/VIOを同じ走行データで比較し、真値軌跡と初期姿勢を合わせて誤差/追跡喪失を評価する。
ノイズのない描画での成功は実機D455の成功を保証しない。

GL検証（displayが必要、portable pytestには含めない）:

```bash
PYTHONPATH=ros2_ws/src/kart_sim /tmp/kart-sim-env/bin/python ros2_ws/src/kart_sim/test/render_smoke.py
```

検証結果はdocs/simulation.mdを参照。ROS build/load/通信、Linux Docker描画、実車との一致は未確認。

## monitorによる自動周回とrosbag保存

ROSなしのmacOSでも、`--preview --sensors --record-dir /absolute/output`で
既存map作成pipeline用のrosbag2を直接保存できる。`rosbags==0.11.6`をrequirementsで導入する。
[rosbagsの公式Writer API](https://ternaris.gitlab.io/rosbags/api/rosbags.rosbag2.html)を利用し、
SQLite3 `.db3` + `metadata.yaml`、CDR、metadata version 9で出力する。
ROS driverやROS daemonを起動する必要はない。LinuxのROS nodeでも同じwriterを使用する。

1. localhost URLを開き、`Start rosbag`で記録を開始する。
2. `Auto lap`で大会コースを周回する。走行と録画の開始は独立。
3. `Stop drive`で制動、`Stop rosbag`でbagを閉じてmetadataを確定する。
4. stateの`bag.recording=false`・`bag.finalized=true`と`bag.path`を確認し、そのdirectoryを既存map生成の入力bagに指定する。

連打した開始/停止は重複writerを作らない。開始ごとに時刻+ランダムsuffixの新directoryを作り、
既存bagへ上書きしない。終了時もwriterを閉じる。録画中のResetは拒否する。
操作はHTTP threadからqueueへ渡し、MuJoCo/GLとwriterはsim threadだけで操作する。
UIで要求受付後、stateの`error`/`bag.error`と実状態を確認する。

保存対象は左右画像/CameraInfo、RGB/CameraInfo、カメラ固定TF、IMU、`/clock`、
`/sim/ground_truth/{pose,imu}`、`/sim/odometry`。名前・型・Hzは上のROS表と同じ。
`/tf_static`はtransient-local/reliableで、真値のmap/odom変換を含めない。
このwriterはnodeのpublish_truth_tf設定と独立に、常にmapping用の固定TFだけを保存する。
動的`/tf`・推定器出力・制御指令は保存しない。実機のkart_bag_managerの操作/設定は変更しない。
非圧縮60Hz stereo+30Hz RGBで画像だけ約21MB/s（約1.3GB/min）。
描画/保存がwall timeより遅くなった場合もsim時刻で取得し、実時間60Hzの達成を保証しない。

自動周回はmap JSONの`autodrive.waypoints`（後輪軸XY、m、閉ループ）をPure Pursuitで追従する。
`speed_mps`既定0.45（0.05..1.5）、`lookahead_m`既定0.35（0.15..1.0）。
これらはROS parameterではなくmap資産の設定。大会mapに周回経路を追加、空室mapには経路なし。
曲率による減速と速度比例制御を使い、経路離脱0.65mまたは8 sim秒の停滞で制動する。
車体の矩形衝突proxyには実物のwheel cutoutがないため、chassisと4輪の自己接触を除外する。
室内・コース障壁との接触は有効。
**真値姿勢を使うデータ取得用controllerであり、VSLAMや画像認識による自律走行ではない。**
monitorのAuto/manual/Stopを押すとsim内部が制御を所有し、外部ROS指令より優先する。
require_modeによる外部指令のgateをこの内部操作には適用しない。実機bridgeへの指令出版はない。
外部ROS制御へ戻す場合はsimを再起動する。起動だけではAUTO/録画を開始しない。

ROS版の例（全parameterは運用YAMLを保持し、必要なものだけ明示上書き）:

```bash
ros2 launch kart_bringup sim.launch.py camera_enabled:=true monitor_enabled:=true publish_truth_tf:=false record_dir:=/workspaces/record/sim
```

monitor HTTPは`GET /`、`GET /state`、`GET /{infra1,infra2,color}.png`と
`POST /command`を提供する。bodyは`{"action":"auto_start"}`など。
actionは`auto_start/stop/record_start/record_stop/reset/manual`、manualのみ
`"value":[steering,throttle]`（各-1..1、負throttleはreverse）を受ける。
これはlocalhost専用HTTP APIで、追加ROS topic/serviceはない。
