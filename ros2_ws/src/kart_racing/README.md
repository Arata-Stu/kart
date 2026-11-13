# kart_racing

RCカー追従・追越しと静止障害物迂回の実験実装。
Pythonの`racing_behavior`は任意選択のlive経路ルータ、C++17の`predictive_spliner`は従来のshadow比較実装。
90 Hz VO、既存PP/PID、TF所有者は維持する。画像検出・2D追跡は別起動の
[kart_opponent_tracker](../kart_opponent_tracker/README.md)とbringupが担当する。

## live追従・追越し（実験機能、既定OFF）

`bringup.sh`の地図走行で「追従・追越し」を`off / shadow / active`から選ぶ。
shadowは通常ラインを変更せず候補表示のみ、activeはHDMapの基線を
`/planning/course_reference`へ分離し、`racing_behavior`だけが`/planning/reference_line`を配信する。
activeには追加学習したracing **2クラス**engine・trackerが必要。TUIはtrackerを連動ONにし、
認識OFF/検出のみに戻すと追越しOFFにする。COCOのsuitcase等を障害物には付け替えない。
E2E、駐車、固定AUTO、自動復帰とは併用しない。起動してもAUTOには移行しない。

- 同方向のRCカーが2.0 m/s以下: 周回学習を待たず、許可された直線で横へ出る簡易候補を評価。
- 2.0 m/s超のRCカー: 追従中に相手の周回ごとの速度・横位置を学び、モデル成立後に学習予測で候補を評価。
- 停止RCカー（0.10 m/s以下の確認済み位置）: 地図上の位置保持で迂回候補を評価。
- obstacle: 静止物として迂回。左右を両方確認し、優先側が塞がっていれば反対側を選ぶ。
- どちらも不成立: `BLOCKED`で速度0。未確認対象は`OBSERVE`で速度0。

どちらも自車の設定速度が相手より十分速く、経路全長が許可区間に収まることを確認する。
地図境界・登録障害物と仮定寸法の車体掃引、曲率、簡易一定速度／学習GPの物体予測を検査する。
2.0 m/s以下・静止物の合流余裕は0.20 m、それより速い車は0.60 m＋相手速度×0.70 s
（別途、自車後輪軸→後端・対象半長・位置余裕を加算）。
停止物の撮影位置は最大5秒保持し、前方視野から消えたことだけで基線へ戻さない。
わずかに動く静止近似は速度×経過時間が位置余裕を超えた時点で失効する。
通常観測の期限は0.4秒。追越し採用後は同じ側の並走区間を保持し、対象の見失いだけでは
直ちに基線へ戻さない。`COAST_PASS`は期限付きの対象予測で継続、`HOLD_SIDE`は合流余裕を待つ状態。
予測は最大6秒、前後・横の不確かさを経過時間とともに拡大し、合流は予測した両車端の余裕を再確認する。
許可区間・直線性・確認する通路の範囲を越えて延長しない。地図上のline別`passing_zones`が未設定なら
新たな追越し候補を採用しない。採用側の通路不成立、予測期限超過、時刻逆行、入力断、地図/設定変更は
保持経路上の速度0へラッチし、STOP・1秒以上停車・基線から0.15 m以内で明示resetする。
静止と誤判定した再発進や対向・横切りを含む場合は予測前提が成立しない。

自車はrear_axle基準で前端=全長−後端距離の非対称外形を使う。後端距離0.1065 mは既存車体方針の暫定値。
地面投影点は対象中心の近似であり、物体寸法は3D測定ではない。未観測の側方・合流先が空いている
証明にはならず、この経路は時刻付き軌道や衝突監視器ではない。速度・投影・車体寸法は実測未校正。
**portable単体試験とUI表示確認の対象であり、ROS結合・Jetson負荷・実車追越しは未確認。**

### 地図での許可と学習モデル

Studioの「3 ライン → 追越し許可区間を設定」で、対象ライン・開始/終了WP・許可する側を指定し、
追加後に地図を保存する。WP番号は1始まり、保存indexは0始まり。地図クリック・複数区間・閉路の始点跨ぎに対応。
許可は当該ラインのXY/閉路状態のSHA-256へ結び、形状が変われば再設定する。速度だけの変更では保持する。
map.json、exportのhd_map.yaml/CSV bundle、地図転送に同じ許可を含める。未設定は追越し許可ゼロ。
区間に入ってから候補を採用し、横へ出る動作から復帰完了までを同じ許可区間へ収める。

学習は既存C++`OpponentModel`（Predictive Splinerの局所Matérn/RBF GP）を
`libkart_racing_opponent_model`のC ABIとPython ctypes adapterからインプロセスで使う。
GPU追加・推論毎のsubprocess・実行時コンパイルは行わない。経路生成はPythonの有界候補探索であり、論文のSQP再現ではない。
同じ相手が毎周似た走りをする前提で、走路被覆90%以上かつ連続進捗95%以上の学習成立を要求する。
許可区間外の追従中にも実観測だけを学習する。予測で補った点を教師へ戻さない。
欠測では成熟モデルを保持し、未成熟モデルは欠測で飛んだ区間を連続した学習済み周回へ数えない。
ID・地図の不連続、学習済み走行からの逸脱ではモデルを失効させる。再検出の未確認位置で合流を承認しない。
短期trackが消えても同一IDの学習記憶を独立に最大64個保持し、上限時は採用中の相手以外の最古モデルを置換する。
一度も確認されなかった一過性の候補は0.4秒の観察後に失効させ、正常な空結果が続く場合に永久FAULTへしない。
確認済み対象の喪失や投影/入力異常とは区別する。GPが地点ごとに予期する加速・横位置変化は許容範囲と照合する。
GPの速度σによる位相誤差と、未観測中の加速・横移動誤差を加味して合流を判定する。σは校正済み安全確率ではない。
native libraryがない場合も2.0 m/s以下・静止物の簡易方式は動き、高速側の学習追越しだけを抑止する。
通常はamentのlibへbuild/installし、portable試験では絶対パスの環境変数`KART_RACING_OPPONENT_MODEL_LIBRARY`で指定できる。
設定したパスのlibraryが不正なら別libraryへ黙ってfallbackしない。

診断JSONの`planning_mode`はsimple/learned/follow、`learning`は利用可否・学習成立・被覆率、
`permission`は採用区間と残長、`target_observation`は観測/予測・経過時間・位置不確かさを表示する。
合流未承認の間、制御経路は横へ出る区間と並走区間だけを含む。PPの先読みに合流区間を先渡ししない。

### racing_behaviorのROS API

node/実行ファイル`racing_behavior`。root namespaceで次の既定名。標準TF・ROS管理topicは省略。
20 Hzのsteady timer、出力点数は最大300、観測は元撮影stampとsteady受信時刻の両方で失効させる。

|方向|既定topic|型|QoS|内容|
|---|---|---|---|---|
|入|/baseline_reference|kart_interfaces/msg/ReferenceLine|reliable/transient_local/1|launchでshadowはreference_line、activeはcourse_referenceへremap|
|入|/hdmap/document|std_msgs/msg/String|reliable/transient_local/1|正規化したmap文書・走行境界・登録障害物|
|入|/racing/map_points|kart_interfaces/msg/GroundDetections|best effort/volatile/5|有効性付きRCカー/障害物の地図投影点|
|入|/racing/ground_points|kart_interfaces/msg/OpponentGroundPoints|best effort/volatile/5|確認済みRC track、撮影時刻TFでmapへ変換|
|入|/visual_slam/tracking/odometry|nav_msgs/msg/Odometry|best effort/volatile/5|同stampのmap→rear_axle TFから位置・速度推定|
|入|/operation_mode/state|kart_interfaces/msg/OperationModeState|reliable/volatile/1|動的設定・resetのSTOP確認|
|出|/racing/candidate_reference|kart_interfaces/msg/ReferenceLine|reliable/transient_local/1|候補経路と速度|
|出|/planning/reference_line|kart_interfaces/msg/ReferenceLine|reliable/transient_local/1|activeだけで生成する制御用経路|
|出|/racing/behavior_path|nav_msgs/msg/Path|reliable/volatile/1|Foxgloveの選択経路|
|出|/racing/behavior_markers|visualization_msgs/msg/MarkerArray|reliable/volatile/1|左右候補、寿命0.3秒|
|出|/racing/behavior_status|std_msgs/msg/String|reliable/volatile/1|最大128 KiB JSON: 状態・理由・対象・左右余裕・合流位置・寸法仮定・パラメータ|

service `/racing/behavior/reset` (`std_srvs/srv/Trigger`) は上述の停車条件でラッチだけを解除する。
`behavior_status`の`stamp_ns`は動的入力の最古stamp、`published_at_ns`は診断発行時刻で区別する。
制御経路のpublisherが複数なら停止目標。node停止はactive専用PPのline/wall timeout各0.4秒で
既存制御の異常停止経路へ渡す。運用PIDのbrake_limit=0では速度0はthrottle0/惰行となり、
設定減速度1.0 m/s²や物理停止完了の証拠ではない。通常の制動設定は変更しない。

### racing_behaviorの全パラメータ

pkg `config/behavior.yaml`は単体用、運用正本は`kart_bringup/config/racing/behavior.yaml`。
`mode`・`map_file`は起動時固定。残り33項目は標準parameter serviceと動的UIから、STOPかつ1秒以上停車中に変更できる。
変更後は経路を停止保持し、基線近傍でresetしてから再評価する。UIの「設定」対象は`behavior`。
「行動計画をリセット」ボタンで同じTriggerを明示的に呼び出せる。結果不明時の自動再送は行わない。

|名前|既定|単位・意味|
|---|---:|---|
|use_sim_time|false|標準ROS時計選択|
|mode|shadow|shadow候補のみ / active制御経路反映|
|map_file|空文字|必須、選択HDMapの絶対パス|
|speed_cap|0.8|m/s、behavior経路上限|
|vehicle_length / vehicle_width|0.35 / 0.19|m、自車の仮定外形|
|rear_axle_to_rear|0.1065|m、後輪軸から自車後端、前端は全長から差引く（未実測）|
|uncertainty_margin|0.12|m、境界・物体との幾何余裕|
|rc_length / rc_width|0.35 / 0.19|m、RCカーの仮定外形|
|obstacle_length / obstacle_width|0.50 / 0.40|m、検出障害物の仮定外形|
|slow_speed|0.35|m/s、低速RCと移動RCの分類閾値|
|stationary_anchor_speed|0.10|m/s、地図位置を固定して保持する閾値|
|headway|0.70|s、追従車間/移動対象の合流余裕の速度係数|
|follow_min_gap|0.40|m、両車外形間の追従車間下限|
|moving_merge_margin|0.60|m、移動車の合流余裕（速度係数/外形は別加算）|
|static_merge_margin|0.20|m、低速車/静止物の合流余裕（外形は別加算）|
|moving_speed_advantage|0.20|m/s、移動車追越しに必要な速度差|
|moving_observation_s|0.50|s、移動車予測の連続観測期間|
|object_timeout|0.40|s、動的観測の期限|
|static_memory_s|5.0|s、確認済み静止位置の保持期限|
|min_transition|0.80|m、左右移動の最小距離、曲率由来下限も適用|
|max_curvature|1.20|1/m、候補曲率上限|
|sample_spacing|0.05|m、候補標本間隔（最大300点）|
|lookahead_m|20.0|m、候補経路の距離上限、3..50|
|preferred_side|left|左右とも成立した場合の優先側、left/right|
|deceleration|1.0|m/s²、追従上限の仮定減速度|
|prediction_horizon_s|15.0|s、名目速度による予測時間上限|
|simple_pass_speed|2.0|m/s、周回学習を待たず候補評価するRC速度上限、0.1..5|
|prediction_hold_s|6.0|s、採用後の未観測対象を予測保持する上限、0.5..10かつobject_timeout以上|
|prediction_speed_uncertainty|0.10|m/s、速度予測の不確かさ、0.01..1|
|prediction_acceleration|0.05|m/s²、未観測中の加速不確かさ、0.01..2|
|prediction_lateral_speed|0.03|m/s、未観測中の横方向不確かさの拡大率、0.01..0.5|
|prediction_uncertainty_limit|1.5|m、前後方向の予測不確かさ上限、0.1..3|
|max_pass_heading_change|0.25|rad、追越し区間の基線方位変化上限、0.05..0.6|

Studio「走行調整」には状態・速度・対象IDと推定速度、左右の不成立理由・最小余裕・合流位置、
車体/対象寸法を表示する。地図は青が採用経路、緑が左候補、橙が右候補、破線が不成立。
未接続・未対応・配信なし・受信停止を分け、0.5秒で失効する。Foxgloveではmap固定座標で
`behavior_path`、`behavior_markers`、`map_markers`、Raw Messagesの`behavior_status`を使用する。

以下は従来C++ shadow実装の仕様であり、live behaviorとは別に起動する比較用機能。

## アルゴリズムと範囲

論文 https://arxiv.org/html/2410.04868v2 を参考に独立実装した。
相手のd(s)はMatérn 3/2、v(s)はRBFの局所GP（最大20近傍）。
周回距離でbin化し、最大2048 bin、標準bin幅約0.5m。元論文の全GP/0.1m/SQPとは異なる。
標本を無制限に保存せず、相関した連続観測の分散を標本数で割らない。
約95%周の連続進捗と90%以上のbin観測で学習成立を判定する。
ID変更・時刻逆行・0.5秒超の観測断・不可能な進捗で履歴をリセット。
成熟モデルから横位置/速度が大きく逸脱した場合も再学習するため、発進時の速度を観測した
binが次周と異なると、学習完了まで複数周かかることがある。

学習前は時間車間と停止距離による速度上限で追従する。
学習後はGPの速度平均+2σで相手、自車は加速上限で15秒まで0.1秒刻みに前進予測し、
並走開始〜合流余裕を得る区間（RoC）を推定する。
左右×3種類の立上がり長の五次スプラインを比較する有界候補探索。
元論文のSQP最適化ではないため「Predictive Spliner実験版」と表示する。
左右の幅と曲率、相手の横位置平均±2σ、車体幅・余裕、合流距離を考慮する。
候補生成は直線・既知の明示通行区間のみ。曲線上の追い越しや未観測空間を扱わない。
同一IDの理想観測を継続できる条件でPASS→MERGE→PASSEDを評価する。
予測失効・合流不能はFAULT。FAULTでの速度0は物理停止・安全退避を保証しない。
GPのσは校正済みの安全確率ではない。安全認証・実車用衝突監視の代替ではない。

## ROS API

node `predictive_spliner`、component `kart_racing::PredictiveSplinerNode`、
実行ファイル `predictive_spliner`。全topicは相対名、root namespaceで以下の名前。

| 方向 | 既定topic | 型 | QoS | 内容 |
|---|---|---|---|---|
| 入 | /racing/observation | kart_interfaces/msg/HeadToHeadObservation | best effort/volatile/1 | 同時刻基準へ整合した自車・相手Frenet状態と区間情報 |
| 出 | /racing/shadow_plan | kart_interfaces/msg/HeadToHeadPlan | reliable/volatile/1 | 入力callbackごとの追従速度・空間経路・状態、timeoutで無効化 |

service/actionなし。標準rosout/parameter_eventsおよび標準parameter serviceは表から省略。
入力headerは両車と境界など必要観測の最も古いstampを保持し、frame_idとreference_idは必須。
reference_idは地図・参照ラインのrevisionを含め、形状変更時には変更する。
sは両車で同じ周回原点を共有した連続距離、dは左正、速度はs方向のm/s。
投影のヘアピン誤対応防止・自車運動補償・画像時刻整合は入力frontend側の責務。
observed=falseは喪失であり自由空間ではない。passing_allowedは直線、全区間幅、障害物なし、
合流までの相手観測可能性を確認した供給元だけがtrueにできる。
RGB/Infraは候補、Depthは任意。単眼bboxだけから無条件にメートル距離を作らない。
racing_observerが有効な接地点と閉じた参照ラインからこのtopicを生成する。

出力valid_for_secは元stampからの有効時間。publish時刻で鮮度を延長しない。
stations/offsetsは空間経路であり時刻付きの実行可能軌道ではない。
ReferenceLine・/auto/control_cmdへの自動変換は行わない。
時計停止時もsteady clock watchdogで無効化する。通常のデータ処理は入力イベント駆動。

## パラメータ

すべて起動時固定・read_only。pkg configは単体用、bringup/config/racingが運用設定正本。

| 名前 | 既定 | 単位・意味 |
|---|---:|---|
| vehicle_length | 0.35 | m、両車の共通保守長（0..2） |
| vehicle_width | 0.19 | m、両車の共通保守幅（0..1） |
| margin | 0.08 | m、幾何余裕、正値 |
| min_gap | 1.0 | m、停止時車間・合流余裕、正値 |
| headway | 0.7 | s、速度比例車間、正値 |
| acceleration | 1.5 | m/s²、自車予測加速度、正値・未実測 |
| deceleration | 2.5 | m/s²、追従速度上限の減速度、正値・未実測 |
| max_curvature | 1.2 | 1/m、直線での候補曲率上限（0..10） |
| transition_min_distance | 3.0 | m、往路・復路それぞれの横移動距離下限（0より大、20以下） |
| transition_time | 1.0 | s、速度上限に掛ける距離係数（0より大、5以下） |
| prediction_horizon | 15.0 | s、RoC探索上限（0..30） |
| input_timeout | 0.2 | s、元stampとsteady受信期限（0..0.5） |

範囲の0はすべて除外。全値有限必須。車体・加減速値は実車校正済みではない。
予測モデルの上限速度20m/s、観測分散上限d=0.25m²/v=4(m/s)²は初版の固定境界。

## 実行・検証

```bash
bash scripts/build.sh --packages-up-to kart_bringup
source ros2_ws/install/setup.bash
ros2 launch kart_bringup racing_shadow.launch.py
# 単体起動時は --params-file でpkgまたはbringup YAMLを指定
```

launch引数なし。shadow launchは設定ファイルと値をログに表示する。
ROSなしのコアテスト:

```bash
c++ -std=c++17 -O2 -Iros2_ws/src/kart_racing/include ros2_ws/src/kart_racing/test/test_predictive_spliner.cpp -o /tmp/test_predictive_spliner
/tmp/test_predictive_spliner
PYTHONPATH=tools/app python -m pytest tools/app/tests/test_head_to_head.py
```

Studio「試走→Head-to-head→局所planner→Predictive Spliner」で理想観測試走。
長いコースでは試走時間300秒・3周以上を使う。GP被覆率を判断理由に表示する。
相手は基準ライン上（d=0）、実車応答は理想加減速で、ROSノードを起動しない。
シミュレーションは車体掃引の境界・静的障害物・相手干渉を別途検査する。
可変横位置の相手、遮蔽、実カメラ認識、ROS結合、Jetson性能、実機は未検証。

Predictive Splinerの待機理由は、GP観測待ち、追い越し許可なし、速度差不足、
開始車間不足、自車横ずれ、前方予測不足、予測時間不足、前後距離／区間長不足、
設定上の横余裕不足を区別する。複数条件が不成立なら最初に検出した理由を表示する。
これは候補探索の結果であり、追い越し不可能の証明ではない。Studioでは従来方式専用の
「推定未確定」をGPモードに表示せず、試走終了時にも待機理由を残す。

横移動の基準距離は `max(transition_min_distance, speed_cap * transition_time, sqrt(6 * abs(offset) / max_curvature))`。候補探索で1／1.25／1.5倍を評価する。往路・復路に同じ値を使い、曲率制約は維持する。パラメータは起動時指定。

Studioの距離可視化は、共通C++ plannerが評価した候補の開始・終了・横移動距離・
予測並走開始位置を使う。採用時は採用候補、棄却時は有効側の横移動距離が最短の候補を示す。
青＝横へ出る、紫＝追い抜き、橙＝戻る、の帯を基準ライン上に描画する。
帯は沿線距離の目盛りであり、横方向の軌道や安全性の保証ではない。
許可区間残長との差と、横移動完了前に並走が始まる場合を表示する。候補未評価時は
設定上の下限のみと明示し、曲率・追い抜き距離は未算入とする。
診断値はStudio用native出力で伝え、ROSメッセージの契約は変更しない。

## 停止車両の回避とFrenet観測

起動時固定planner設定: stationary_speed=0.12 m/s（速度+2σの上限）、stationary_hold=0.7 s
（同じIDのs,dがほぼ一定の継続時間）、avoidance_speed=1.0 m/s（停止車両迂回中の速度上限）。
進路上に停止車両があれば、既知の直線区間で左右の横移動、車幅、追従誤差用余裕、復帰距離を確認して
AVOID_STATIC→MERGE→PASSEDを出す。周回学習は不要。余裕不足はBLOCKED（目標0）、
通路失効・相手再発進・喪失は計画を破棄。対象は既定寸法のカート1台。
移動相手には従来の追従・学習済み追い越しを使う。接地点は車体中心の近似であり誤差は実測が必要。

Pythonノード/実行ファイル racing_observer は以下を使用する。絶対topic名、標準TF・ROS管理topicは省略。
独自service/actionなし。起動時固定 position_sigma_m=0.03 m は位置誤差の仮の下限。
|方向|topic|型|QoS|
|---|---|---|---|
|入|/planning/reference_line|kart_interfaces/msg/ReferenceLine|reliable / transient_local / 1|
|入|/racing/ground_points|kart_interfaces/msg/OpponentGroundPoints|best effort / volatile / 5|
|入|/racing/passing_corridor|kart_interfaces/msg/PassingCorridor|best effort / volatile / 5|
|出|/racing/observation|kart_interfaces/msg/HeadToHeadObservation|best effort / volatile / 5|

閉じた参照ライン・撮影時刻の地図→base_link TFが必要。0.35〜0.8秒の位置列から縦速度を推定。
0台/複数台・投影の曖昧さ・古い観測では観測無効。通路入力は参照revision・track_id・frame一致かつ
観測時刻以前0.2秒未満のみ採用。直線全区間の通行可能性と並走観測を確認する外部入力であり、
現状この認定を実車センサから行うpublisherはない。既定では静止車両手前でBLOCKEDになる。

Studioは候補ごとに地図上の車体掃引・曲率を照合してからC++ plannerが採用側を確定する。
「両側が同等なら優先する側」は同点時のみ使用する。ROSのshadow plannerは引き続き
PassingCorridorの左右幅を使用し、Studioの登録ポリゴンを実センサの通路認定として使用しない。
