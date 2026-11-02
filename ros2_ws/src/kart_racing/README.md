# kart_racing

相手1台の追従・GP相手モデル・予測スプラインのC++17実験実装。
90 Hz VO、既存PP/PIDは変更しない。ROS非依存の共通headerをStudio試走とROS componentで使う。
**ROS出力はshadow専用。実車の制御経路への接続、画像検出・距離推定・追跡・Frenet投影は未実装。**

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
実画像frontendは未提供のため、現状実センサーからこのtopicは自動生成されない。

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
