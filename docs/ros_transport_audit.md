# RC走行用QoS・DDSの監査（2026-10-22）

現状は制御depth 1、sensor best effort、地図transient local、同一container内intra-processという
妥当な出発点。ただしJetson上の実効QoS・遅延・欠落率を未計測であり、レース向け最適設定とは未判定。
以下は静的確認と測定手順。既定のセンサ／DDS設定は変更していない。

## 確認した構成と改善候補

| 経路 | 現状 | 評価・次の確認 |
| --- | --- | --- |
| Joy→mux→bridge | 主にdepth 1。mux入力subscriberはbest effort、出力はreliable。指令期限150 ms | 維持。処理停止・記録併用時の入力年齢と出力間隔を測る。期限150 msを目標遅延と扱わない |
| RGB E2E | Resize入力2／出力1、TensorRT入出力1、複数の中間stageはDEFAULT | 優先調査。実効DDS depthとGPU側queueは別。対応parameter・実機版を確認して小さいdepthをA/B比較する |
| VSLAM/VGL | DDS depth5（localization側明示）、画像内部buffer10、IMU buffer50 | 同期と推定に必要な保持。E2Eと同じ理由で一律depth1にしない。追跡率・同期欠落・姿勢年齢で評価 |
| RealSense | 画像・CameraInfoはSENSOR_DATA、IMU QoS未明示 | ドライバの実効QoSと撮影stamp→受信年齢を取得。露光・USB遅延はDDSと分離 |
| EVS | direct接続、packet購読OFF。async内部queue64、最大年齢20 ms | depth数だけから遅延を推定しない。event→tensor窓終端・GPU処理・drop診断を照合 |
| rosbag | QoS override空、cache/compression queueは設定0 | wrapperは0をCLIに渡さず、rosbag2の既定値へ委ねる。無効／容量0とは断定不可。録画ON/OFFで制御への影響を比較 |
| CycloneDDS | loopback UDP、multicast OFF、カーネルsocket容量未指定 | 同一計算機の通信として維持。ドロップ・再送・CPU負荷が観測されたら調整。無条件のsocket拡大やDDS変更はしない |
| intra-process/NITROS | launchでintra-process有効、tensorはGPU buffer利用 | 利用可能性と実際のzero-copy成立は別。プロセス配置・subscriber追加によるコピーを計測 |

NVIDIA共通QoS変換ではprofileとdepthが別で、depth=0がDEFAULTのdepth初期化になる実装がある。
従って`SENSOR_DATA`の文字だけでdepth5と断定しない。前の一般的なROS SensorDataQoSの説明と区別する。
出典: [isaac_ros_common/qos.cpp](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/isaac_ros_common/src/qos.cpp)。
これは上流mainの確認であり、実機apt版の実装一致は未確認。起動中のendpointを正とする。

DDSのsocket bufferはOSにも制限され、要求値と実際の容量は一致しない場合がある。
候補parameterや共有メモリの条件は、実際の導入版の
[CycloneDDS公式設定資料](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html)で確認する。
共有メモリ化やFast DDSへの変更で、GPU転送を含む全段のコピーが自動的に無くなるとは扱わない。

## 起動中のメタデータ取得

JetsonのROS開発コンテナ内、現在の環境をsourceした別terminalで実行する。SSHは自動実行しない。

```bash
cd /workspaces
python3 -m tools.pipeline_audit.ros_snapshot \
  --output /workspaces/analysis/transport/baseline.json
```

環境変数・CycloneDDS設定、endpointの実効QoS、関連nodeのparameter dump、導入apt版、
UDP socket情報と5秒間のUDP counter差分を新規JSONへ保存する。画像/Tensorをsubscribeせず、
制御publish・parameter変更・sysctl変更・録画開始はしない。ROS graph探索・parameter要求の負荷は加わる。
同時点の一括snapshotではなく、各CLIコマンドを順に取得する。

既定topic/nodeは全モードの共通候補なので、未起動nodeは失敗情報として残る。
欠測は0や正常と解釈せず、部分取得はexit code 2、全ROS metadata取得成功は0。
既存ファイルへ上書きしない。namespace・対象を絞るなら`--topic`／`--node`をそれぞれ繰り返す。

```bash
python3 -m tools.pipeline_audit.ros_snapshot \
  --topic /realsense/infra1/image_rect_raw \
  --topic /visual_slam/tracking/odometry --node /visual_slam \
  --sample-seconds 10 --output /workspaces/analysis/transport/vslam.json
```

`Udp.RcvbufErrors`、`SndbufErrors`などはnetwork namespace全体のcounterであり、増えても
DDS単独の原因や特定topicの欠落を証明しない。非Linux／ツール不足ではnull／取得失敗を残す。
CLI実行ごとに最大10秒。起動中node多数の場合は取得に時間がかかる。

## 比較で決める項目

1. 同じ入力、FPS、モデル、電力modeでbaselineを取得。録画OFF/ONを分ける。
2. 既存の[offline E2E latency](../ros2_ws/src/kart_bringup/README.md#offline-e2e-latencyrgb)で
   relay受信→decode、relay受信→最初のcommandの中央値・P95・P99・最大・欠測数を比較する。
   USB・撮影・mux・bridge・サーボ時間はこの試験の対象外。
3. 実走行記録は既存pipeline_auditで元header年齢・新規frame間隔・制御間隔・STOP理由を照合する。
   bag時刻差はcallback間の純粋な通信遅延ではない。
4. E2Eのqueue/depth候補を一段ずつ変更し、同じ条件で繰り返す。
   平均だけ改善してP99・停止回数・新規frame間隔が悪化する候補は採用しない。
5. UDPドロップ等があり、CPU/GPU/録画負荷との関係を切り分けた後にDDS/socket候補を比較する。

速度3 m/sなら30 msで9 cm、150 msで45 cm進む。これは単なる移動距離換算であり、
障害物への接近量や許容遅延の保証ではない。コース幅・余裕・操舵応答から必要な予算を決める。
現段階で特定のP99値を達成済み、安全な走行上限とする根拠はない。

## 検証境界

metadata collectorの欠測、counter reset、読取コマンド限定、上書き拒否はportable testで確認する。
ROS graph・DDS実通信・GPU・実bag・録画併用・車両挙動の実行検証は未実施。
