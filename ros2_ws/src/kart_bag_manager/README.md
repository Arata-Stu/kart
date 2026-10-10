# kart_bag_manager

JetPilotの`jetpilot_bag_tools/bag_manager_node.py`を基にしたrosbag管理。
保存先の重複回避、MCAP、topic選択、分割、圧縮、QoS override、任意のRAW録画連携を引き継ぐ。
録画の開始・停止と走行モードは独立。自動録画・走行開始との連動は行わない。

## ノード・構成

| 項目 | 既定名 |
| --- | --- |
| package | `kart_bag_manager` |
| 管理ノード | `/kart_bag_manager` |
| 実行ファイル | `kart_bag_manager_node` |
| 子プロセス | `ros2 bag record`（ノード名`kart_rosbag_recorder`を指定） |

Python＋`ament_cmake_auto`/`ament_cmake_python`。setup.pyは使用しない。
録画プロセスの監視とファイル終了処理を走行containerから独立させるため、C++ Componentではなく
独立したPythonプロセスで起動する。記録データは`ros2 bag record`の子プロセスが購読する。

## 入出力topic

以下はnamespaceなし・remapなしの名前。managerのtopicは相対名なので、namespace=`kart`では
`/bag/request`は`/kart/bag/request`になる。ROS標準の`/rosout`・`/parameter_events`等は省略。
managerのQoSはすべてreliable / volatile / keep-last depth 10。

| 方向 | 既定topic名 | 型 | 内容 |
| --- | --- | --- | --- |
| 入力 | `/bag/request` | `kart_interfaces/msg/BagRequest` | START=1、STOP=2、SPLIT=3、MARK=4 |
| 出力 | `/bag/status` | `kart_interfaces/msg/BagStatus` | 既定1 Hz＋要求・イベント時。recording/current_uri/last_event/message |
| 条件付き出力 | なし（無効） | `kart_interfaces/msg/BagRequest` | `raw_recording_driver_node` | `""` | OpenEB driverの絶対node名。空で標準サービス連携無効。例`/event_camera/event_camera_driver` |
| `raw_recording_service_timeout_s` | `5.0` s | サービス利用可能待ち・応答待ちの各期限。正の有限値 |
| `raw_recording_request_topic`設定時のみRAWレコーダへ要求を配信 |
| 子プロセス入力 | `topics`に列挙した名前、または全topic | 各topicの発見された型 | 実際のbag内容。QoSはrosbag2の適応設定またはoverrideファイル |

既定topic一覧は[config/bag_manager.yaml](config/bag_manager.yaml)。走行指令・モード・Joy・基板状態・
トリム・bag要求/状態・diagnostics・TFを記録する。D455のRGB/IR/IMUとVSLAMは候補名を含む。
**D455/localizationのlaunchは未実装なので、センサーtopic名は実構成に合わせる必要がある。**
相対topicはmanagerのnamespaceで解決し、`/tf`と`/tf_static`は既定でglobal名を維持する。
no_discovery=falseなら録画開始後に現れた対象topicも発見対象となる。

条件付き出力`/bag/raw_diagnostics`はdiagnostic_msgs/msg/DiagnosticArray、reliable / volatile / depth 10、
既定1 Hz。標準topicsにも含める。RAWサービスの成功応答・失敗・timeout・所有状態を報告する。
`raw_recording_driver_node`が空の場合は配信しない。

独自サービス・actionはない。RAW連携時だけdriverの標準サービスをclientとして使用する。
`/event_camera/event_camera_driver/set_parameters_atomically`（rcl_interfaces/srv/SetParametersAtomically）で
保存先を設定し、`/event_camera/{start,stop,split}_raw_recording`（std_srvs/srv/Trigger）を順序付きで呼ぶ。
既定は連携無効。driverノード名を指定した場合、そのnamespaceからTrigger名を解決する。pause/snapshot等を有効にする場合は、子rosbag2 recorderのサービスを
別途操作する。本managerはそのサービス状態を監視しない。

## L1 / R1と状態

`kart_joy_manager`のL1がSTART、R1がSTOPを発行する。走行モード・deadmanとは独立し、
同時押しはSTOP優先。長押し・接続時に押されていたボタンでは繰り返し要求しない。
**R1は録画停止であり、車両停止は×。** Joy切断で録画を自動停止しない。

状態`message`は`idle` → `starting` → `recording` → `stopping` → `idle`。
失敗・異常終了は`error`となり、`last_event`に理由を示す。
recording中のSTARTは無視、stopping中のSTARTも無視する。終了後に改めてSTARTすると別の保存先を作る。

`recording=true`は子プロセス生存＋出力ディレクトリ生成の確認を意味し、メッセージ受信や
完全な記録を保証しない。start_paused/snapshotを設定してもこの意味は同じで、messageに注意情報を付ける。
停止要求時にはrecording=falseになるが、**stopping中は終了処理中**。
正常終了コードとmetadata.yamlの存在を確認してidleへ移る。これは全データの内容検証ではない。
current_uriは最後の試行先を保持するので、URIだけで録画成功を判定しない。

## 保存先・停止・RAW連携

- 既定保存先は`/workspaces/record`。kartのproject root mountによりホストと共有する。
- session_layout=true（bringup既定）では起動日時で `YYYY-MM-DD/HH-MM/recording_name`。空名はrun。
  同じセッションの再録画は_01等。STARTまで親directoryも作らず、単なる起動/終了では何も作らない。
- 以下の旧形式はsession_layout=false（単体package既定）の場合。
- recording_nameが空なら`YYYYmmdd_HH-MM_label`。ラベル内のパス記号等は置換する。
- recording_nameありなら`YYYY-MM-DD/recording_name`。既存パス・symlinkがあれば`_01`等を付け、上書きしない。
- CLIは引数配列で実行し、shellを通さない。子を独立したPOSIX process groupにする。
- 50 ms周期で起動・終了を監視し、通常コールバック中に待機しない。STOPは起動待ち中にも受け付ける。
- SIGINTでflushを待ち、既定10 s後にSIGTERM、さらに5 s後にSIGKILL。強制終了はerrorとして報告する。
  node終了時のみ同じ終了処理を同期的に待つ。manager自体へのSIGKILLでは後始末は保証できない。
- 手動SPLITはJetPilot同様に無視し、recording_split_duration_sを使用する。
- RAW連携を有効にした場合、開始確認後にSTARTとcurrent_uri、停止・異常時にSTOPを送る。
  時間分割時はRAW側へSPLITを送るが、MCAPとRAWの境界が厳密同期する保証はない。
  サイズ分割はMCAPのみで、RAWには連動しない。MARKはlast_eventとRAW要求に残す。

## パラメータ

managerのすべての設定は起動時固定（read-only）。独自parameterの基準値はノードと個別pkgのconfigで一致する。
共通の`use_sim_time`はconfigでfalse。実運用では
[kart_bringup/config/recording/bag_manager.yaml](../kart_bringup/config/recording/bag_manager.yaml)を使用し、
単体launchのみ個別pkgのconfig/bag_manager.yamlを使用する。

| 名前 | 既定値 | 説明 |
| --- | --- | --- |
| `output_dir` | `/workspaces/record` | 親ディレクトリ。空不可。起動時に作成 |
| `session_layout` | false | 起動日時/date/time/name形式。bringup運用YAMLはtrue |
| `recording_name` | `""` | 固定フォルダ名。単一名・120 bytes以内。空なら日時＋label |
| `record_all` | `false` | trueなら`--all-topics`、falseならtopicsを指定 |
| `topics` | config記載のリスト | 記録topic。record_all=falseかつ空ならSTART拒否 |
| `exclude_topics` | `[]` | 除外topicのリスト（正規表現ではない）。明示topicsでは一覧から除外、record_allではCLIへ渡す |
| `storage_id` | `mcap` | storage plugin。必要なpluginをインストールする |
| `serialization_format` | `""` | 空ならrosbag2既定 |
| `max_bag_size` | `0` bytes | サイズ分割。0で無効、非負 |
| `recording_split_duration_s` | `0` s | 時間分割。0で無効、非負整数 |
| `max_cache_size` | `0` bytes | 正値だけCLIへ渡す。0はrosbag2既定を使用（cache無効化ではない） |
| `compression_mode` | `""` | 空 / none / file / message |
| `compression_format` | `""` | 例zstd。対応pluginが必要 |
| `compression_queue_size` | `0` | 正値のみCLIへ渡す。0ならrosbag2既定 |
| `compression_threads` | `0` | 正値のみCLIへ渡す。0ならrosbag2既定 |
| `qos_profile_overrides_path` | `""` | recorder購読のQoS override YAML |
| `include_hidden_topics` | `false` | hidden topicも発見対象にする |
| `no_discovery` | `false` | trueで開始後のtopic discoveryを無効化 |
| `snapshot_mode` | `false` | rosbag2 snapshotモード。別途snapshotサービス操作が必要 |
| `start_paused` | `false` | 一時停止状態で起動。別途resumeサービス操作が必要 |
| `extra_args` | `[]` | ros2 bag recordへの追加引数。shell文字列ではなく配列。管理する主要flagの上書きは拒否 |
| `status_period_s` | `1.0` s | 状態配信間隔。正の有限値。内部監視50 msなのでそれより細かい配信は不可 |
| `raw_recording_request_topic` | `""` | 空で無効。例`/event_camera/raw_recording/request`。bag/request自身は指定不可 |
| `recording_start_timeout_s` | `5.0` s | ディレクトリ生成待ち。正の有限値 |
| `stop_timeout_s` | `10.0` s | SIGINT後の待機。正の有限値 |
| `terminate_timeout_s` | `5.0` s | SIGTERM後の待機。正の有限値 |

JetPilotの廃止済み`max_bag_duration`互換パラメータは移植せず、recording_split_duration_sに統一した。
LyricalのCLIに合わせ、旧位置引数topic／`--exclude`を`--topics`／`--exclude-topics`へ変更した。
Lyricalは`--topics`単独と`--exclude-topics`の併用を拒否するため、明示topic録画では
除外後の一覧だけを`--topics`へ渡す。全topicが除外された場合は開始を拒否する。
状態ログはinfo/errorの呼出し箇所を分け、録画失敗時にもログseverity変更でノードを落とさない。
[ROS 2 Lyrical公式CLI実装](https://github.com/ros2/rosbag2/blob/lyrical/ros2bag/ros2bag/verb/record.py)と照合した。

## ビルド・起動

```bash
cd /workspaces/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to kart_bringup
source install/setup.bash
# 全体起動: managerは既定で起動するが、録画はL1まで始めない
ros2 launch kart_bringup vehicle.launch.py
# 単体起動
ros2 launch kart_bag_manager bag.launch.py
# CLIからの操作（録画開始／停止のみ。走行は操作しない）
ros2 topic pub --once /bag/request kart_interfaces/msg/BagRequest '{command: 1, label: test}'
ros2 topic pub --once /bag/request kart_interfaces/msg/BagRequest '{command: 2}'
ros2 topic echo /bag/status
```

bag.launch.py引数はnamespace（空）、config（package shareのconfig/bag_manager.yaml）、
shutdown_timeout（20 s）。launchがmanagerへSIGTERMを送るまでの猶予を確保する。
vehicle.launch.pyの既定値はbringup/config/bringup.yamlで管理し、enable_bag_manager=true。
運用YAMLのoutput_dirが保存先となり、record_dirを明示した場合だけ上書きする。
bag_configを明示すると運用の録画設定ファイル自体を差し替える。
終了猶予bag_shutdown_timeoutも運用YAMLで20 sを指定する。stop/terminate_timeout_sを延ばす場合はこの猶予も延ばす。
二重にmanagerを起動しない。録画中の子ログとmanagerログは起動端末に出る。

## 検証

```bash
# プロジェクトroot、ROSなしで実行可能
PYTHONPATH=ros2_ws/src/kart_bag_manager python3 -m unittest discover -s ros2_ws/src/kart_bag_manager/test -v
# ROS環境
colcon test --packages-select kart_bag_manager kart_system
colcon test-result --verbose
```

macOS上で、制御下のダミー子プロセスを用いた開始・停止・flush模擬・異常終了・timeout・強制終了、
保存名重複、引数生成を確認。L1/R1のedge・長押し・同時押し・再接続もC++単体テストで確認。
**実rosbag2でのMCAP生成・再生、ROS結合、Jetsonでの帯域・容量・終了時flush、RAW連携は未検証。**
初回はbag/statusだけでなく、停止後の`ros2 bag info <current_uri>`と実際の記録内容も確認する。

Jetsonのハードウェア監視`/system/jetson/diagnostics`（diagnostic_msgs/msg/DiagnosticArray、
reliable/volatile/depth1、既定2Hz）を標準topicsに含める。
vehicle launchの公式jtop nodeが配信するCPU/GPU/温度/電力等を通常の録画START/STOPで保存する。
独自topicリストやexcludeを使う場合は明示追加が必要。node起動だけでは録画しない。
`ros2 bag info`でtopicとmessage countを確認する。実Jetsonでの記録は未確認。


## OpenEB native RAWと同じsession directoryへ保存

`raw_recording_driver_node`を設定すると、bag出力directory生成後に、その絶対パスを
OpenEBのraw_recording_dirへ設定する。成功応答を待ってRAW STARTを呼ぶ。
STOP/SPLITも順序付きで送信し、サービス失敗はbag/raw_diagnosticsへ報告する。
rosbagの録画状態とRAW開始成功は別で、RAW失敗だけではrosbagを停止しない。
packet topic・tensor payloadをrosbagへ保存せず、OpenEBのnative RAWを別ファイルに保存する。

```text
session/
  metadata.yaml
  session_0.mcap
  openeb_..._000000.raw
  openeb_..._000000.raw.metadata.yaml
  openeb_..._000001.raw                 # RAW SPLIT時
```

全parameterを記載したconfig/bag_manager_openeb.yamlを用意した。
個別pkgの基準profileであり、実運用はkart_bringup/config/recording/bag_manager_openeb.yamlを使う。
OpenEB側はraw_recording_enabled=true、raw_recording_auto_start=false、
raw_recording_split_duration_s=0で起動する。RAWの分割周期はmanagerだけが所有する。
rosbagとdriverから同じ絶対保存パスが見える必要がある。

```bash
# カメラ側（openeb_ros2のkart branchを対象ROS/CUDA環境でbuildした後）
ros2 launch openeb_ros2 tensor.launch.py \
  driver_config:="$(ros2 pkg prefix openeb_ros2)/share/openeb_ros2/config/driver_bag_linked.yaml"

# manager単体。既にvehicle launchでmanagerを起動している場合は二重起動しない。
ros2 launch kart_bag_manager bag.launch.py \
  config:="$(ros2 pkg prefix kart_bag_manager)/share/kart_bag_manager/config/bag_manager_openeb.yaml" \
  shutdown_timeout:=26
```

bringupからはbag_configで運用のbag_manager_openeb.yamlを指定し、bag_shutdown_timeout:=26を指定する。
RAW連携時の終了猶予はstop_timeout_s + terminate_timeout_s +
raw_recording_service_timeout_s + 余裕を確保する（既定合計20 s、例26 s）。
開始待ち中のSTOPも保持する。応答timeout時はRAW成功と判定せず、開始送信済みならcleanup STOPを試みる。
手動RAWが既にactiveなら保存先変更が拒否され、managerはそのwriterをSTOPしない。

raw_recording_request_topicの既存BagRequest出力は他のRAW recorder用に残す。
OpenEBには標準サービス連携を使用するので空のままとする。
MARKはmanagerのlog/diagnosticに残し、RAWファイルの専用markerには変換しない。
サービス開始・停止、MCAP分割境界とRAW分割境界、sensor時刻の厳密同期は保証しない。

ROSなしのRAW relay追加12テストを含む21テストが通過。
実カメラ・MCAP/RAW同時生成・サービス結合・実機flushは未確認。

OpenEB学習収集では/event_camera/tensor_timing（std_msgs/msg/String、JSON v1、
reliable/volatile/depth 512、既定250 Hz）をOpenEB用bag profileへ追加。
GPU Tensorは記録せず、窓のセンサ時刻とsensor_to_ros_offset_nsを保存する。
RAWの相対時刻には各RAWのSDK timestamp shiftを足し、そのoffsetでROS時刻へ変換する。
録画開始ROS時刻をRAWの0へ直接対応させない。
RAW shift取得ツールはtools/evs_raw_timing（scripts/sensors/evs-raw-timing.sh）。
USB遅延を含む推定なので、実機で同期精度を確認する。
