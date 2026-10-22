# センサ・E2E・最終制御の記録解析

既存Kart Map Studioの「解析」タブでも、bag選択→解析→グラフ表示を実行できる。
UIの導入・実行環境は[アプリREADME](../app/README.md#走行記録の時系列解析)を参照。

取得済みrosbag2を読み、時間の跳ね上がりと、その時のセンサ・制御・状態を同じ時系列で比較する。
bagのパスは実行時に指定する。MCAP／SQLite3に対応し、ROS環境・GPU・車両接続は不要。
ROS再生、SSH、制御topicのpublish、車両設定の変更は行わない。

起動中のROS metadata・QoS・DDS/UDP設定は別入口の
`python3 -m tools.pipeline_audit.ros_snapshot --output <new.json>`で取得できる（標準ライブラリのみ）。
これはライブROSのgraph探索／parameter読取を行うが、画像・Tensorの追加購読は行わない。
既定UDP観測時間5秒、`--sample-seconds`は0..60秒。`--topic`／`--node`の反復で対象を上書きする。
既存outputは拒否、全ROS metadata取得成功はexit 0、未起動node等の部分取得はexit 2。
測定手順・制約は[RC走行用QoS・DDS監査](../../docs/ros_transport_audit.md)を参照。

```bash
# kartのproject rootで実行
python3 -m venv /tmp/kart-audit-env
/tmp/kart-audit-env/bin/python -m pip install -r tools/pipeline_audit/requirements.txt

KART_AUDIT_PYTHON=/tmp/kart-audit-env/bin/python \
  bash scripts/pipeline_audit.sh \
  --bag /path/to/recorded_bag --output /path/to/analysis/run01
```

`--bag`はmetadata.yamlを含むrosbag2ディレクトリ、または単独の`.mcap`。
録画を終了・flushしたデータを使う。既存outputへの上書きとbag内部への出力を拒否する。
入力bagは変更しない。Pythonは`KART_AUDIT_PYTHON`未指定なら`python3`。

RGB topicが異なる場合は`--source-topic /camera/color/image_raw`を追加する。
EVSでは窓終端のROS timestampを持つ軽量metadataをsourceにする。

```bash
KART_AUDIT_PYTHON=/tmp/kart-audit-env/bin/python \
  bash scripts/pipeline_audit.sh --bag /path/to/evs_bag \
  --source-topic /event_camera/tensor_timing --output /path/to/analysis/evs01
```

EVSのRAWや画素は解析しない。窓の積算時間・センサ時計とROS時計の対応を遅延ゼロとみなさない。
tensor_timingが未記録ならsource照合はできず、不足を報告する。

## 折れ線とスパイクの見方

`report.html`をブラウザで開く。PlotlyのJavaScriptを同梱し、外部サーバやCDNは不要。
ドラッグで拡大、ダブルクリックで全体へ戻る。全パネルのx軸が連動し、凡例で線を選べる。
イベント表の行をクリック／Enterすると、その時刻の前後1秒を全グラフで比較できる。

| パネル | 見るもの |
|---|---|
| Header age | `bag記録時刻 - 元header時刻`。負値・急増・一定offset |
| Exact same frame | 同じstampのsource→推論入力／結果／decoder／auto／最終制御の最初の記録差 |
| Cadence | bag記録間隔とsensor header間隔。途切れ・固まり・重複・逆行 |
| Synchronization | sourceと最近傍の左右IR／IMUのheader差。物理同期・時計校正値ではない |
| Control requests | auto・decoder・最終指令のsteering/throttle/brake/reverse。指令パルスも表示 |
| State | モード、E2E active、復帰、HOST経路、診断severity |
| Board | 基板が報告したservo/ESC設定µs。実測PWMではない |
| 応答比較（先頭） | 操舵＋角速度、駆動／制動＋加速度。左右に別単位、IMU topic／軸の切替、A/B手動時間差 |
| Motion | IMU角速度[rad/s]・加速度[m/s²]とodomの前進速度proxy。VSLAM速度は校正済みground truthではない |
| Diagnostic | CPU・温度・電力等、記録された診断の数値。必要な線を凡例で有効化 |

RGBと`/event_camera/event_image`の表示ずれも、両topicのHeader age・Cadenceと
Synchronization（最近傍header差）で比較できる。画像payloadは展開しない。
event imageは実運用YAMLで40ms窓・40ms stride（25Hz）。これを固定40msの配送遅延と扱わない。
header差は同じ出来事の映像差や物理同期を示さない。EVSの受信由来offsetやpublish時刻に近いheaderは、
内容の古さを隠す場合がある。同じ動きを映した場面の目視・RAW時刻等と照合して判断する。
bag記録時点のずれと、player・表示subscriberによって再生時に加わるずれを分ける。
EVS clockパネルは、同じstream/epoch内の「記録経過時間−EVSセンサ経過時間」と
sensor-to-ROS offsetの変化を表示する。受信由来offsetの更新でheader ageが小さく保たれても、
センサ時刻の進みの不足を別に確認できる。segment開始時点ですでに存在する遅れは分からず、
物理的な映像遅延やハードウェア時計の校正値とは扱わない。epoch間を結合せず、欠測は0としない。

画像header間隔も同時に開くならセンサ生成／記録欠落を候補とする。
header間隔は一定でbag側だけ固まるなら転送・callback・recorder負荷等が候補となる。
同一stampの指令再配信が増えた場合は新しい推論が届いていない可能性がある。
いずれも観測からの候補であり、このツールだけで原因を断定しない。

## 指令とIMUの応答を折れ線で比較する

「時系列」の先頭に、最終制御指令とIMUの比較を追加した。
制御topicは`topics.final`を優先し、記録がなければauto、decoderの順で選ぶ。

- 上段: steering（左軸、正規化指令）＋角速度（右軸、rad/s）。
- 下段: throttle / brake / reverse（左軸）＋加速度（右軸、m/s²）。加速度未記録なら下段は出ない。
- 「角速度 topic / axis」「加速度 topic / axis」で比較するセンサ軸を選ぶ。
  既定は`/realsense/imu`のgyro_y／accel_xを優先するが、車体ヨー／前後軸への対応を意味しない。
  frame_idを凡例・選択欄に示し、記録にない場合はunknownとする。
- 指令が変わる点A、IMUが変わり始める点Bを順にクリックすると、縦線と`B−A`のmsを表示する。
  「A/Bを選び直す」でリセット。拡大・凡例・共通時間軸の操作は従来どおり。

**時間軸は両方ともbag記録時刻**。header時刻やコマンドの元画像stampを使って
車体応答を先行／遅延させない。機械系・タイヤ・速度・路面・記録時計のずれを含む定性的な比較で、
通信や推論だけの遅延の校正ではない。元timestampはシフトしない。モデルを仮定した遅れ推定は次節の任意操作で行う。
固定スロットル0.15のような一定入力区間から縦方向の遅延は読めないが、操舵が変化した区間は
角速度との比較に使える。信号が弱い区間・振動区間の立ち上がり判断には曖昧さがある。

角速度／加速度はsensor frameの生値。車体frameへのTF回転・重力除去は行わない。
ROS Imuのcovariance先頭が-1の成分は未提供として除外する（0の線で埋めない）。
旧版のmetadataには加速度・frame_id・利用可否がないため、全機能を使うには元bagを再解析する。
`timeline.csv`は元の全解像度を保持し、比較パネル用に同じデータを重複書込みしない。

## 操舵と角速度から見かけの遅れを推定する

「操舵と角速度から応答遅れをモデル推定」を開き、表示範囲と角速度軸を選んで
「応答遅れを推定」を押す。既定gyro_yは今回の車体で観測された相関に合わせた表示の選択で、
車体ヨー軸へ校正したことを意味しない。加速度からの自動推定は行わない。

モデルは`angular(t) = b + G * steering(t - τ)`。Gの符号も適合するため、逆相関を扱える。
`ω ≈ -Kδ(t-τ)`という表記なら`K=-G`に相当するが、δは実舵角ではなく正規化指令。
単純な純遅れモデルなので、サーボ・車体のなまりやタイヤ・速度・路面・記録時刻の差もτに含まれる。
速度一定は未確認の仮定で、固定throttleも一定速度を証明しない。通信／推論遅延の校正値と扱わない。

- 表示用に間引いた点ではなく、元の全解像度の指令と角速度を使う。
- 指令は直前値を保持するZOHとして参照する。範囲外へ外挿せず、指令gapを跨いで補間しない。
  どのτ候補でも同じIMUサンプルを使える区間だけを採用する。
- 区間の先頭70%で各τのG/bを最小二乗適合し、そのRMSEが最小のτを選ぶ。
  後半30%はτ選択に使わず、選んだモデルのR²／RMSEを確認する。
- 既定探索は0..500 ms、5 ms刻み。上限は10..2000 msで指定でき、実際の探索点は5 ms刻み。
  5 ms未満の精度を主張しない。負のτは探索せず、0 ms端が最良なら境界警告を出す。
- 予測角速度を右軸へ紫の破線で重ね、τごとの誤差曲線も表示する。
  観測した指令／IMUの値・時刻は変更しない。
- 探索端の解、弱い適合、検証失敗、平坦／複数候補の誤差曲線は「不安定・参考値」とする。
  信号や連続区間が不足すれば「推定不可」とし、0 msで埋めない。

品質判定の既定値は解析用のヒューリスティックで、統計的な保証ではない。
有効データは80点以上・1秒以上、適合区間の入力標準偏差0.03以上（正規化）、
角速度標準偏差0.02 rad/s以上。R²適合0.5未満／検証0.3未満で警告する。
最小MSE＋角速度分散の2%以内にあるτ候補の最小〜最大を「最小誤差近傍」と表示する。
これは信頼区間ではない。幅がmax(50 ms, 探索上限の30%)を超えると曖昧として警告する。
指令gapはmax(50 ms, 指令の記録間隔中央値×3)。同一／逆行時刻は拒否し、元サンプル10万点超は区間短縮を求める。

範囲／角速度軸／探索上限を変えると、古い推定線と保存結果を無効化する。
グラフのzoom／panも表示範囲へ反映する。「推定JSONを保存」はブラウザ内で作成した
`response-delay-estimate.json`を保存する。τ/G/b・適合／検証・実効区間・件数・設定・警告・
誤差曲線・元topic／bag／provenanceを含む。静的summary.jsonへクリック結果を自動書込みしない。
実装は`response_delay.js`、UI接続は`response_delay_ui.js`で、report.htmlへ同梱する。
単純なランプ入力ではbとτを区別できず、周期信号でも複数解になり得るため、入力変化が複数ある区間で確認する。

## 集計を混同しない

- 「条件確認済み」は、直前のmode=AUTO、E2E status=active、復帰active=falseが期限内に記録され、
  同じsource stampが一意に見つかった最初の制御指令だけを集計する。
- 復帰topic未記録、MANUAL、STOP、状態の鮮度喪失等は理由を付けて除外する。
  同じstampの観測差は「状態未確認も含む」集計と別の線に残す。E2E由来の保証はしない。
- 同じ画像stampの指令再送と、新しい画像からの最初の出力を区別する。
  再送中に増えたheader ageを初回推論時間として平均しない。
- source timestampの重複・再利用で対応が曖昧なら除外する。近いstampで代用せず、1ns違えば別frame。
  記録順序が逆で負の差になる場合も、負の処理時間として集計しない。
- 時計を自動シフトしない。未来stamp、逆行、invalid stamp、配信gap、診断warning、状態変化、
  大きいheader ageをイベントへ保存する。閾値は分析用であり車両の許容値や校正値ではない。

## 成果物

| ファイル | 内容 |
|---|---|
| report.html | 単独で開ける時系列グラフ・統計・除外理由・topic一覧 |
| summary.json | 平均/P50/P95/P99/最大/最小、照合率、時計問題、欠けたtopic、全設定と制約 |
| timeline.csv | 全解像度のlong形式時系列。panel,series,t_s,value,unit |
| matches.csv | 同一frameの全指令照合、初回／再送、条件判定と除外理由、絶対ns時刻 |
| events.csv | スパイク・gap・状態変化・warningと直前のmode/E2E/復帰/HOST/ESC設定 |
| metadata.jsonl | 抽出した小さいmetadata。画素・Tensor本体は含めない |

表示間引きは各区間の最小・最大・先頭・末尾を残し、一点だけのパルスを消さない。
CSVは間引かない。欠測の大きな区間では折れ線を切る。
入力manifestはファイルサイズ・mtime、設定にはSHA256を保存する。bag全体の内容ハッシュではない。
メモリ使用量は抽出metadata件数に比例する。画像・TensorはHeader先頭のplain CDR
little/big endianを読み、画素／Tensor本体をdeserialize・保持しない。
未対応serializationはエラー行として明示する。

## 設定とCLI

`config.json`をコピーし`--config /path/to/config.json`へ指定する。ROS parameterではない。

| 設定 | 既定 | 意味 |
|---|---|---|
| schema | 1 | 設定schema |
| topics | config.json参照 | source、tensor_input/output、decoder、auto、final、mode、e2e_status、recovery_active/status、active_path、output_channels、evs_timing、logsの絶対topic名 |
| sync_topics | 左右IR・IMU | 最近傍header差を調べるtopic |
| state_timeout_s | 0.5 | E2E/復帰等の記録状態の有効期間[秒] |
| mode_timeout_s | 0.3 | modeのheader・記録時刻の有効期間[秒] |
| match_window_s | 2.0 | 同じstampのsource候補を探すbag時刻差の上限[秒] |
| future_tolerance_ms | 5.0 | headerが記録時刻より先にあると警告する閾値[ms] |
| age_spike_ms | 100.0 | header age／照合した記録差のspike検出[ms] |
| gap_factor | 3.0 | topicごとのbag記録間隔中央値に対するgap閾値倍率 |
| gap_floor_ms | 20.0 | gap閾値下限[ms]。実効値はmax(floor, median×factor) |
| plot_max_points | 6000 | 各線の表示点数上限。解析とCSVに影響しない |

`--source-topic`はsourceだけを上書きする。RGB/EVSの自動推定による時計変換はしない。
`--records-json`はテスト用の抽出metadata JSON入力（bagとの同時指定不可）。
topic型の不一致、欠損、未記録をreportへ残す。Image／CameraInfo／IMU／Odometryと
DiagnosticArrayはbag内から発見して追加する。rosoutは記録されている場合のみ利用する。

## 測れないものと次回の記録

測るのはheaderと**recorderの記録時刻**、および同じframeの記録差。
画像callback開始、publish時刻、GPU kernel単独時間、bridgeのserial write、基板受信、
ESC端子の実波形、物理的な制動／操舵応答は、通常のbagだけでは分離できない。
同じ時刻のスパイクだけでセンサ・通信・GPU・車体の責任と断定しない。
bag時計リセットを跨ぐ元の受信順序も、時刻でmergeされた記録からは復元できない。

通常bringupのbagには画像・IMU・最終指令・auto・mode・recovery・E2E status・基板出力・診断を指定している。
復帰が無効ならdecoderは直接autoへ流れ、recovery/input_cmd等の欠落は仕様どおり。
tensor入力／出力とrosoutは標準では含まれない場合がある。
追加の短時間診断で/e2e/tensor_inputと/e2e/tensor_outputを保存すると段階が増えるが、
GPU Tensorのhost転送や記録負荷がタイミングを変える可能性がある。
本番bag設定や実行ノードはこのツールによって変更しない。
設計上の境界は[時刻解析方針](../../docs/policies/timing_audit.md)も参照。

## 検証

```bash
/tmp/kart-audit-env/bin/python -m pip install pytest
/tmp/kart-audit-env/bin/python -m pytest tools/pipeline_audit/test_audit.py -q
```

既知の遅延・再送、AUTO/MANUAL/復帰の分離、時計逆行・未来stamp・順序逆転、曖昧なframe照合、
一点spikeの表示維持、実SQLite3/MCAP形式の合成bag、画像をdeserializeしないこと、
既知の負ゲイン・オフセットと遅れの回復、holdout不一致、ランプ／周期信号の曖昧さ、gap除外、
全解像度のモデル入力、区間・軸変更時の推定無効化、
HTML生成とエスケープ、上書き拒否を検証する。
実走行bagによる解析結果・ROS/GPU・実センサ同期・実PWMの妥当性は未検証。
