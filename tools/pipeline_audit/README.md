# センサ・E2E・最終制御の記録解析

既存Kart Map Studioの「解析」タブでも、bag選択→解析→グラフ表示を実行できる。
UIの導入・実行環境は[アプリREADME](../app/README.md#走行記録の時系列解析)を参照。

取得済みrosbag2を読み、時間の跳ね上がりと、その時のセンサ・制御・状態を同じ時系列で比較する。
bagのパスは実行時に指定する。MCAP／SQLite3に対応し、ROS環境・GPU・車両接続は不要。
ROS再生、SSH、制御topicのpublish、車両設定の変更は行わない。

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
| Motion | IMU角速度とodomの前進速度proxy。VSLAM速度は校正済みground truthではない |
| Diagnostic | CPU・温度・電力等、記録された診断の数値。必要な線を凡例で有効化 |

画像header間隔も同時に開くならセンサ生成／記録欠落を候補とする。
header間隔は一定でbag側だけ固まるなら転送・callback・recorder負荷等が候補となる。
同一stampの指令再配信が増えた場合は新しい推論が届いていない可能性がある。
いずれも観測からの候補であり、このツールだけで原因を断定しない。

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
HTML生成とエスケープ、上書き拒否を検証する。
実走行bagによる解析結果・ROS/GPU・実センサ同期・実PWMの妥当性は未検証。
