# ROS bagとnative sensor記録の連携

- session directoryの所有者はkart_bag_manager。rosbag子プロセスの出力directory生成後にのみ
  native recorderへ絶対パスを渡す。driverからも同じパスが見えるmountを使用する。
- native RAWはOpenEB writer、MCAPはrosbag writerが別ファイルとして保存する。
  EventPacketやGPU tensorをnative RAWの代わりにrosbagへ重複保存しない。
- sensing repoにkart_interfaces依存を持ち込まず、manager側のadapterで標準ROS parameter/serviceに接続する。
  保存先設定の成功応答を待ってSTART。START/STOP/SPLITは非同期で順序を保つ。
- RAWの成功・失敗はbag/raw_diagnosticsで別途報告する。bag/status.recordingだけでRAW成功とは判定しない。
  RAW失敗によって既存rosbagを自動停止しない。
- RAWを単独で手動開始した場合と、bag managerが所有するRAW sessionを区別する。
  active RAWの保存先変更を拒否し、開始を拒否されたmanagerが手動writerをSTOPしない。
- 時間分割の正本はbag manager。連携時はdriverの独立分割timerを無効化する。
  同じ周期でもMCAP/RAWの分割境界が厳密同期するとは扱わない。
- ROS contextを生かしたままRAW STOP応答・rosbag終了処理を待つ。launchの終了猶予は
  bagのstop/terminate timeoutとRAW service timeoutに余裕を加える。
- 可視化画像はsensor側の独立した任意経路。packet配信OFFでも画像出力が可能。
  画像の時刻はpacket受信時刻への推定対応であり、hardware同期を保証しない。

## セッション保存

bringupではsession_layout=trueで起動日時/date/time/run_nameへ保存する。
最初のSTARTまでdirectoryは作成しない。同セッション再録画はname_01等で既存資産を保持する。

## EVS学習時刻契約

RAW relative_usはreaderのshiftを明示してセンサ時刻へ戻す。
Tensor windowごとにsensor_to_ros_offset_nsをbagへ保存し、整数nsで対応付ける。
録画開始ROS時刻とRAWの0を等置しない。Tensor headerとwindow endも区別する。
RAW splitごとのshift、sensor resetごとのepoch、メタデータsequenceの欠落を確認する。
packet topic/Tensor本体は標準収集対象外。offsetは受信遅延を含む推定で実機校正が必要。


EVS制御dataset抽出器はRawReaderのdo_time_shifting=falseを使い、RAWを最初から
センサ絶対usで読む。この場合はshift sidecarを加算しない。
相対us readerを使う別ツールは従来のshift復元式を使う。両方式を混同しない。
複数stream/epochのRAWはmanifestで対応を明示する。窓は記録済みTensorTimingから取得し、
教師の対応時刻はTensor stamp、境界は[start,end)。RAW端・splitをまたぐ不完全窓は除外する。


### 単一RAW内の処理epoch（2026-11-06）

bag直下にRAWが1件あり、TensorTimingが同じstreamの場合、複数sensor_epochでも
sequence・epoch・窓終了の前進と、epoch境界でのlatest event時刻の非逆行を確認して自動対応する。
40msのスライディング窓はepoch境界でも重なりを許容する（同一終了時刻の重複は拒否）。
`--raw`で単一RAWを指定した場合も同じ規則を使う。これは上記の複数epochに対する
manifest必須規則の例外で、Tensor処理履歴のresetでもepochが増えることに対応する。
窓や時刻は補正せず、各サンプルの元epochを保持し、metadataのraw_filesにsensor_epochsを記録する。
RAWは同じreaderで順に読み、既存の時刻逆行検査を維持する。
複数stream、センサー窓終了の逆行・同値、epoch境界のlatest event逆行、
複数RAWと複数epochの組合せは自動対応しない。拒否時は前後のsequence・epoch・時刻を表示する。
単体テストで連続epoch・重なる窓の抽出と逆行・同一終了時刻の拒否を確認。実bag・SDKでの結合確認は別途必要。


### RAWのepoch別時刻対応を明示する場合

RAW SDKの連続時刻と記録TensorTimingの時刻原点が異なると実データで確認できた場合、
manifestの各files要素に任意の`epoch_offsets_us`を指定できる。
例: `"sensor_epoch": 0, "epoch_offsets_us": {"0": 0, "2": 33554432}`。
キーは対象epochの10進文字列、値はRAW座標へ加算する整数マイクロ秒。
同じRAWを複数行にせず、1行で対象epochを列挙する。指定しない従来manifestは補正0を維持する。
窓の開始・終了・latest eventにのみ加算し、教師のROS時刻と元timingは変更しない。
補正後の全窓をstride・教師選択前に検査し、逆行や同一終了時刻は拒否する。
サンプルにraw_time_offset_us/raw_window_start_us/raw_window_end_us、metadataのraw_filesに
明示対応を保存する。受信時刻だけから補正を自動推定しない。
RAW自体の小逆順は従来のmax_reorder_usで検査し、補正で回避しない。


### Forward packet gap recovery (2026-11-06)

`evt3_runs`, `evt3_runs_gpu`, `evt3_runs_gpu_direct` のdirect histogram経路は、
パケット連番が前方へ飛んだ場合（stale queue破棄を含む）、座標・LOW時刻・集計履歴を
破棄してHIGH/LOWを再取得する一方、HIGH時刻と24-bit折り返し累積を保持する。
短い欠落がTensorTimingの時刻を33.554432秒等巻き戻すことを防ぐ。
epochは引き続き更新し、欠落イベントや不完全な履歴を補完しない。
連番逆転・再開始、geometry変更、decode例外は従来のfull resetのまま。
連番不連続時はprevious/current/preserve_clockをWARNへ出力する（1秒throttle）。
新規parameter/topicはなし。20ms queue age、4000us逆順許容など既存設定は変更しない。
これは短いforward gapの復旧であり、複数周（1周16.777216秒）を跨ぐ長い入力断や、
連番を維持したセンサー時計resetの自動識別を保証しない。grouped_gpu経路は対象外。
`evt3_runs_histogram_test`で2周累積後の欠落、欠落中のwrap、再取得、full resetを検証。
ROS/CUDA結合・実機録画は未確認。既存RAW/timingは書き換えず、過去記録のmanifest補正は引き続き必要。


## EVS / action / RGBの時間軸（2026-11-06確認）

- RAW reader: `do_time_shifting=False`のデコード時刻（us）。オンラインdecoderと周回原点が一致する保証はない。
- TensorTimingのwindow_start/end_sensor_us、latest_event_sensor_us: オンラインdecoderのセンサー時刻（us）。
- Tensor header / tensor_stamp_ros_ns: ROS時刻（ns）。窓終了ではない。最新適用packetのROS時刻と
  予定出版時刻の小さい方を基に、前回headerより1ns以上進める。
- packet header: driverがpending packetを作成した際の`now()`。露光時刻・ハードウェア同期時刻ではない。
- sensor_to_ros_offset_ns: packet header ROS時刻 − 最新decoded eventセンサー時刻×1000。
  受信由来の推定で、パケットごとに変わり得る。
- action（/teleop/control_cmd）header: Joy managerが指令を出版するときのROS `now()`。
  モードheaderもROS `now()`。実車応答の時刻ではない。
- RGB: 比較ツールではImage.header.stampをROS時間軸の値として使用する。
  実機ドライバの時計変換・露光との遅延は別途確認が必要。

manifestのepoch_offsets_usをΔとすると:

```
raw_decoder_us = online_sensor_us + Δ
ros_ns = (raw_decoder_us - Δ) * 1000 + sensor_to_ros_offset_ns
```

RAWをshift有効で読む別経路では、まずSDK timestamp shiftを足してshift無効の座標へ戻す。
SDK shiftとΔは異なる値であり、二重加算しない。既存RAW shiftツールのsensor_usという名前は
RAW decoder側の座標を意味し、オンライン時刻への一致を保証しない。
学習教師はtensor_stamp_ros_ns以前の最新指令を選び、既定100ms以内。
新しいsamples.jsonlにはcommand_stamp_ns / command_age_nsも保存する。
`--clock header`を比較の基準とし、bag時計はcommand/modeにbag受信時刻を使う別方式として扱う。
固定AUTO設定のStringにはheaderがないためbag受信時刻を使う。

最初の検証目標はoffsetレベルの整合。窓の重なり、処理・露光・受信遅延をゼロと仮定しない。
RGBとEVSに共通して写る物体を左右に数回動かし、開始・中間・終了で位置と動きの対応を見る。
一定のずれはoffset、増大するずれはclock drift、途中の飛びはreset等として切り分ける。
視点・解像度・40ms集積の差を考慮し、これをマイクロ秒同期や空間校正の証明としない。


### Dataset作成時のEVT3時刻自動補正

単一RAW・単一streamでは、manifest未指定時（単一`--raw`も含む）に自動補正する。
RAWをshift無効で全走査し、記録開始・終了の両端とTensorTimingの最新イベント時刻が
それぞれ1秒以内で一致する16,777,216usの整数倍を一意に選ぶ。単なる範囲重複では選ばない。
epoch間はsensor_to_ros_offset_nsの差から整数周回差を求め、全timingの残差が1秒以内、
同一epoch内の補正値一定、補正後時系列の前進を検査する。
この1秒は周回候補の検査幅であり、任意の1秒補正や同期精度の保証ではない。
RAW自体の逆行はmax_reorder_us（既定5000us）を超えれば停止する。
RAW.metadata.yamlがあればRAW名と録画開始ROS時刻を照合し、最初のtimingと2秒以上離れていれば拒否する。
古い記録でsidecarがなければ両端とoffset系列の検査で判定する（根拠の欠如をmetadataに記録）。
異なる録画の同じ長さ・周回差を区別するイベント一致の証明ではないため、初回はRGB比較等で検証する。
複数stream/RAWや長い先頭末尾欠落、整数周回で説明できないresetは明示manifestが必要。
手動manifestは優先し、自動補正を二重適用しない。
補正値と判定根拠はdataset metadata.jsonのraw_files[].epoch_offsets_us / auto_alignmentへ保存する。
元RAW・bag・TensorTiming・教師ROS時刻は変更しない。UIのRAW手動指定欄を空欄にして利用できる。
既存datasetは変わらず、再作成時に適用する。RAW全走査が1回増えるため抽出時間は増える。
新規ROSノード/topic/parameterはなく、既存CLIの標準処理として実行する。
