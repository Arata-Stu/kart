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
