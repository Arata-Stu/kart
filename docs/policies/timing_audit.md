# 記録からの時刻・遅延解析

- 読取専用オフライン解析を基本とする。bagを再生せず、制御topicをpublishしない。
- センサheader、bag記録時刻、callback受信時刻、publish時刻、steady時刻、実PWM計測を混同しない。
  未記録の段階の遅延・物理時計同期・GPU処理時間を推定値で埋めない。
- 元画像timestampを引き継いだ指令はexact stampで照合する。初回と再送を分け、
  MANUAL・STOP・復帰・出典不明の指令をE2E遅延へ混ぜない。
- 状態が未記録の同一stamp観測差も保存し、条件確認済み統計と明確に分離する。
  0サンプルは0msでも正常でもない。
- 自動の時計offset補正・近傍frameへの置換・欠けたTF／センサ情報の捏造は行わない。
- 時系列は共通時間軸にし、表示間引きで一点spikeを消さない。全解像度CSVと除外理由を残す。
- 相関と因果を分ける。VSLAM速度・基板出力設定・ESC指令モデルを物理計測と扱わない。
- 画像・Tensorを解析目的でCPUへ展開しない。途中Tensorの記録には追加負荷があることを明示する。
- 合成データ・実形式の合成bagによる検証と、実走行bag・ROS/GPU・実機の検証を区別する。
- DDS/QoSの判断は起動中endpointと実効parameterを確認し、DDS history・内部処理queue・OS socket bufferを分ける。
  profile名だけからdepthを決めつけない。監査手順は[RC走行用transport監査](../ros_transport_audit.md)。
  metadata取得は画像/Tensorの追加購読・制御publish・parameter変更を行わず、欠測を正常と扱わない。

## 再生によるE2E latency試験

- bringupのoffline入口からRGB bag→実運用image encoder→TensorRT→decoderを起動できるようにする。
- 元の画像stampを保持したrelayの受信を起点にsteady clockで測り、過去のheaderと現在のwall timeを減算しない。
- デコード完了の軽量header通知はopt-inとし、CUDA readback後に出版する。画像/Tensorの追加記録は行わない。
- decodedと制御timerによる最初の指令／同じframeの再出版を別集計する。90Hz入力と50Hz制御の件数差を推論落ちと断定しない。
- warmup・曖昧なstamp・不一致を保存して集計から除外し、0件を0msとして埋めない。時系列の点spikeを間引かない。
- 全pipeline購読準備後に再生し、STOP warmup後のAUTOは計測専用namespace内に限定する。
- namespaceとclockをsessionごとに分離する。過去のmode／制御／動的TFを再生しない。bridge・mux・復帰・実センサを起動しない。
- 実車入力までの遅延／閉ループ性能をこの試験で測ったことにしない。センサUSBや撮影は再現せず、relay・観測の負荷を含む。
- bag自然終了後はdrainし、Ctrl-Cでも子processを停止・待機してからCSV／JSON／HTMLを確定する。中断／欠測を成功扱いにしない。
- 初期対応はsingle-head RGB。EVS RAW/Tensor、sectionのpose/head再生は未対応として明示拒否する。

## 指令とIMUの定性的な応答比較

- 解析レポートに操舵指令＋角速度、駆動／制動指令＋加速度を共通のbag記録時間軸で表示する。
- 指令の正規化量とIMUの物理単位は左右の軸に分離し、振幅を同じ量として扱わない。
- IMUのtopic・sensor軸・frame_idを明示する。未校正のTF回転・車体ヨー軸の仮定・重力除去を行わない。
- Imuのcovariance[0]=-1で利用不可とされた角速度／加速度は欠測とし、0の測定として描かない。
- A/Bの手動選択は記録上の時間差を示すだけとし、遅延の自動校正やtimestamp補正を行わない。
- 一定スロットルから応答遅延を同定せず、入力変化がある区間を使う。車体・タイヤ・路面・速度・記録時刻のずれを分離した値と主張しない。


## 操舵とIMUのモデル遅れ推定

- sensor angular_yとsteeringの観測された逆相関を扱えるよう、符号付きゲインGとbiasを含む純遅れモデルを任意操作で適合する。
- gyro_yの既定選択は車体ヨー軸の校正ではない。正規化指令を実舵角と扱わない。
- recorder時間軸の元解像度を使い、候補間で共通のサンプル・同じ区間を比較する。外挿／gap補間／timestamp補正は行わない。
- τ/G/bの選択には先頭70%のみを用い、後半30%で当てはまりを確認する。
- 入力不足・探索端・低い適合／検証成績・平坦／複数解を警告する。誤差近傍を統計的信頼区間とは呼ばない。
- 速度一定と単純な応答モデルは未検証の仮定。τを通信・GPU・サーボのどれか単独の遅延と主張しない。
- 結果には区間・topic／軸・モデル・探索刻み・品質判定・provenanceを残し、UIの条件変更時には古い推定を無効化する。
