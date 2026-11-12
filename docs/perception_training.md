# RCカー検出をStudioで追加学習する

公式YOLOv8sを初期値に、`rc_car`の1クラスを学習する。
COCO80モデルと学習済みモデルは別に管理する。RCカーをsuitcaseと認識するようなクラスのずれを
実走行画像で調整し、RCカーがいない画像でのオレンジダクトなどの誤検出は「対象なし」の負例として扱う。
精度の改善は独立した録画・実画像で確認する。画像上の検出と路面投影の成否は別々に確認する。

## 1. RGBを録画してNotebookへ取り込む

Studioの「Jetson起動」から通常の「データ記録」を選び、RGBを有効にする。
実運用の認識と合わせる場合は640×480を使う。録画開始・停止は既存のrosbag録画ボタンを使う。
「転送」で録画済みbagをStudioを実行しているPCへ取り込む。
録画中のbagは抽出に使わず、停止・ファイルの確定後に取り込む。
画像をFoxgloveへ配信していなくても、bagにRGBを記録してあれば抽出できる。

学習用と検証用は別のbagにする。同じ走行の連続画像をランダムに分けて評価値を上げない。
別走行・違う位置や向き・照明の録画を検証側へ残す。
両側にRCカーあり・なしの場面を含めると、検出漏れと誤検出を確認しやすい。
同じbagのコピーを両側へ登録することも検査で拒否する。

## 2. 「RCカー検出」のプロジェクトと画像抽出

`tools/app/start.sh`でStudioを起動し、「RCカー検出」を開く。
プロジェクト名を付け、bag・RGB topic・学習用/検証用・間引きHz・最大枚数を選んで抽出する。
既定topicは`/realsense/color/image_raw`、2Hz、最大500枚/回、プロジェクト全体は5000枚まで。
RGB/BGR/monoのImageとJPEG/PNGのCompressedImageに対応する。
画像の撮影stampとbagの記録stampは区別して保持する。間引きにはbag記録時刻を使う。
画像サイズは抽出元のまま保存し、bbox座標を変更するresizeは行わない。

## 3. bboxを確認する

RCカー全体を囲うbboxを描き、位置・大きさを調整して保存する。
クラスは`rc_car`固定。オレンジダクト、路面、人だけなどRCカーがいない画像は、
枠を消して「対象なし」と確認して保存する。
未確認の空画像は負例と区別し、学習には使わない。

「bbox候補」を作ると、選んだ重みの検出枠を手動作業の下書きとして利用できる。
公式80クラスモデルではcar以外のクラスで認識された枠も候補にする。
候補は正解annotationではなく、採用・修正・削除してから人が確定する。
ダクトを覆う巨大なperson枠などは採用しない。
全てのRCカーを付ける。RCカーを含む未注釈部分を「背景」として学習させない。
保存時はプロジェクトと画像のrevisionを確認し、他のタブからの編集を黙って上書きしない。

### 確認すべき画像を選ぶ

「データ準備」内の選定補助で、未確認の**train画像**に確認順を付けられる。
既定はYOLO、処理上限500枚、選定100枚。bagごと・時間方向に候補を分散してから計算し、
似た連続画像に枠付けが偏らないようにする。選定はannotationや確認状態を変更しない。
「枠を確認」の「選定順で確認」で優先画像から進み、通常の全画像表示へも戻れる。
順位外・対象なし・val画像も残す。検証用bagは通常の表示で幅広く確認し、選定で分布を変えない。

- **YOLO**: 公式80クラスまたは完了したRCカー学習の重みを使う。クラスを問わないbbox候補、
  信頼度の中間域、見た目の多様性から優先度を作る。YOLO無検出や背景を調べる枠も混ぜ、
  モデルの見落としをそのまま選定から排除しない。優先度はRCカーが写る確率ではない。
  「YOLO詳細」で確認用score下限`review_confidence`（既定0.10）、想定最大検出数
  `expected_max_detections`（既定3）、大きな枠の面積比`large_bbox_area_ratio`（既定0.40）を調整できる。
  下限以上の有効bboxが想定数を**超える**画像、または画像内にclipした最大bboxの面積比が
  設定値**以上**の画像を要確認候補にする。低scoreの枠を大量に数えて要確認にすることはない。
  近接したRCカーや実際の複数物体では正しい検出の場合もあり、誤検出の確定や自動negative化には使わない。
- **DINOv3**: 先にtrainのRCカー枠と対象なしを、それぞれ最低1枚、できれば複数の外観で確認する。
  RCカーのbbox cropを正例、対象なしの画像を負例として、凍結特徴の局所類似度で並べる。
  背景だけの類似に寄りすぎないよう、全体に加えて2×2・3×3の局所cropを使う。
  valのannotationは参照例に使わない。似ている候補を探す機能であり、自動検出の正解ではない。

YOLO選定は10枚ごとに1・3番目を要確認枠、5番目を背景確認枠、10番目を探索枠にし、
通常枠は要確認以外を優先して混ぜる。該当候補がなければ他の候補で補う。
同じbagの近接時刻や似た外観の重複も抑え、要確認画像だけに注釈が偏ることを防ぐ。
候補を採用しない画像も削除せず、正例・負例の確定は前節の手動確認で行う。

DINOv3は既存E2Eと同じ、cleanな`e2e.repos`固定commitのソースと公式ViT-S/16重みを指定する。
ローカルパスをUIへ入力し、未配置なら配置後に実行する。モデルは自動取得しない。
処理は候補500画像まで、正例crop32・負例frame16/局所特徴32までを上限とし、batchに分ける。
元モデルの導入手順は[weights/dinov3](../weights/dinov3/README.md)参照。

少数を確認→再選定→追加確認の順に繰り返せる。annotationだけの更新では以前の順位を継続でき、
追加した正解をスコアへ反映したいときに再計算する。画像の追加や入替では古い選定を無効にする。
重み・設定・参照例・順位・理由をprojectの`selection.json`に保存する。
選定はローカル処理とし、LLMやクラウドへ画像を送らない。

## 4. 学習環境と実行

「学習環境をセットアップ」はStudio実行PCの`.venv/perception`にPython 3.11と固定版の
Ultralytics等を導入する。初回はネット接続が必要。公式YOLOv8sの重みがなければ
固定URLから取得し、SHA256を照合する。既存の重みは書き換えない。
既存環境を使う場合はUIでPythonの絶対パス・重みの絶対パス・deviceを設定する。
`auto`はCUDA→MPS→CPUの順に利用可能なdeviceを選ぶ。GPUを使うならStudioをそのPC上で実行する。
ブラウザを開いているだけでは別PCのGPUで学習しない。

既定は50epoch、batch8、入力サイズ640、学習率0.001、seed0、workers2。
学習用・検証用のそれぞれに、別bag由来の確認済みpositive画像が最低1枚必要。
これは実行可能な最低条件であり、評価の十分さを保証する枚数ではない。
確認済みnegativeも含めてYOLO形式の固定snapshotを作り、未確認画像は除外する。
学習開始後のannotation編集は、そのrunのsnapshotを変更しない。
epochごとのloss、precision、recall、mAP50、mAP50-95を記録する。
少量データの指標だけで実機精度を判断せず、残しておいた録画でもbboxを確認する。

ジョブは既存Studioのログ・中止機構を使い、UIを閉じても管理プロセスが継続する。
途中で失敗・中止したrunは完了モデルとしてexportできない。
checkpointや診断ログを残し、既存の成功モデルは上書きしない。

## 5. kart用ONNXを書き出す

完了した学習からONNXをexportする。固定FP32入力`images=[1,3,480,640]`、
出力`output0=[1,5,6300]`、NMSなし、dynamicなし、opset17とし、kartのraw Detect契約を検査する。
モデル一覧から`opponent.onnx`、出力検査の`report.json`、由来を示す`artifact.json`を取得できる。
モデルの「JetsonへONNX転送」で、上部の接続設定と同じSSH接続先へ送信できる。
「SSHホスト上の保存先」と「Docker内で対応する保存先」を、実際の共有マウントに合わせて指定する。
既定はそれぞれ`/home/kart/workspaces/kart/weights/racing/yolov8s/finetuned`と
`/workspaces/weights/racing/yolov8s/finetuned`。この対応をアプリが自動推測することはない。
既存の転送キューを共用し、3つのモデルファイルと転送manifestを一時領域へ送り、
全payloadのSHA256を照合して新規モデルdirectoryとして公開する。既存モデルを上書きしない。
runtimeの`.ultralytics`・`.matplotlib` cacheは送信対象から除く。

完了後はSSHホスト上のパス、Docker内のパス、buildコマンドを表示する。
**アプリはbuildを実行しない**。TensorRTは実行するJetsonのkartコンテナでbuildし、`opponent`profileを使う。
転送ごとのモデルdirectoryを入力にできるよう、`--model-root`を用意している。

```bash
cd /workspaces
source ros2_ws/install/setup.bash
bash scripts/racing_trt.sh --model opponent --model-root /workspaces/weights/racing/yolov8s/finetuned/<転送後のdirectory名>
```

build出力のengineは、そのdirectory内の`engines/`へ保存する。
標準の`weights/racing/yolov8s/finetuned/`配下なら通常bringupのengine選択にも列挙する。
異なる配置先の場合は、buildが出力したplanの絶対パスをbringupへ指定する。
このモデルの追跡対象はclass0。COCO80モデルのcar=2とは混同しない。
重みの配置・build手順は[認識設定](../ros2_ws/src/kart_bringup/config/racing/README.md)を参照。
学習・exportによって走行中のモデルや車両モードは自動変更しない。

## 保存場所・API・検証

全てStudioのdata root配下に保存する。`perception/`はGit管理対象外。

- `perception/projects/<id>/`: 原画像・手動annotation・未確認の候補・bagとsplitの由来。
- `perception/runs/<id>/`: 固定dataset snapshot・学習要求・metrics・best/last重み・報告。
- `perception/models/<id>/`: 完了したONNX・shape検査・由来。
- `.kart-studio/perception.json`: Python・初期重み・deviceの設定。
- `.kart-studio/perception_tools.json`: 転送先とDINOv3ローカルパス。

GET `/api/perception`、`/api/perception/project?id=...`、`/api/perception/metrics?id=...`、`/api/perception/selection?id=...`。
POST `/api/perception/{create,annotate,settings,tools-settings,setup,extract,prelabel,select,train,export,push}`。
画像は`/perception-image?id=...&frame=...`、完成モデルは`/perception-file?id=...&file=...`。
ファイルの公開対象を限定し、任意のファイルパス・shell commandをHTTPから実行しない。
bag抽出・snapshot・annotationの整合性、サービス/API、canvas座標と編集、学習/export契約を単体検証する。
実データでの学習品質、Jetson TensorRTと実走行の評価は別途必要。

一次資料: [Ultralytics YOLOv8](https://docs.ultralytics.com/models/yolov8/)、
[Detection dataset形式](https://docs.ultralytics.com/datasets/detect/)、
[Train](https://docs.ultralytics.com/modes/train/)、[Export](https://docs.ultralytics.com/modes/export/)。
画像選定の特徴抽出: [DINOv3公式](https://github.com/facebookresearch/dinov3)、
[DINOv3論文](https://arxiv.org/abs/2508.10104)。
