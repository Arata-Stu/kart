# Kart Map Studio の設計方針

## 対象と画面

Notebook側で記録から地図を作成・編集してJetsonへ渡すローカルWebアプリ。
画面は「地図」「転送」「学習」の3つ。E2Eは独立したbackend/frontend moduleで実装する。
Jetsonのbringup、車両操作、録画開始停止は対象に含めない。
学習画面はデータセット作成・学習・ONNX export・オフライン評価・特徴量確認・モデル転送を扱う。

- 高さ付きVSLAM点群を背景に左右境界を編集し、centerline、raceline、customlineを生成する。
- 画面全体を縦に伸ばさず、キャンバスを主役にする。工程ごとに右パネルを切り替える。
- 一覧とログのみ内部スクロール。設定の詳細はダイアログ。1440×900と1280×720で確認する。
- 一つの地図に複数の開路／閉路laneを定義し、地図内で一意のIDにより選択・編集する。
  生成ラインはオフライン参照ライン。オンラインplanner・障害物回避・レーン間の自動接続・信号・競技ルールは対象外。

## 責務

`tools/app` は標準ライブラリHTTPサーバーとビルド不要のES modules。
HTTP、ファイル保存、ジョブ、転送、地図geometryは別モジュールに分ける。
`kart_mapping` はROSを必要とするbag再生、cuVSLAM処理、最終snapshot採取を担当する。
実運用のVSLAM設定は `kart_bringup/config/mapping`。JetPilotへの実行時依存を作らない。

## 座標と成果物

- bagの動的 `/tf` と記録済み自己位置を再生グラフへ混ぜない。カメラ外部姿勢は `/tf_static` を使う。
- 最後の点群を保存し、odom座標なら終了間際まで更新された最新時刻の `map→odom` を一度だけ適用する。
- map座標の点群・SLAM pathには二重変換しない。必要なTFがない場合は失敗とし、単位変換で代用しない。
- 元点群、frame、点群時刻、採用TFと時刻、bag、設定を保存する。高さは変換後map座標のZであり床からの距離ではない。
- 高さフィルタは表示だけ。元点群・座標・HDMap境界を切り詰めない。
- 最終SLAM pathを優先する。odom履歴への最終剛体TF適用はループ閉じ込みによる全履歴最適化と同一ではない。
- `map.json` を正本にし、revision付きで保存する。境界を変更したら生成ラインを無効化する。
- HDMap YAMLとCSVは明示的な出力操作で同じrevisionから生成する。生成方式と制約をmetadataに記録する。
- Racelineは走行可能領域内の曲率低減候補と加減速制約付き速度。最短ラップや実車安全性を保証しない。

## 転送・ジョブ

SSH鍵とknown_hostsを利用し、パスワードや秘密鍵をアプリへ保存しない。
Jetsonのrecordをディレクトリ単位で探索し、選択bagをSCPでローカルへ取得する。
mapの送信もSCPを使い、転送は一時領域で完了してから新規ディレクトリとして公開する。
既存データを上書きしない。失敗・中止した一時領域は成功成果物と区別する。
長時間処理は停止可能なジョブとし、プロセスグループを管理する。同じ資源の競合を拒否する。
サーバーはloopback限定。任意シェルコマンドAPIを設けない。

## 検証境界

geometry・TF・保存競合・SSH引数・HTTPをローカルテストし、ブラウザで編集とレイアウトを確認する。
ROS/GPU環境とJetsonがない場合、cuVSLAMの実処理と実転送は未確認と明示する。

## ラインと地図再利用の拡張（2026-10-07）

- Racelineは簡易方式／helper最小曲率／helper反復最小曲率を切り替える。後者は閉路専用。
  既存の固定済みhelper forkを専用Pythonで実行し、global optimizer本体のrepos依存は追加しない。
  DockerのRaceline依存はNotebook向けamd64だけに導入し、Jetson arm64では専用環境を作成しない。
  判定にはイメージ内のdpkgアーキテクチャを使う。推論・ROSの共通依存とは分けて扱う。
  依存不足や失敗で方式を暗黙に変えない。全方式に境界検査、helperに出力曲率検査を適用する。
- Customlineの区間速度は始点からの距離に対する上限。区間端を点列に挿入し、閉路の始点跨ぎと
  重複区間の最小上限、区間前の減速を含む加減速伝播を扱う。編集とUndoの対象として正本に保存する。
- 位置合わせは別地図のHDMap／各ラインへのSE(2)変換だけ。点群・VSLAM・TFは保持する。
  プレビュー時の元／先全文と変換が一致した場合のみ適用し、旧正本をバックアップする。
- 画面は地図／転送のまま維持。方式選択だけを常設し、区間速度と位置合わせはダイアログへ分離する。

## 地図の削除

地図削除はフォルダ単位で`map/.trash/`へ移動し、GUIから復元できる。
確認で対象と未保存編集の破棄を示す。地図ジョブlockとrevision検査を適用し、同名への復元は拒否する。
元bag・リモートJetsonを削除しない。完全消去と容量解放はこの操作に含めない。

## 複数laneとオフラインライン（2026-10-08）

- 正本とexportのschemaは`kart.hdmap.v2`。`lanes[]`の各要素に不変の識別子としてIDを付け、
  境界・開閉路・Custom編集点・区間速度・生成済みラインをlane単位で保存する。ID変更は新規laneとして扱う。
- 旧単一lane形式は読み込み時に`lane_001`へ変換する。読み込みだけでは元ファイルを書き換えず、保存時にrevisionを進める。
- selectorは編集対象のlane IDを選ぶだけ。走行中のlane選択・オンラインplannerへの指令は送らない。
  今後のselectorが同じIDを参照できるよう、HDMapとCSV参照にIDを保持する。
- lane間の重なりは許可し、独立してラインを生成する。自動接続や通過ルート、回避方向を推定しない。
- 一つのlaneの境界変更は、そのlaneの生成結果のみを無効化する。Custom編集はそのlaneのCustomlineのみ。
- exportは全laneのCenterline生成を要求する。`lanes/<id>/*.csv`とHDMapの`offline_lines`で対応付ける。
  単一lane時だけ従来のルート直下CSVも併記する。複数lane時に暗黙の既定laneを作らない。
- 位置合わせは全laneのIDを保持して変換。転送はv2の編集正本と全laneの同一revision成果物を送信する。

## 同一点群からの地図複製

- VSLAM再実行を不要にするため、保存済み地図を別IDの独立したファイル群として複製する。
- 全lane・生成ライン・snapshot・cloud・cuVSLAMバイナリと履歴を保持する。exportキャッシュは再生成する。
- 新しい正本はrevision 1から開始し、複製元ID・revision・fingerprintを残す。共有ハードリンクは作らない。
- 元／先IDをジョブlockし、revision検査、同名拒否、リンク拒否、一時領域からの公開、中止時の掃除を行う。
- GUIでは未保存編集を元へ保存することを明示し、処理後に未保存編集がなければ複製先を開く。

## 公式offline地図生成への移行（2026-10-08）

地図作成はNVIDIA `create_map_offline.py` の `edex compute_poses` に委譲する。
独自のbag再生・固定drain・save_mapによる生成経路は使わない。
地図作成はStereo rectified入力。既定はVSLAMのみ、明示選択で公式depth・occupancy工程を追加し、占有地図と3D meshを保存する。cuVGL・可視化再生は別工程。
地図生成成功と点群取得成功を分離し、`snapshot_status=pending` の地図を一覧・複製・削除可能にする。
点群未取得の間は編集・生成・export・HDMap転送・位置合わせを拒否する。
後続工程として保存地図を可視化ありで再生し、自己位置合わせと最新TFを確認してsnapshotを取得する予定。

## HDMap ROS配信

`kart_hdmap`はv2正本／exportを読んで全laneのMarkerArrayと選択laneのPathを配信する。
編集・経路計画・走行指令は担当しない。lane IDの表示選択と明示的なreloadだけを提供する。
HDMap座標をそのまま配信し、frame不一致を拒否する。TF配信や最終map→odomの再適用は行わない。
2D HDMapのZは0とし、3D路面推定と混同しない。異なるVSLAM地図間の整合はStudioの位置合わせで行う。
marker更新は専用topic内で古い表示を削除し、未選択／未生成Pathは空配信する。

## 点群取得の成功条件

公式offline地図作成後、元bagを保存地図の作業コピーへ再生し、VSLAM可視化点群を取得する。
LocalizeInMapのservice受理だけでは成功扱いにしない。診断のsaved-map localizedと追跡OKを確認する。
成功前／追跡喪失区間のデータを捨て、診断・軌跡・TFの末尾stamp確認と最新TF適用後にpendingからreadyへ移行する。可視化点群は更新駆動のため末尾stamp一致を要求せず、最終成功区間の最新非空点群を使う。点群と入力末尾の時間差を記録・警告し、全地図の網羅性は保証しない。
取得済みsnapshotの再生成はHDMap座標を変える危険があるため初版では提供しない。
VGLは今後の位置推定機能として対応予定。FoundationStereo low_res（深度）と
ALIKED 424×240（VGL特徴抽出）の軽量化を区別する。地図とONNX・入力shapeを対応付け、
対象GPUで生成したTensorRTを使用する。軽量モデル不在時に標準大型モデルへ自動fallbackしない。

## セット編集・静的禁止領域（2026-10-08）

左右境界の組を中点操作で同時編集し、選択組／全組の幅を設定する。正本は左右XYのまま。
左右の再サンプリング・方向／位相合わせは確認付きの明示操作とし、Undo可能にする。
独立した禁止領域は地図単位の`obstacles`（IDと単純polygon）として保存・export・複製・転送する。
位置合わせは全laneと禁止領域へ同じSE(2)を適用する。領域変更は全生成ラインを無効化する。
生成・export時は閉路終端を含む全線分について内部・接触・車幅半分＋余裕を検査する。
これは静的なオフライン判定であり、自動回避・車体矩形の掃引判定・オンラインplannerは後続。
VGLは後続開発とし、走行時にもJoyからlocalization要求を発行できる設計を目指す。
Joy入力とlocalization実行を別責務にし、要求受付とlocalize成功を区別する。


## E2E workflow

4工程の切替でコンパクトに表示し、ログと中止は既存Jobsへ集約する。
学習・exportはNotebook Pythonの固定CLIを別processで実行し、TorchをUIサーバーへimportしない。
train/validationのbag重複を防ぎ、名前・path・数値・出力modeを検証する。
成果物はステージング内で検証後に公開。同名上書きは不可。
モデル転送はONNX＋metadataだけとし、SHA256照合後に新規directoryへ公開する。
推論node起動、engine生成、走行開始はこのUIから自動実行しない。

## オフライン試走と方向反転（2026-10-10）

試走はライン工程から開くブラウザ内のコンパクトなパネルとする。
まずPure Pursuit＋運動学的自転車モデルで選択laneを1周／終点まで確認する。
ROSノードや実車PIDと等価とは扱わず、実車へ接続しない。
境界・禁止領域の表示と、試走軌跡の衝突判定は区別する（後者は未実装）。
試走設定は実車設定へ自動適用しない。
方向反転はlane単位で左右交換・点列反転・Custom区間速度の距離変換を一括実施する。
古い生成結果は無効化し、速度・曲率・長方形車体の掃引検査を再生成で更新する。
閉路始点の保持とUndoを提供し、表示・CSVの進行方向を一致させる。

Web UIの短い入口はscripts/webui.shとし、Dockerの対話Bashではwebuiコマンドも提供する。
Jetson接続の初期SSHユーザーはkart。保存済み接続設定は自動上書きしない。

SSH認証とknown_hostsの正本はホストの~/.ssh。Dockerは読み取り専用共有し、
Notebookでも既存SSH agentを転送する。接続先検証は有効のまま、初回登録はホストで行う。

地図生成の失敗時は公式ログと中間成果物を`map/.failed/<ID>/`へ保持し、保存先をジョブログに表示する。成功地図として公開しない。ユーザーによる中止は従来どおり一時領域を削除する。

E2Eのオフライン評価はexport済みONNXを対象にrosbagの有効教師付きMANUAL画像で行い、ROS制御topicをpublishしない。学習と同じ前処理とモデルmetadataの出力設定を用いる。UIで予測/教師グラフと誤差指標を提示し、Jetsonへの転送前に確認可能にする。実車安全ゲート・TensorRT速度・閉ループ追従性能の検証とは区別する。

Jetson向け事前TensorRT buildはscripts/e2e_trt.shで行う。通常E2E起動のcache再利用はハードウェア・ソフトウェア指紋とONNX/engineハッシュ・deserialize検証を必須とし、未検証engineを明示falseで使うことも拒否する。

E2E bringupではTensorRT engineを自動生成しない。未build・不適合なら案内して起動を停止する。事前buildはe2e_trt.shへ分離する。オフラインONNX評価はCUDA優先、利用不可時のCPU fallback理由と実際のproviderをログ・結果へ記録する。

bagによる地図確認はVSLAM単独とVGL併用を明示選択する。Map StudioのVSLAM地図をVGL準備済みbundleと混同せず、VSLAM単独にVGL資産を要求しない。

Web UIのVGL準備は選択中の公式VSLAM地図から明示実行し、別bundleへ保存する。元地図とHDMapを変更せず、モデル資産不足を大型モデルへのfallbackで補わない。

地図送信にはcuVSLAMの内容ハッシュで対応付く完成済みVGL bundleを全版同梱する。
HDMapと同じ一時領域で検証・公開し、対応bundleの有無をログへ表示する。
GPU固有モデルengineは地図転送に含めず、実行先で準備したモデルをbringupで選択する。

カメラbagのオフラインlocalization評価はcamera_linkを既定基準とする。車体TFを仮定・生成しない。実車localizationのbase_linkとは明示的に区別する。

## Occupancy gridの編集補助表示

点群のみでのHDMap編集を基本とし、公式dense出力は任意の表示レイヤーとする。
元map座標のbefore_adjustmentラスタのみを既定の対応データとして採用する。
YAMLの原点・解像度・回転と画像の上下方向を明示的に扱い、地面合わせ後の画像を
元mapへ無変換で重ねたり、map←odomを二重適用したりしない。
対応ラスタがなければ表示不可の理由を示し、点群編集は妨げない。
表示ON/OFF・透過率は閲覧設定であり、保存HDMapや生成ライン・禁止領域を変更しない。

既存地図へのdense追加生成はdepthとoccupancyの2段階とする。
元official runを作業コピーし、VSLAM地図・点群・HDMap形状は変更しない。
深度中間成果物は独立して保存し、occupancyの再実行で深度推定を繰り返さない。
成功した出力だけmanifestへ登録し、失敗・中止では既存の成果物を維持する。

3D meshプレビューは公式のmesh_before_adjustment.plyを元map座標で使い、
公式生成コードや元ファイルを書き換えない。座標系を確定できない成果物をidentityで救済しない。
点群の正規化済みTFは再適用しない。2D HDMapの表示zは表示専用で、計測標高と扱わない。
レンダラーは独立モジュールとし、閲覧終了時にネットワーク読込とGPU資源を解放する。

### RGB mesh後処理

RGB着色はtools/mesh_colorに分離し、公式offline処理を変更しない。
現在の地面合わせ前meshと同じrunのカメラposeを使用し、元bagのRGB CameraInfoと静的TFで投影する。
別のodom軌跡や地面合わせ後poseを暗黙に代入しない。
元座標・面構造を保持した別PLYを公開し、観測不能頂点は灰色とする。
UIは着色率を表示する。RGB表示は座標整合性の保証ではない。

RGB着色の既定遮蔽判定はNotebook GPU/EGL深度描画とする。
CPU ray版はCLIで明示指定した場合のみ使用し、EGL失敗・software renderer時の暗黙fallbackは禁止。
GPU描画とCPUのbag読込・色割当・保存の実行境界、および実GPU検証の有無を明記する。

## 学習データセットの保存階層

UIで新規作成するdataset・ONNXモデル・評価は`e2e/{datasets,models,evaluations}/YYYY-MM-DD/HH-MM-SS/<name>/`へ保存する。
作成要求時のサーバーローカル日時を使い、同一日時・同名の上書きを拒否する。
内部IDと学習時の参照はdatasetsからの相対パスとし、日付を跨ぐ同名datasetを区別する。
既存の平置きdatasetは移動せず参照可能にする。一覧は日付でグループ化する。
読み込みは既存のパス境界・symlink拒否を維持し、一時成果物を公開しない。


学習成果物の削除は`e2e/.trash/<token>/data`への移動とし、kind・元の相対ID・削除日時を保存する。
UIから元の場所へ復元でき、既存保存先を上書きしない。計算中の整理操作と使用中成果物の
移動を資源ロックで拒否する。旧形式も扱い、依存成果物の連鎖削除・完全削除は行わない。

学習UIは記録済みRGBと学習済みcheckpointの特徴量をオフライン確認できる。
RGB・共通基底のパッチPCA・ステア入力感度を区別して表示し、attentionや走行性能の証明として扱わない。
確認結果も日付/時刻で保存し、ごみ箱への移動・復元を既存成果物と同じ方法で提供する。
