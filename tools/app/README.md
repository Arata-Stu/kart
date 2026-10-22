# Kart Map Studio

Notebookでbagを受信し、Isaac ROS cuVSLAMの地図・点群からHDMapと走行ラインを作成する。
**地図 / 転送 / 学習**の3画面。Jetsonのbringup・車両操作機能は持たない。
学習画面からデータセット作成・DINOv3学習・ONNX export・オフライン評価・RGB特徴量確認・Jetsonモデル転送を実行できる。
[設計方針](../../docs/policies/ui_app.md)に対象と座標契約を記載。

## 起動

Docker入室直後（/workspaces/ros2_ws）から:

```bash
../scripts/webui.sh
# http://127.0.0.1:8766
```

新しいDockerイメージでは、どのディレクトリからも `webui` だけで起動できます。
既存コンテナでも `source /workspaces/docker/scripts/kart-ros.bash` を一度実行すれば有効です。
引数は従来どおり渡せます（例: `webui --port 8769`）。
リポジトリ直下からは `bash scripts/webui.sh`。旧tools/app/start.shも引き続き利用可能です。

Python 3.10以上。UI本体は標準ライブラリ、ES modules、Canvasのみ。npm/build不要。
Macでも点群編集・簡易ライン生成・SSH/SCP転送を利用できる。helper方式は別途専用Python環境が必要。
VSLAM地図作成はCUDA対応LinuxのkartコンテナでROSをビルドして実行する。

```bash
cd /workspaces/ros2_ws
colcon build --symlink-install --packages-up-to kart_mapping
source install/setup.bash
cd /workspaces
./tools/app/start.sh
```

`start.sh`は存在すれば`/opt/ros/lyrical/setup.bash`とkartのinstallをsourceする。
Pythonを選ぶ場合は`KART_STUDIO_PYTHON=/usr/bin/python3`。
ROS処理は`ros2 run kart_mapping build_map`で別プロセスへ分離する。

設定: `--port 8766`、`--data-root /path/to/data`。
後者未指定ならrepo直下の`record/`・`map/`・`e2e/`・`.kart-studio/`を利用する。
同じdata-rootのサーバー重複起動は拒否する。loopback以外へのbindには対応しない。
画面を使うNotebook／Linuxホストのブラウザからアクセスする。Dockerはhost networkを使う。

## 地図作成・編集

1. 「転送」で録画停止済みbagを受信する。既存の`record/`内のbagも再帰探索する。
2. 「bagから地図を作成」でbagと地図名を選ぶ。NVIDIA公式offline処理でRealSense左右IR（rectified）からStereo地図を生成する。
   詳細でtopic・frame・隔離ROS domainを指定できる。生成直後は「点群未取得」で、以下の編集手順はsnapshotがある地図のみ対象。
   点群取得前に「VGL地図生成」で対応bundleを生成する。続いて「保存地図から点群を取得」で
   元bag、VGL地図、実行GPU用モデルを選ぶ。VGL地図の入力shape・ONNXハッシュとローカルモデルを照合し、対応モデルを初期選択する。
   複数候補は選び直せる。対応モデル不在・元地図不一致・地図整合性不良では開始を無効化する。
   パスとサイズを表示して確認可能にし、実行GPUのengine互換性は起動時に検査する。VGLで初期姿勢を求め、VSLAMのlocalize成功後の点群を保存する。
   地図内容hash・ONNX hash・bag画像サイズの不一致は拒否する。ROS/GPU/実bagでの結合動作は未確認。
3. 作成された地図を開き、高さの下限・上限を調整する。高さはmap座標Zで、床からの距離ではない。
4. 「境界」で左右境界を同じ走行方向に描く。クリック追加、ドラッグ移動、Delete削除。
   右ドラッグ／Alt+ドラッグでパン、ホイールで拡大、全体表示で戻る。Undo/Redoあり。
5. 閉路は最後の点を始点に重ねない。内外の始点が多少ずれていても位相を合わせる。
   開路の左右は始終点が対応するように描く。形状が複雑で中間線が領域を外れる場合は生成を拒否する。
6. 右上のlane selectorで対象IDを選び、「ライン」でCenterline、Raceline、Customlineを生成する。
   Customlineは「点を編集」で描き、DeleteとUndoで調整する。
7. 「HDMap・CSVを書き出す」で同一revisionの成果物を保存する。「転送」からJetsonへ送信できる。

既存のKart `snapshot.json` またはJetPilot `vslam_reference_snapshot.json`を
サーバー上の絶対パスで読み込める。旧snapshotのodom点群には保存された`map_from_frame`が必要。
TFが欠けている場合は読み込みを拒否する。旧形式でTF時刻がない場合はその制限を画面に表示する。
**snapshot単体のインポートはcuVSLAMバイナリ地図を含まない**。
転送一覧は「VSLAM＋HDMap」と「HDMap・点群のみ」を区別する。

高さフィルタは表示だけ。元snapshotは全点保存、ブラウザ表示は最大25万点に均等間引きする。
高さ選別はその表示用点群に対して行うため、細かい構造は原点群で再確認する。
保存前の移動や再読込では破棄確認が出る。境界変更で生成済みラインを無効化し、
Custom制御点だけの変更ではCustomlineだけを無効化する。

## ライン生成の契約

| 設定 | 既定値 | 意味 |
|---|---:|---|
| spacing | 0.15 m | 再サンプリングの目標間隔（最大600点） |
| vehicle_width | 0.19 m | TT-02ボディ付き製品を参考にした暫定車幅 |
| vehicle_length | 0.35 m | ユーザー実測の全長（2026-10-22） |
| rear_axle_to_rear | 0.1065 m | 後輪軸中心から後端。前後等オーバーハングの暫定仮定 |
| margin | 0.05 m | 車体前後左右の余裕。helperの追加余裕はなし |
| max_speed | 3.0 m/s | 速度上限 |
| lateral_accel | 2.5 m/s² | 曲率に応じた横加速度上限 |
| accel | 1.5 m/s² | 加速上限 |
| decel | 2.5 m/s² | 減速上限 |
| curvature_limit | 2.0 1/m | helper方式の曲率上限。簡易方式には適用しない |
| optimizer | local | Raceline方式: local / mincurv / mincurv_iqp |

Centerlineは弧長対応した左右境界の中点。閉路の位相と方向を合わせてから領域内を検証する。
Racelineは画面で簡易方式（local）、最小曲率（mincurv）、反復最小曲率（mincurv_iqp）を選択する。
簡易方式はCenterlineから始めて法線方向に点を動かす局所曲率二乗低減。
点だけでなく隣接線分と境界の距離を検査し、`車幅/2 + margin`を確保する。
Customlineは手描きの点列の頂点と速度区間境界を保持して再サンプリングし、同じ領域・車幅検査をする。
Raceline／Customlineには横加速度制約と前後方向の速度制約を適用。開路の始終点は速度0。
**最短ラップ最適化ではない**。全条件での最適性・実車の操舵応答や制動性能は検証していない。
Centerline単体は幾何学的基準線で、車幅制約はRaceline／Customline生成時に検査する。

## 複数laneをIDで編集する

右パネル上部のselectorで編集するlane IDを選ぶ。「＋」で空のlaneを追加するか、選択laneの境界と
Custom編集点・区間速度をコピーする。コピーでは生成済みラインを引き継がず、改めて生成する。
「−」は選択laneを削除する（最低1つは残す）。追加・削除・編集は「保存」で確定し、Undo/Redoに対応。
IDは英数字で始まる英数字・`_`・`-`の64文字以内。最大64 lanes、大文字小文字だけが違うIDも重複として拒否する。

選択中のlaneを色付きで、他laneの境界を灰色の破線で表示する。切り替えても未保存編集は保持する。
各laneに左右境界、開路／閉路、Custom制御点、区間速度、Centerline/Raceline/Customlineを持つ。
一つのlaneを変更しても別laneの生成結果は無効化しない。IDの変更UIは設けず、作成時に確定する。
例えば`main`と`shortcut`を作って独立に編集・生成できる。境界は引き続き左／右を個別に編集する。
「境界 → セット」で左右の中点をドラッグすると、同じ番号の左右点が同じ量だけ移動する。
クリックで左右点を同時追加し、Deleteで組を削除する。「幅を適用」は選択組だけ、未選択なら全組の幅を変更する。
左右の点数が異なる場合は確認後に等弧長で再配置する。「対応を揃える」で方向・閉路の始点も距離により対応付け直す。
再配置は境界形状を変える場合があるためプレビューを確認する。個別編集への切替、Undo/Redo、保存・再読込に対応。
保存形式は従来の左右境界のまま。幅は左右点間の距離であり、曲線に直交する道路幅を保証しない。

### 走行不可能領域

境界パネルの「走行不可能領域」を開き「＋」で独立したポリゴンを作る。
キャンバスで3点以上を順にクリックし、頂点ドラッグ／Deleteで編集。「−」で領域を削除する。
IDは自動採番、全lane共通のmap座標・メートル単位の静的領域。最大100領域、各3〜100頂点。
面積0、自己交差、重複IDは保存を拒否する。作成途中の3点未満も保存できない。
`map.json`と`hd_map.yaml`の`obstacles: [{id, polygon: [[x,y], ...]}]`に保持する。
領域変更で全laneの生成ラインを無効化。全方式の生成・export時に線分交差、内部、接触、
車幅/2＋margin以内の接近を検査して拒否する。点列の点間をまたぐ細い障害物も検出する。
Centerlineも検査するため、中央を塞ぐ領域があるlaneでは生成できない。通路を分けたlane定義が必要。
回避最適化・自動迂回は行わず、車両の長さ・向きに依存する矩形占有や動的障害物はまだ扱わない。
複製・転送・位置合わせでも領域を保持し、ROSでは赤い境界として表示する。

旧地図は`lane_001`として読み込む。元ファイルは閲覧だけでは変更せず、保存時にv2へ更新する。
このUIで生成する線は**オフライン参照ライン**。画面のselectorは編集対象を選ぶものであり、
オンラインplannerや走行時selectorは今後の実装。障害物回避・lane間の接続・自動ルート生成は行わない。

## 地図の削除・復元

地図を選択し、キャンバス上部の「削除」で確認すると地図フォルダ全体を`map/.trash/`へ移動する。
VSLAMバイナリ・点群・境界・ライン・export・位置合わせbackupをまとめて保持する。
元の`record/`のbagとJetson側のデータには触れない。未保存の編集は破棄される旨を確認画面に表示する。
「ごみ箱・復元」から地図を選ぶと元の名前へ戻せる。同名地図がある場合は上書きせず拒否する。
処理中の地図、確認後にrevisionが変わった地図は削除できない。
ごみ箱への移動なのでディスク容量は解放しない。GUIからの完全消去は未実装。

## Customlineの区間速度

「ライン → Customline → 区間速度」で開始距離・終了距離（m）・速度上限（m/s）を入力する。
基準はCustom制御点列の先頭。紫の○と距離ラベルをキャンバスに表示する。
全体速度上限と区間上限の小さい方を使い、重複区間は最小値を採用する。
閉路は開始 > 終了で始点を跨ぐ。開始 = 終了は拒否。全周指定は0〜表示される全長。
始終点を出力点列へ挿入するため、点間隔より短い区間も反映する。
区間境界では低い側の上限を採用し、前後の加減速制約を全周／全経路へ伝播する。
生成後は速度を橙（低速）〜青（高速）で表示する。

区間は最大40個。`map.json`の`lanes[].custom_speeds`へ`{start,end,speed}`で保存する。
適用後はCustomlineを再生成する。他の生成ラインは無効化しない。Undo/Redoに対応。
制御点編集後も距離指定は固定。全長を外れる区間は保存・生成時に拒否するので調整する。
制御点が必要数未満になった場合は区間設定をクリアする。
速度は目標一定速度ではなく上限であり、曲率制限・加減速によりさらに低くなる。

## Racelineの方式切替と専用環境

Racelineは、そのlaneに**保存・生成済みのCenterline**があれば、その点列を指定点間隔へ再サンプルして初期経路に使う。
領域方式で生成したCenterlineも同じ点列を利用し、別の境界中点列へ置き換えない。
保存済みCenterlineがない場合だけ従来の境界中点を作る。方式の選択変更だけでは保存済みCenterlineは変わらない。
初期経路の出典・生成方式・元点列hash（保存済みCenterlineの場合）は`reference_info`へ保存・exportする。
初期経路での距離不足は「Raceline初期経路」の検査として、補間後の違反とは区別して表示する。
簡易方式は実行可能な初期経路を要求する。helperは領域内の初期点を横移動できるため、
最終結果の余裕を最適化前の参照点へ要求せず、QPと最終検査で車幅・余裕を満たす。
Centerline・境界・車体寸法・指定余裕そのものは変更しない。

Raceline欄には現在の車幅・全長・余裕（cm）を表示する。寸法の「調整」から、
車幅・全長・後輪軸から後端までの距離・余裕・曲率上限・点間隔を変更できる。
余裕込みの幅／全長と選択中の方式を確認し、「適用してRaceline生成」でそのまま実行する。
「設定のみ適用」は生成を開始しない。速度等は同じ画面の詳細を開いて調整する。
既存の「車幅・速度などの生成設定」も共用し、生成結果には実行時の設定を保存する。
後輪軸から後端までの距離が全長以上の場合は、生成前に入力エラーを表示する。

既定は従来の簡易方式。helperの2方式は**閉路専用**。
`docker/python/raceline`で固定した`trajectory_planning_helpers` forkとquadprogを利用する。
`global_racetrajectory_optimization`のcheckoutは不要。
Notebook向けLinux x86_64（amd64）Dockerにはビルド時に依存を導入し、`/opt/env/bin/python`を自動選択する。
Jetson（arm64）イメージにはRaceline環境を導入しない。別環境では次のように指定する:

```bash
KART_RACELINE_PYTHON=/path/to/raceline-env/bin/python ./tools/app/start.sh
```

未指定で`/opt/env/bin/python`がなければサーバーと同じPythonを使う。
専用環境はNumPy/SciPy/Matplotlib/quadprogと、依存定義のコミット固定helperを導入しておく。
ROS用Pythonを切り替える必要はない。依存不足・最適化失敗時はエラーとし、簡易方式へ自動fallbackしない。
計算は別プロセスで実行し、停止要求と600秒の上限に対応する。
QPの`constraints are inconsistent`は依存不足と区別し、曲率上限・最小旋回半径・車幅・余裕と
境界／設定の確認案内を表示する。これは近似QPの失敗であり、実経路の不存在を証明するものではない。
法線方向の合計幅が車幅＋両側marginを下回る場合は、
該当参照点の座標・幅・必要幅を表示して最適化前に拒否する。設定の自動緩和はしない。

helper入力は周期スプラインで平滑化（残差二乗和の基準は点数×0.05² m²）し、等弧長で再配置する。
法線方向の実境界までの距離から左右幅を求め、車幅と片側marginを一度だけ最適化に適用する。
marginを片側の合計余裕として使用し、最初は追加余裕なしで解く（既定の必要幅は0.29 m）。
補間後は最大0.05 m間隔の出力と長方形車体の掃引を検査する。
境界距離・車体検査で違反した場合は、不足量＋1 mmを基準にQPの幅制約を厳しくして再計算する。
繰り返し違反する場合は補正量を少なくとも倍増し、初回を含め最大8回・全体600秒までとする。
IQPが入力幅を書き換えるため、各回とも元の参照経路・幅から解き直す。
指定marginや曲率上限は緩めない。最初からQPが不成立、補正でQP不成立、または上限まで検査違反なら失敗とする。
補正中のQP不成立時も直前の違反位置を表示し、失敗候補を保存しない。
実際の片側補正量を`optimizer_info.interpolation_reserve_m`、再計算前の失敗位置と補正量を
`optimizer_info.clearance_refinements`へ記録する。一律の片側5 cm追加は行わない。
最終的に元の境界との線分距離・自己交差・曲率上限（数値許容1%）を検査し、違反結果は保存しない。
この検査は出力点列に対するもので、実車追従や連続曲線全体の厳密保証ではない。
方式・設定・helperの版と取得元・前処理条件を生成結果およびexport metadataに保存する。
最小曲率QPは近似であり、すべてのコースで簡易方式より低曲率になることを保証しない。

## 別のVSLAM地図へのHDMap位置合わせ

1. 新しい点群／VSLAM地図を作成または開き、「点群 → 別の地図のHDMapを位置合わせ」を選ぶ。
2. 元地図とXY移動（m）、反時計回りヨー回転（度）を入力する。
3. 「点群上でプレビュー」で水色の境界・ラインを点群と比較する。「調整」で再入力できる。
4. 「この位置で保存」で適用先の境界・ラインを置き換える。

変換は`p_target = R(yaw) p_source + [x,y]`。回転中心は元地図原点。
境界・Custom制御点・生成済み全ラインのXYとheadingを変換する。
距離station・幅・曲率・速度・加速度・Custom区間指定は保持する。最適化は再実行しない。
元地図と適用先のsnapshot、点群、cuVSLAMバイナリ、TFは変更しない。

元／先の全文fingerprintと変換をプレビューに結び、変更後の適用は拒否する。
両地図のジョブロックを取得する。適用先の旧`map.json`（読み込み時にv2へ正規化）を`registration-backups/`へ保存してから
新revisionの編集正本をatomic replaceする。履歴は`map.json.registration`に記録する。
復元する場合はStudioを停止し、そこに記録されたbackupを同じ地図の`map.json`へ戻す。
適用後の新しいexportは別revisionに生成し、古いexportを再利用しない。
縮尺変更・鏡映・局所歪み補正・自動位置推定は行わない。コース全体で一致するか確認する。

## 成果物

```text
map/<name>/
  map.json             # 編集正本・revision・生成方式/設定
  snapshot.json        # 元点群・SLAM path・採用TF・設定・bag出典
  cloud.json           # UI向けmap座標点群（最大25万点）
  cuvslam_map/         # cuVSLAM保存データ（bagから作成した場合）
  job.json            # 作成時のジョブ入力
  exports/revision-N-v2/
    hd_map.yaml        # JSON表記のYAML 1.2。全lane ID、境界、centerline、CSV参照
    lanes/<lane_id>/centerline.csv  # x_m,y_m,width_right_m,width_left_m
    lanes/<lane_id>/raceline.csv    # s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2
    lanes/<lane_id>/customline.csv  # 同上（生成済みのものだけ）
    metadata.json      # revision、fingerprint、生成設定
```

kartのHDMap形式は`kart.hdmap.v2`。`lanes[].id`と`lanes[].offline_lines`のCSV相対パスで対応付ける。
`line_role=offline_reference`を明示する。単一lane時のみルート直下にもCSVを併記する。
全laneのCenterline生成が済んでいないとexportを拒否し、対象IDを表示する。
旧v1のexportは保持し、v2の出力先を分ける。地図publisherはkart_hdmapが対応。オンラインplannerとの接続は今後の対象。
JetPilotの競技ルールや複数Laneネットワーク形式を暗黙に互換扱いしない。
保存は楽観的revision検査、JSONはatomic replace、exportはrevision単位の新規ディレクトリ。
過去のexportは残す。各laneの最新状態でCenterlineを再生成してから書き出す。

## Jetson転送

接続設定は`.kart-studio/connection.json`にユーザー・ホスト・ポート・rootのみ保存する。
秘密鍵・パスワードは保存しない。`ssh`と`scp`がPATHに必要。
Jetson側はPython 3、SSHサーバー、SFTPサブシステムが必要。rootは既存ディレクトリを指定する。
英数字・`/_.-`のリモートパスを扱い、空白・日本語名・IPv6は初版では未対応。

- 「探索する」はSSHで選択ディレクトリの子だけを列挙する。フォルダを辿ってbagを選ぶ。
- SCP pullは`.pull-*`へ受信。転送前後の送信元ファイル件数・サイズ・更新時刻を比較し、
  受信側の件数・サイズも確認する。録画中のbagでは使わない。正常完了後に指定名で公開。
- pushは現在revisionのexportとmap/snapshot/cloud、存在すればcuVSLAM地図をbundle化。
  `.kart-upload-*`へ送信し、件数・サイズ一致後に`<name>_rN_<random>`へrenameする。
  既存地図を上書きしない。失敗したremote一時領域はログに場所を残し、必要に応じて手動整理。
- known_hosts検証を無効化しない。初回SSH鍵の準備・接続確認は利用者が通常の端末で行う。
- ブラウザを開くだけではSSH接続しない。bringup、Docker、録画要求、車両操作APIはない。

## モジュール

| モジュール | 責務 |
|---|---|
| backend `server` | HTTP/Host/Origin/JSON検証とルーティング |
| `maps` / `lanes` / `exports` / `storage` / `snapshots` | lane管理、移行、保存、revision、ID別export、座標の正規化 |
| `geometry` / `lines` / `speed_sections` | 境界検査・各ライン・区間速度profile |
| `optimizer` / `optimizer_worker` | helper専用環境の実行・最適化adapter |
| `registration` | HDMap位置合わせのプレビュー・保存 |
| `jobs` | 排他、履歴、ログ、停止 |
| `transfers` / `remote_agent` | SSH探索、SCP、一時領域と公開 |
| `mapping` | UIリクエストの検査、ROS worker起動 |
| frontend `canvas` | 描画、パン/ズーム、点編集 |
| `maps` / `transfers` / `dialogs` / `jobs` | 各画面の操作 |
| `api` / `app` | 状態、通信、画面切替 |
| ROS `kart_mapping` | bag検査、cuVSLAM、最終TFと点群保存 |

## 検証

```bash
PYTHONPATH=tools/app:ros2_ws/src/kart_mapping python3 -m unittest discover -s tools/app/tests -v
PYTHONPATH=ros2_ws/src/kart_mapping python3 -m unittest discover -s ros2_ws/src/kart_mapping/test -v
```

ローカル検証内容は[実装・検証記録](../../docs/ui_app_validation.md)。
ROS/CUDAでの地図作成、実JetsonとのSSH/SCP、実コースのライン評価はローカルテストと区別する。

合成点群でUIだけを試す場合（実測データではない）:

```bash
python3 tools/app/make_demo.py --data-root /tmp/kart-studio-demo
./tools/app/start.sh --data-root /tmp/kart-studio-demo
```

## 同じVSLAM地図から別バージョンを作る

地図を開き、上部の「複製」→名前（例: `course_v2`）→「保存して複製」。
未保存の編集は元地図へ保存してから複製し、完了後に新しい地図を開く。
点群・軌跡・最終TFを含むsnapshot、cuVSLAMバイナリ（存在する場合）、全laneの境界・区間速度・生成ラインを引き継ぐ。
元地図と独立したファイルなので、一方の編集・削除はもう一方へ影響しない。
新しい地図はrevision 1から開始し、元地図IDとrevisionを記録する。書き出しキャッシュは複製せず再生成する。
複製は中止可能なファイルコピーのみで、ROS・GPU処理やbag再生は行わない。コピー分のディスク容量は必要。

既定の地図生成は公式 `create_map_offline.py --steps_to_run edex compute_poses`。
平坦路向けにworkflowの`ground_constraint=true`を既定とし、`--override cuvslam.cfg_planar=true`を渡す。
保存済み地図には遡及適用されないため、poses・occupancy・meshは再生成する。
任意でdepth・occupancyを追加し、占有地図と3Dメッシュを生成できる（末尾の説明を参照）。
VSLAMバイナリ・軌跡・ログを保存し、「VSLAM作成済み・点群未取得」と表示する。
HDMap用点群は作成しない。点群未取得の地図は複製・削除可能だが、HDMap編集・生成・export・送信は不可。
後続の可視化あり再生・点群取得は下記の別操作で実行する。従来の0.5倍再生／5秒drainは生成工程から廃止。
既存のsnapshot付き地図は引き続き編集できる。

## 保存地図からの点群取得

点群未取得の地図を開き、「保存地図から点群を取得」を押して地図生成に使ったbagを選択する。
CUDA対応LinuxのROS環境でのみ開始可能。点群未取得の状態表示はMacでも確認できる。
地図の作業コピーを使って可視化ありVSLAMを1倍速で再生し、原点hintからの自己位置合わせ成功を確認する。
末尾の点群・軌跡・TFを保存すると自動的に編集可能になる。取得済み地図への再取得は行わない。
失敗・中止時はpendingのままで、元のVSLAM地図を変更しない。ログ・停止は通常のジョブ画面を使用。
任意地点からの位置合わせはこの工程の対象外。VGL地図の追加生成は「VGL地図を追加生成」から実行する。


### TT-02の暫定寸法

新規ライン生成の車幅既定値を0.19mへ変更した。余裕0.05mは維持し、
現行の線分距離判定は半車幅＋余裕=0.145mとなる。
[公式仕様と暫定外形](../../docs/policies/vehicle_geometry.md)参照。
既存地図に保存したline.settingsは書き換えない。古い地図を開くと旧車幅が復元されるため、
生成設定で車幅0.19を指定し、各ラインを再生成する。
全長は2026-10-22のユーザー実測値0.35mを使用する。後輪軸→後端は未再計測のため、
暫定値0.1065mを維持する。保存済みの全長0.47mは自動変更しないため、生成設定で0.35mへ変更して再生成する。


### 長方形の基本接触判定

全ラインの生成結果とexport時に、後輪軸基準の幅/全長/後端距離を使用して再検査する。
車体前端はvehicle_length−rear_axle_to_rear=0.2435m、左右は±0.095m。
marginは前後左右へ加える。UI「ライン生成設定」から3寸法を変更できる。
ライン位置は後輪軸の参照軌跡として扱い、姿勢はRace/Customのprofile yaw、Centerは点列の接線から計算する。

点間は位置を線形補間、姿勢を最短角で補間する。両端長方形の凸包と回転偏差上限による
保守的な掃引領域で、点間の薄い障害物や回転する角も検査する。接触も不可とする。
閉路は最後→最初の区間を含む。コース境界と禁止領域を確認し、違反時は保存/exportを拒否する。
保守近似のため実際には接触しない軌跡を拒否する可能性はある。

開路の左右境界は出入口を塞ぐ壁とは扱わず、端で接線方向へ全長＋余裕分だけ延長して
長方形検査する。出入口に実際の壁がある場合は独立禁止領域として定義する。
最適化内部の候補探索は従来の車幅ベース、最終候補の採否に長方形検査を使う。
拒否時に長方形に合わせた回避経路を自動探索する機能はない。

生成設定にcollision_model=rear_axle_rectangle_v1を記録する。
古いモデルのラインを含む地図はexport前に全ラインの再生成が必要。
既存exportを自動上書きせず、生成revisionを更新する。
これはオフラインの名目軌跡の検査。実車の追従誤差、動的障害物、停止距離を含む
走行中のfootprint監視は別途必要で、Pure Pursuitが障害物を回避するわけではない。


## E2E学習画面

上部「学習」に4工程を分離する。一度に1工程だけ表示し、右側に対応する成果物一覧を置く。
実行中ジョブは画面を移動しても継続し、下部のログ・履歴からログ確認・中止できる。
ページ再読込・UIサーバーのCtrl-C後もジョブは独立プロセスで継続する。同じdata-rootでUIを起動するとPID・ログ・実行状態を再取得する。
OS再起動・コンテナ停止で終了した学習のcheckpointからの自動再開は行わない。

1. **学習環境**: Notebook上のPython絶対パス、DINOv3固定commitのソース、公式重み、deviceを指定する。
   学習/ONNXの依存は[python_ws/e2e](../../python_ws/e2e/README.md)。ROSをsourceした環境でUIを起動し、
   指定Pythonからrclpy・rosbag2_py・kart_interfacesを読める必要がある（同じROS Pythonでsystem-site-packages venv）。
   UI本体のPythonと学習Pythonは分離可能。`KART_E2E_PYTHON`は未保存時の既定値。
2. **データセット**: 受信済みbag・名前・時計・許容時間差を指定。topicは折りたたみ設定。
   MANUALのみ、ブレーキ/後退を除外。詳細は[kart_e2e](../../ros2_ws/src/kart_e2e/README.md)。
   データ作成中にbagのファイルサイズ・更新時刻が変化した場合は公開しない。
3. **学習**: 学習用・検証用データセットをそれぞれ複数選択できる。同じbag・同じデータセットを両方に指定可能（自動分割なし）。
   同じ画像を共有する場合、validation指標は未知データへの汎化評価として扱わない。
   2出力／steer-only、epoch、batch、学習率、encoder凍結／finetuneを設定する。
   ライブラリに完了epochと最終epochのvalidation MSEを表示。実際のbest.ptは最良epochのもの。
4. **ONNX export**: 実験のbest.ptを選びモデル名を指定。
   checker・ORT数値比較・metadata/hash検証に成功した成果物だけ表示する。
5. **Jetson転送**: 共通の接続設定を使用。学習環境の「Jetsonモデル保存先」は存在するディレクトリを指定。
   model.onnxとmetadata.jsonだけを一時領域へSCPし、SHA256照合後に一意名のフォルダへ公開する。
   checkpoint・Python・TensorRT engineは転送せず、Jetsonでengineを生成する。推論launchや走行は開始しない。
   転送後のリモートパスはジョブ結果に保存される。中断時のリモート`.kart-upload-*`は未公開で残ることがある。

データセットはdata-root下の`e2e/datasets/YYYY-MM-DD/HH-MM-SS/<name>/`へ保存する。
日時はUIサーバーのローカル時計で作成を開始した時刻（秒まで）。異なる日時なら同名も作成でき、
同一日時・同名の上書きは拒否する。学習用・検証用の選択欄は日付でグループ化し、時刻・名前・サンプル数を表示する。
従来の`e2e/datasets/<name>/`も移動せず一覧・学習に使用できる。内部IDはdatasetsからの相対パスで、
同じ名前でも日時を含めて区別する。処理中・失敗した一時成果物は一覧に出さない。
ONNXモデルと評価も`e2e/{models,evaluations}/YYYY-MM-DD/HH-MM-SS/<name>/`へ保存する。
学習実験・TensorRT計測は従来どおり`e2e/{runs,engines}/<name>`、環境設定は`.kart-studio/e2e.json`。
この自動階層はUIからの作成に適用する。直接実行する抽出CLIのoutput引数は明示指定のまま。
作業中は同一ファイルシステムの非表示一時directoryを使い、成功時のみrenameして公開する。
同名成果物の上書きはしない。既存のCLI成果物の任意インポート・成果物削除は今回の対象外。
学習・export・VGLなどは、入力の読取りを共有し、異なる出力なら並列実行できる。
同じ成果物への書込み、処理中の入力の編集・削除、同じROS domainを使う処理は競合として拒否する。
VSLAM等ほかのGPU処理との排他はないため、GPUメモリに余裕のある実行計画にする。

API: GET `/api/e2e`、POST `/api/e2e/{settings,dataset,train,export,push}`。
HTTPは既存のloopback・同一origin制限を維持する。リクエストから任意shell文字列を受け取らない。
実行commandは固定module＋検証済みargv。SSH鍵・known_hostsの準備は既存転送画面と同じ。

検証（2026-10-09）: app単体テスト67件通過。UIから公式DINOv3実装＋ランダム初期重み・
合成画像で1epoch学習→ONNX exportを実行し、成果物一覧への反映まで確認した。
ORT最大出力差は約5.22e-8。これは配線の確認であり学習精度の評価ではない。
転送のファイル限定・ハッシュ検証・破損時の公開拒否はローカル代替で検証済み。
実rosbagからの抽出、Notebook CUDA学習、実JetsonへのSCPとTensorRT推論は未確認。

実験名は予測対象に応じて`dinov3-control-v1`または`dinov3-steer-v1`を提案する。
ONNXモデル名は選択した実験の名前を基に`<name>-v1, -v2, ...`を提案する。
各一覧の最大versionの次を使い、手入力した名前は更新時も保持する。同名上書きは禁止のまま。
公式重みの配置先は`weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth`。
保存済みの空の重み設定にもこの既定パスを補う。別名・別の場所は「学習環境」で変更可能。
標準画像424×240は抽出時に保持し、学習・推論時に212×120へ縮小、正規化後224×128へ中央paddingする。

### Jetson IPプリセット

共通定義は`config/jetson_hosts.json`。接続設定のIPプリセットから
`10.42.0.1`（既定）、`192.168.11.190`、`192.168.55.1`を選択できる。
選択で変更するのはhostのみ。ユーザー・SSHポート・record/mapルートは保持し、
「設定を保存」でbag探索、SCP pull、map/ONNX転送へ共通適用する。
既存の保存済み接続先は維持する。未設定時のhostは共通既定値を使用する。
SSHユーザー既定はkart、record/mapは/home/kart/workspaces/kart/{record,map}。
学習環境設定のモデル転送先も/home/kart/workspaces/kart/modelsを既定とする。
既存の保存済み設定は自動で上書きしない。接続設定の「初期値に戻す（保存）」でユーザー・IP・ポート・record/mapルートを既定値へ戻し、即座に保存できる。SSH鍵やknown_hosts、学習環境設定には影響しない。
任意IP・hostnameの手入力も可能。SSHユーザー未設定なら転送前に設定画面を開く。
Foxglove起動スクリプトも同じ定義を使い、`--preset notebook|lan|usb`で選択する。
UIでの選択は保存済みSSH接続、CLIでの選択はその実行だけに適用し、互いに上書きしない。

### ラインの方向反転・試走

「3 ライン」上部の「進行方向を反転」は選択lane全体に作用します。
左右境界を交換して点列を逆順にし、Customも反転します。閉路は各点列の始点を保持し、
開路は始終点を交換します。Customの区間速度は距離を`全長−旧距離`へ変換して
同じ物理区間に残します。Undo可能です。反転後は各生成ボタンで再生成してください。
生成済み結果は無効化し、向き・曲率・加減速・車体掃引検査を再計算します。
保存とCSV出力にも新しい方向が反映されます。地図上の矢印で方向を確認できます。

「試走 ▶」は選択laneの生成済みCenterline / Raceline / Customlineを使うブラウザ内プレビューです。
Pure Pursuitと運動学的自転車モデルを20ms刻みで計算し、後輪軸基準の車体長方形、
実行軌跡、速度、舵角、追従誤差と最大誤差を表示します。再生・一時停止・リセットが可能です。
閉路は1周、開路は終点で終了します。2m超の追従逸脱または300秒で停止します。
交差部で別の線分へ飛ばないように、位置探索を進行方向の近傍に制限します。

| 試走設定 | 既定値 | 意味 |
|---|---:|---|
| 速度上限 | 1m/s | Centerは一定目標、Race/Customは生成profileとの小さい方を速度目標にする |
| 先読み | 0.8m | Pure Pursuitの目標点までのライン沿い距離 |
| 軸距 | 0.257m | 自転車モデルのホイールベース |
| 最大舵角 | 26度 | 試走用の仮定。実車設定は書き換えない |

加速1.5m/s²・減速2.5m/s²の簡易速度追従を使用します。開路始点のゼロ速度による
停止継続を避けるため、最初の5cm区間だけ5cm地点の速度を参照します。
境界・禁止領域は表示のみで、試走軌跡に対する衝突判定・回避は行いません。
ROS controllerの完全再現、タイヤ・モータ・PID・推定誤差の再現ではありません。
シミュレータから実車への指令送信や設定保存はありません。
モデルは`simulation_model.js`、表示・操作は`simulation.js`、方向変換は`direction.js`へ分離しています。

追加検証: `node tools/app/tests/test_simulation.mjs`（ES Modulesを自動判別するNode 22以降）。
反転の往復・速度区間・左右方向の円周試走・開路終端を確認します。

### E2E実行時設定の保存

学習画面の固定throttle（既定0）・throttle上限（既定0.2）は正規化値で、
0 ≤ 固定 ≤ 上限 ≤ 1。checkpointとONNX metadataへ引き継ぎ、
bringup TUIで選択したモデルのsteer_only/steer_throttleに応じて自動適用する。
既存のruntime情報がないモデルはCLIで設定を明示し再exportする。


### DockerのSSH設定共有

dev.shはJetPilotと同じく、起動したホストユーザーの~/.sshを/home/admin/.sshへ
読み取り専用でmountする。既存の秘密鍵・config・known_hostsを再利用し、
コンテナ再作成で消えない。鍵をimageやリポジトリへコピーしない。
SSH agentが起動していればNotebookでもソケットを渡す（aarch64はCLI側の既存転送）。

初回の接続先登録・鍵認証設定はホストで一度行う。UIのIPと同じ宛先を使い、
fingerprintを確認して登録する。コンテナ側からknown_hostsへの追記はできない。
ホストでssh-add済みのagentを使う場合は、そのシェルからdev.shを起動する。

この変更はイメージ再ビルド不要。既存コンテナにはmountを追加できないため、
Web UI等を終了してホストでdocker stop kart_dev（EVSはkart_evs_dev）後、dev.shを実行する。
コンテナ内だけに保存したSSH設定は停止前に必要に応じてホストへ移す。

Jetsonのrecord探索はフォルダをツリーとして展開・折りたたみできる。親・兄弟フォルダを残して現在のフォルダと選択bagを強調し、上部のパス各階層から親へ戻れる。接続設定の変更時にはツリーと選択をリセットする。

「ログ・履歴」で処理を選び、「ログをコピー」で表示中のログをコピーできる。表示は末尾約64KBが対象で、ログが空の場合はコピーできない。

HDMap編集ではカーソル位置への追加結果を点線でプレビューする。laneセット編集は左右境界・幅・追加区間を表示し、2点目による最初の境界の向き変更も実際の追加処理と同じ計算を使う。左右個別・Custom・障害物は次の線分と閉路への接続を表示する。既存点付近は移動ガイドに切り替わり、プレビュー自体は地図やUndo履歴を変更しない。

### HDMapのリセット・waypoint区間

地図上部の「HDMapリセット」は全lane・障害物・生成ライン・区間速度を空にして、現在のlane IDの空laneを1つ残す。点群とVSLAM地図は保持する。確認後も保存前の編集状態で、Undo可能。「保存」で確定する。以前書き出したexportは自動更新されないため、ラインを作り直して再exportする。

Customの「区間速度」は番号付きwaypoint図と開始・終了WPの選択欄で指定する。開始欄を選んで図をクリックすると終了選択へ進み、選択区間が紫で表示される。閉路は末尾→先頭を跨ぐ区間と「WP 1（周回末尾）」に対応する。既存の距離指定は勝手に丸めず保持する。内部保存は従来の距離形式で、Custom点列を編集した後は区間を選び直す。

Raceline初期値はJetPilot `tools/app/frontend/app.js` と `python_ws/map_tools/generate_raceline.py` の `race-stacks` presetを参照。共通項目は点間隔0.10m、曲率上限1.0/m、余裕0.05m、最高速度3.0m/s、横加速度2.5m/s²、加速1.5m/s²、減速2.5m/s²。kartの車体幅0.19m・後輪軸位置は暫定値を保持し、全長はユーザー実測の0.35mへ変更（2026-10-22）。JetPilotの最低速度0.8m/sは停止・制約と競合するため導入しない。helperの内部再サンプリング・平滑化はアルゴリズムが異なるため同一値を意味しない。保存済みラインの設定は維持する。

学習開始時にDINOv3ソースが見つからない場合は、Notebookコンテナの`/workspaces`で`vcs import . < e2e.repos`を実行する。`.pth`の配置だけではソースは導入されない。UI「学習環境」でソースを`/workspaces/python_ws/dinov3`、重みを`/workspaces/weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth`に設定する。環境変数の追加は不要。

### ONNXのrosbag評価

学習→ONNX export→「4 rosbag評価」→Jetson転送の順で操作できる。モデル・bag・評価名を選び開始する。画像/教師操作/モードtopic・時計・許容差は「1 データセット」の設定を共用する。評価履歴をクリックすると教師と指令予測の時系列グラフ、MAE/RMSE/P95を表示する。全フレームCSVとJSONは`e2e/evaluations/<日付>/<時刻>/<評価名>/`。モデルとbagを処理資源として固定し、bag内容の変更や評価失敗時には結果を公開しない。転送に合格閾値は設けず、評価結果を人が確認する。評価はCUDA優先のONNX RuntimeでのMANUAL画像・教師比較で、閉ループ走行やJetson上TensorRTの動作を保証しない。

学習環境のPythonは用途別に指定する。`python`は学習・データ抽出用、`inference_python`はONNX export・rosbag評価用で既定`/opt/inference/bin/python`。Dockerの既存ONNX/ONNX Runtime環境を利用し、システム側のPyTorchはsystem-site-packages経由で参照する。独自venvだけにPyTorchを入れた場合はexport用Pythonからもtorchをimportできる環境を指定する。既存設定の学習Pythonは保持する。

オフライン評価はONNXサイズ（MiB）、実行providerと推論の平均・中央値・P95を表示する。10回のwarmupと前処理は計測から除外する。既存の評価結果は当時の統計を表示し、新統計には再評価が必要。「NotebookでTensorRT build・速度計測」を展開し、ONNXを選んでビルドすると、別保存したNotebook用engineのサイズと合成入力ベンチマークを結果一覧から確認できる。ROS/実走行/Jetson性能の評価とは異なる。操作にはNVIDIA GPUのあるx86_64 kart Dockerが必要。

ONNX評価はCUDAExecutionProviderを優先し、利用不可・初期化失敗は理由をログに出してCPUで実行する。report.jsonにはprovider/providers/cuda_fallback_reasonを保存する。CUDA利用時のsession.run時間はCPU入力からの転送・出力の読み戻しを含む。Linux x86_64のDocker依存はonnxruntime-gpu 1.23.2（CUDA 12/cuDNN同梱依存）、それ以外はCPU版。更新にはDocker再buildが必要。旧評価結果のproviderは変わらないため再評価する。

## Web UIからVGL地図を追加生成

地図のJetson送信では、`map/vgl/*/vgl_profile.json`を探索し、送信するcuVSLAMの
`.mdb`全ファイルのSHA-256が一致する完成済みbundleを自動同梱する。
名前や生成元の絶対パスでは照合しないため、同じ元地図のHDMapコピーにも対応する。
対応するv1/v2等はすべて送り、ログへ名前を表示する。対応bundleがなければログで明示して従来の地図送信を続行する。
送信先は`map/<地図名>_r<revision>_<識別子>/vgl/<bundle名>/`で、bringupの再帰探索から選択できる。
cuVSLAM/cuVGL地図・VGL設定・profileのみを同梱し、GPU固有のTensorRTモデルは送らない。
同梱時に元地図・設定ハッシュ、画像shape、必須VGL成果物を検査し、不整合時は送信を止める。
転送後はHDMapとVGLをまとめてサイズ・件数照合してから公開する。モデルとのONNX照合は既存のbringup検証で行う。
反映にはNotebook側のコード更新とUIサーバー再起動が必要。既存の送信済み地図は変更せず、UIから再送信する。

既存地図を開き「VGL地図を追加生成」を選択する。モデル候補は入力解像度とフルパスで選択する。
424×240・480×270・640×360・640×480・848×480・1280×720などのモデルを選ぶと、幅・高さが自動確定し、確認専用になる。
モデルはmodelsとmap以下を探索し、manifestの入力shapeとONNXハッシュを照合する。
既定424×240の候補がない場合は未選択とし、他サイズを暗黙に選択しない。
生成ボタン直前にモデルのフルパスと画像幅・高さを表示する。
既知サイズはUIから独立指定せず、サーバーもモデルmetadataから決定する。不一致を明示送信した要求は拒否する。
「その他・パスを手動指定」とmanifestなしの旧モデルのみ手動サイズ指定を残す。
保存名は地図名-vgl-vNを既定とする。実行GPUでbuild済みのengineが必要で、download/buildは自動実行しない。公式offline処理の保存画像・posesがない場合は生成できない理由を表示する。

`POST /api/vgl`はinstalled `ros2 run kart_bringup prepare_vgl_map`をジョブ実行し、検証済みbundleを`map/vgl/<保存名>`へ公開する。既存VSLAM地図・HDMapは変更しない。ログ・中止は既存ジョブ画面を使い、失敗・中止時の一時成果物は公開しない。成功後はbringupで「VSLAM＋VGL」を選び生成bundleを指定する。実走行GPUがNotebookと異なる場合は実走行GPU用モデルengineを別途準備する。UIサーバー再起動とkart_bringupのbuild/sourceが必要。

VGL用モデルがない場合は[kart VGLモデル生成](../vgl/README.md)の`scripts/vgl_model.sh`で
424×240のONNXを再exportし、このGPU用のengineをbuildする。
`models/vgl/424x240/runtime_models`が完成するとVGL追加生成画面の候補へ入る。

SCP転送中はジョブ欄へ転送済み/全体MiB、進捗%、開始からの平均MiB/sを表示する。
約2秒ごとに受信先のpayloadサイズを測り、pushは一時領域のサイズだけをSSHで照会する。
これはネットワークの瞬間速度ではなく、接続時間を含む平均。100%後に既存の整合性確認を行い、
確認完了までは成功扱いにしない。ログにも履歴を残す。短い転送では開始と終了だけ表示される。

laneセット描画は前後の区間の二等分線を使い、曲がり角で左右のoffset境界を接続する。
鋭角の突出量は半幅の2倍まで制限し、直前点から1cm未満の追加を拒否する。
ガイドと確定操作は同一の計算を使用する。「全体を整形」は指定した区間幅を全体へ適用し、
閉路の最初/最後の接続も再計算する（Undo可、既存の区間別幅は指定幅で置き換わる）。
急な折返し・幅より狭いカーブ・交差する中心列を自動的に走行可能な形へ修正する機能ではない。

### Centerline方式の比較（実験機能）

「3 ライン」のCenterline方式から選択する。既定は従来方式で、既存地図の再生成は自動では行わない。

| 方式 | 設定値 | 処理・対応範囲 |
|---|---|---|
| 従来・境界中点 | `centerline_method=paired`（既定） | 既存の境界対応点の中点。閉路・開路 |
| 領域・等距離線（実験／閉路） | `centerline_method=region` | 左右境界間を格子化し、左右それぞれへの距離が等しい線を抽出。現在は閉路専用 |

新方式の入力は**描いた左右境界**。公式occupancy画像や点群から障害物を自動認識する機能ではなく、
FoundationStereo／占有地図の事前生成は不要。独立した左右の頂点数・始点の対応付けにも依存しない。
左右それぞれの元線分への最短距離場をNumPyで計算し、走行可能マスク内の距離差ゼロ等高線を追跡する。
閉路が一つに定まる場合だけ採用し、方向を左境界に合わせて等間隔化する。骨格をDFS順で単純に点列化しない。

`region_resolution=0.02` mが既定。「車幅・速度などの生成設定」で0.005〜0.25 mへ変更可能。
格子は100万セル、出力は600点が上限。上限超過時は黙って粗くせず、解像度／点間隔の変更を案内する。
分岐・複数閉路・接続切れ・開路・NumPy不足では失敗し、従来方式へ自動fallbackしない。
新方式だけNumPyが必要（kart Dockerの既存python3-numpyで利用可）。従来方式にNumPyは不要。

再サンプル後の左右距離差が格子幅の2倍を超える、または`abs(d_left-d_right)/(d_left+d_right)>0.10`なら拒否する。
さらに従来と同じ境界・長方形車体・静的禁止領域の検査を実施する。境界・車体寸法・余裕を自動変更しない。
生成methodは`region_equal_distance_v1`、格子幅・最大／平均距離差・相対距離差等は`region_info`へ保存する。
生成済みCenterの「領域」表示にマウスを置くと、格子幅と最大距離差を確認できる。

生成成功時はそのlaneのCenterを更新するため、**新旧を残して比較する場合は先に地図を複製する**。
失敗・中止時は既存の線を保持する。生成済みCenterlineはRacelineの初期経路に使用する。
Centerの方式を変えて生成した後は、Racelineも明示的に再生成する。Customlineは変更しない。
反映はUIサーバー再起動とブラウザ再読込。今回の追加にROS packageの再buildは不要。

検証: 新旧既定互換、異なる頂点数・方向の閉路、凹形の合成コース、保存・export、失敗時の元線保持、
実際のStudioでのジョブ生成と表示。画像の概略トレースは任意スケールの合成データであり、元mapや実寸ではない。
実地図・実車追従・走行可能性の実測検証は未実施。
関連: [参照notebook](https://github.com/CL2-UWaterloo/Raceline-Optimization/blob/9290c5d503462e46f7e3e9033002e7ddf165ba7b/map_converter.ipynb)。
今回の実装は二値領域と距離場を利用する発想を参考にし、骨格化・DFSのコードは移植していない。

### 凹形コースのcenterlineと失敗位置

左右境界の全長割合による対応で生成線がコース外へ出る場合は、
走行順序を保つ局所的な対応付けを試す。境界間の断面がコースを横切らないことを確認し、
再サンプル・小さな平滑化後も境界検査と長方形車体の検査を行う。
境界形状、車幅、車長、余裕を自動変更して検査を回避することはない。
この対応付けで全てのコースに解が得られる保証はない。

生成失敗logには区間番号だけでなく、区間両端のXY座標と具体的な理由を出す。
線の境界検査では境界までの距離・必要距離を表示する。
車体検査は長方形車体が外へ出た／境界へ接触した理由を区別する。
job状態には`diagnostic`を保存する。生成した同じmap/lane/revisionを表示している場合、
失敗区間を赤、該当境界を黄、車体の検査範囲を赤で地図に重ねる。
編集・再生成・地図再読込みで表示を消す。失敗した生成線は保存・公開しない。

## 占有地図・3Dメッシュ（任意、2026-10-18）

「bagからVSLAM地図を作成」で「VSLAM＋占有地図＋3Dメッシュ」を選ぶ。
workflow.jsonのdense_reconstruction=falseが既定、fs_model_resはlow_res（high_resも選択可能）。
FoundationStereo用の公式engineは事前準備が必要。公式depth→occupancyを追加実行するため処理時間とGPUメモリが増える。
既存地図の変更ではなく新規地図として作成する。既存地図と比較する場合は同じbagで別名を使う。
完成後、地図toolbarの「占有地図・3Dメッシュ」でPNGをプレビューし、PLYと座標系関連ファイルをダウンロードできる。
PLYは外部のPLY対応3Dビューアで確認する。ブラウザ内3D回転表示は未実装。
mesh.plyは地面合わせ後、mesh_before_adjustment.plyは地面合わせ前。HDMapへ自動重ね合わせしない。
実体はmap/<地図名>/official/<run>/。頂点・面が空のmeshは生成成功にしない。
反映にはkart_mappingのbuild/sourceとUI再起動。nvblox_rosを含まない既存イメージはDocker再buildが必要。

### Occupancy grid補助レイヤー

地図を開くと編集キャンバス下端に「占有地図」チェックと不透明度スライダが表示される。
既定はOFF。不透明度45%。点群・軌跡・HDMapの下へ描画し、点群の高さフィルタとは独立する。
黒は占有、白は空き、灰は不明という公式出力の配色をそのまま表示する。
表示は編集の参考用であり、lane・禁止領域の自動変更や衝突判定は行わない。

`GET /api/occupancy-overlay?id=...`は、公式の
`occupancy_map_before_adjustment.yaml/.png`だけを対象とする。
地面合わせ前の元map座標のラスタへYAMLのresolution・origin（x,y,yaw）を適用し、
画像上端からの行方向を反転して描画する。点群に適用済みのmap←odom変換は再適用しない。
地面合わせ後のoccupancy_map.pngへの暗黙fallbackは行わない。
成果物なし・読み込み失敗でも点群編集は継続可能。地図切替時には旧画像を消し、遅れて届く旧応答を破棄する。
PyYAMLを使用（Dockerのpython3-yaml）。複製地図はコピーされたdense成果物を利用する。

検証: `test_occupancy.py`、`node tools/app/tests/test_occupancy.mjs`。
実データとの重なり精度は利用する地図で確認する必要がある。

### 既存地図への工程別追加生成

「占有地図・3Dメッシュ」を開き、`1. 深度推定` → 完了後に同画面を開いて
`2. 占有地図・メッシュ生成`を実行する。VSLAMと点群取得・VGLは従来の別操作を使う。
左右画像とframes_meta.jsonがない地図（点群だけをimportした地図など）では追加生成できない。
深度生成済みなら工程2だけ実行できる。解像度選択は深度推定にのみ適用する。
未保存編集は先に保存し、処理中は対象mapへの書込みとROS domainを既存ジョブ機構で排他する。GPU全体の一律排他は行わない。

`GET /api/dense-steps`は工程の可否を返し、`POST /api/dense-step`はid/revision/step/resolutionを受け取る。
公式runを`dense_runs/<uuid>`へコピーし、VSLAM工程を呼ばずdepthまたはoccupancyだけを実行する。
元のofficial_outputは保持し、depth_output/dense_outputをmapping_result.jsonとmap.json.mappingへ登録する。
コピーには画像・中間成果物の分だけ追加ディスク容量が必要。旧成果物は保持する。
失敗・キャンセル時は未登録の作業コピーを削除する。成功時のみrevisionを進める。
深度を再生成した際は旧occupancy/meshを残し、「旧深度の成果物」と表示する。
新しいoccupancy工程が成功すると新成果物へ切り替える。独立したmesh-only工程はない。
テスト: test_dense_steps.py（公式処理はmock、GPUでの追加工程は未検証）。

### ジョブの保持・並列実行・停止と時間表示

ジョブ一覧には管理PIDと実行PIDを表示する。選択ログとログパネルの開閉はページ再読込後も復元する。
`.kart-studio/jobs/`に状態JSON・入力request JSON・ログ・worker lease・停止要求を保存する。
各ジョブの独立supervisorが子プロセス群を管理し、UIの終了時は切り離すだけで停止しない。
PIDに加えてプロセスの生成情報を照合し、PID再利用による別プロセスへの停止を防ぐ。
同じdata-root・同じOS／コンテナでUIを再起動すると、実行中と完了済みのジョブを再取得する。
異常終了でsupervisorだけ失われた子プロセスは「管理外」として資源を保持する。
生成情報を確認できないプロセスへの停止は拒否する。旧形式でPID情報のない実行中履歴は「中断」とし、自動再実行しない。
コンテナ停止・OS再起動後に処理そのものを継続／再開する機能ではない。

画面下部の「処理を停止」は、同じ欄に表示されている最新の実行中ジョブを対象にする。
複数処理から選ぶ場合は「ログ・履歴」を開き、各行または選択ログの停止ボタンを使う。
停止要求後は「停止要求中」を表示し、実際の終了までリソースの排他を維持する。
子プロセス群へSIGINTを送り、応答しない場合は既存の8秒後SIGTERM・さらに3秒後SIGKILLで停止する。
Python内の作業は各workerのキャンセル確認地点で停止する。公開済み成果物は取り消さない。

経過時間は独立workerのmonotonic clockで計測し、約1.5秒ごとのUI更新で表示する。
成功・失敗・中止時はfinishedとelapsed_sを履歴JSONへ保存し、以後時間を増やさない。
旧履歴やサーバー異常終了で終了時間が不明なものは「時間不明」と表示する。
テスト: test_jobs.py（実子プロセス停止・時間保存）、test_job_time.mjs（時間表示）。

工程別生成ダイアログは、深度と占有地図・メッシュを別カードに表示する。
未生成・実行中・生成済み・旧深度で生成済みを文字と配色で区別し、深度完了後は工程2を案内する。
生成済み工程のボタンは「再生成…」となり、実行直前に最新状態を取得して再実行の確認を行う。
深度の確認では重い処理を繰り返すことと、工程2のみで済む場合があることを表示する。

### Mesh・点群・HDMapの3D重ね合わせ

地図を選び「1 点群」→「3Dで重ね合わせ」を開く。公式処理は変更せず、
`mesh_before_adjustment.ply`を元のmap座標（m、Z上）で表示する。
地面合わせ後のmesh.plyへ暗黙fallbackせず、生成元がnvidia.create_map_offlineであることと
HDMapのframe=mapを確認する。`GET /api/mesh-overlay`がファイルと座標契約を返す。
点群は既にmapへ正規化済みのcloud.jsonを利用し、map←odomを二重適用しない。
点群には現在の高さフィルタを適用する。HDMapは2Dなので既定z=0の平面として表示し、
表示高さを変更可能にする（地図を保存・変更しない）。未保存編集も開いた時点の内容を表示する。
全laneの左右境界・生成ライン・禁止領域を表示する。

左ドラッグ回転・右ドラッグ移動・ホイール拡大、全体表示・上から、各レイヤーのON/OFFと
mesh不透明度に対応する。閉じると読込を中断しGPU資源を解放する。
Three.js 0.180.0と公式PLYLoader/OrbitControlsをMITライセンス付きでvendor配下へ同梱し、
実行時CDNは利用しない。WebGL2が必要。ブラウザ表示上限は256 MiB、500万頂点・1000万面。
大きな地図はダウンロードして外部ビューアで確認する。RGB着色・ICP・自動位置補正は行わない。

検証: test_mesh_overlay.py、test_mesh_layers.mjs（ASCII/両endian binary PLY、XY/Z保持）。
合成地図でブラウザのmesh/点群/HDMap重ね合わせと真上表示を確認。
実地図におけるVSLAM再localize誤差・形状推定誤差は別途確認が必要。

WebGL2初期化が失敗した場合はCanvas 2DによるCPU互換表示へ自動切替する。
回転・移動・拡大・レイヤー切替・表示高さは維持するが、最大約12,000面・15,000点へ
表示のみ間引く。面は奥行き順の簡易描画で、点群・HDMapは手前に重ねるため正確な遮蔽判定ではない。
元PLY・点群・HDMapは変更しない。UIには互換表示と表示数・精密評価不可の注意を明示する。
テスト: test_mesh_canvas.mjs（CPU投影・描画・レイヤー・解放、描画contextはmock）。

### meshのRGB着色

「占有地図・3Dメッシュ」の工程3で、元bagのRGBを使って既存meshへ頂点色を追加する。
元bag欄は空欄なら生成時のパス。移動済みならrecord rootからの相対パスを指定。
完了後の3D重ね表示は着色meshを自動選択し、着色率を表示する。
点群は従来の紫色、未着色mesh頂点は灰色。CPU互換表示も面平均色に対応。
APIはPOST /api/mesh-color。既存job管理の停止・時間表示を利用する。
公式出力とHDMapは変更せず、color_runs配下へ別成果物を保存する。
Notebook用の追加依存・時刻／TF／遮蔽判定・既知の限界は
[RGB着色のREADME](../mesh_color/README.md)を参照。

RGB着色はGPU/EGL深度描画を既定とする。GPU名と画像ごとの処理時間をログへ出す。
GPUが利用できない場合は停止し、長時間かかるCPU版へ自動fallbackしない。
Notebookでの依存追加と描画確認コマンドはRGB着色READMEのGPU節を参照。

学習画面の選択欄は16px、ラベルは14px。データセットの複数選択欄は148pxの高さと
行余白を確保し、rosbag・データセット一覧は名前16px／補足13pxで表示する。

データセット名は選択したrosbagの名前から自動候補を入力する。bagを変更すると候補も追従するが、
手入力した名前は保持する。英数字・`_`・`-`以外は`_`へ整え、先頭英数字・64文字以内にする。
変換後に名前が残らない場合は`dataset`を候補にする。日付・時刻の保存階層は別途自動付与する。

学習のEpochはUI・API未指定時ともに10が既定。抽出後に直接使うtrain CLIも同じ既定値とする。


### 学習成果物の整理・ごみ箱

学習画面の各一覧の「ごみ箱へ」で、データセット・学習実験・ONNXモデル・評価結果を
フォルダ単位で`e2e/.trash/<token>/data/`へ移動する。「整理・ごみ箱」では5種類の成果物
（上記＋TensorRT計測）を切り替えて整理でき、ごみ箱から元の相対パスへ復元できる。
旧平置き成果物も同じ操作を使える。新しいモデル・評価は日付階層を内部IDに含め、
日付別の同名モデルの選択・評価・転送を区別する。命名候補にはパスではなく末尾の名前を使う。

UIから開始した学習・export・評価・engine buildの実行中は整理・復元を拒否する。転送中のモデルや
作成中の対象も資源ロックで保護する。復元先に同じパスがある場合は上書きせず拒否する。
移動後に空になった日付・時刻フォルダは除去し、復元時に作り直す。
元bagや依存する別の成果物を一緒に移動することはない。移動した学習データ等への参照は
履歴に残るため、再利用する場合はその成果物も復元する。
ごみ箱はアプリ固有で、自動消去・完全削除のUIは設けない（移動だけではディスク容量は減らない）。
API: GET `/api/e2e/trash`、POST `/api/e2e/trash`（kind/id）、POST `/api/e2e/restore`（token）。
ローカルfixtureで移動・復元・競合・パス拒否と日付IDのexport/評価/転送を検証する。


### RGB特徴量を確認する

「学習 → 特徴量を確認」で学習済み実験とRGBデータセットを選び、可視化を実行する。
確認枚数は既定12（1〜64）、全区間から均等抽出する。完了後に右の結果を開くと、
RGB、パッチ特徴PCA、ステア出力の入力感度を並べ、フレームを切り替えられる。
PCAは特徴の違い、赤い入力感度は局所勾配であり、走行性能そのものではない。
学習環境のPython・device・DINOv3ソース設定を使用し、best.ptの実際の重みを読み込む。
結果は`e2e/features/YYYY-MM-DD/HH-MM-SS/<name>`へ保存し、ごみ箱・復元に対応する。
20ch EVSにも対応。ONNX/TensorRT実行時の特徴量は対象外。

データセット作成・rosbag評価のbag選択は「bagを階層から選択」でNotebookのrecord配下をツリー表示する。
転送画面と同様にフォルダを展開し、▣のbagを選択する。選択後は相対パスを表示し、
同名bagも親フォルダで区別する。データセット名候補は選択bag名から引き続き自動入力する。

学習の「全体をfine-tuning」はDINOv3 backboneと制御headを同時に更新する。
標準学習率はhead学習`1e-4`、全体fine-tuning`1e-5`（手動変更可）、epochは両方100。
学習名候補に`-finetune`が付き、履歴にも全体学習を表示する。

### 学習後の損失カーブ

学習画面で処理が完了すると、学習・検証MSEのepoch別カーブを自動表示する。
別画面やダイアログを操作中は自動で割り込まず、学習済み実験を選択して確認できる。
既存実験も`metrics.jsonl`があれば再学習せず表示できる。最良epoch（best.pt）、
最終損失、最良からの変化、全epochの数値を確認し、ONNX exportへ移動できる。
グラフは平滑化せず、記録済みMSEをそのまま表示する。


凍結学習の「特徴量キャッシュを使用」は既定ON、DataLoader workersは既定8。
初回にencoder出力を`e2e/.feature_cache`へ保存し、以後はheadだけを学習する。
同じ画像/counts内容・重み・前処理等なら別実験でも再利用できる。キャッシュOFFでは従来どおり入力から計算する。
fine-tuning時はキャッシュ欄を無効化し、全体学習に切り替える。workersはUIで0〜64に変更できる。
キャッシュ作成・再利用は処理ログ、実効設定は各実験のrun.jsonに記録する。

新規RGBデータセットはPNG圧縮を省くため、圧縮なしuint8 RGBの`.npy`へ保存する。
元解像度・画素値は保持する。既存PNGも学習・特徴量可視化・ONNX評価で引き続き使用できる。
CLIの互換PNG出力は`--image-format png`。UIの新規抽出・評価時の一時抽出はNPYが既定。


### ViT-Tinyの特徴蒸留と制御学習

学習画面の「RGBモデル」でDINOv3 ViT-Tiny/16を選択する。
上の学習/検証データで「第1段階：特徴蒸留」を実行し、教師ViT-SのCLS特徴へTinyを近づける。
教師はローカル公式重み、またはViT-Sの学習済み実験を選択できる。
教師は凍結、Tinyと学習用192→384射影を更新する。既定lossはMSE+0.5×cosine距離。

完了後に「Tiny encoder」を選び、通常の学習条件で「学習を開始」すると第2段階の制御head学習になる。
Tinyは192→128→controlのheadを持ち、蒸留用射影を実行モデルに含めない。
通常はencoderを凍結し、必要なら全体fine-tuningも使用できる。
蒸留lossと制御MSEのカーブは区別して表示する。Tiny encoderの保存先は
`e2e/encoders/YYYY-MM-DD/HH-MM-SS/<name>`。整理・ごみ箱から復元できる。
以降のONNX export、評価、転送、推論は同じ画面と入出力を使う。Tiny蒸留は現在RGB専用。

### EVS 20chのGUI操作

「データセット」の入力種別を「EVS · 20ch」に切り替え、rosbagとNotebook内の
同じディレクトリのRAWを自動検出する。RAWが1件・timingのstream/epochが1組なら
manifest指定は不要。複数候補・再接続時は詳細設定でRAW manifest絶対パスを指定する。manifestは`kart.evs_raw_manifest.v1`形式で、
RAWとstream/epochの対応を記載する（形式はkart_e2e README参照）。時刻基準は
EVS選択時にheaderへ切り替わり、sample strideは既定10。固定収集bagは固定AUTOのみを採用する。
RAW抽出を実行する「EVS RAW抽出用Python」にはMetavision/OpenEBのRAW readerが必要。
NumPy 1.26.4 / h5py 3.11.0の専用venvを指定し、学習用Pythonとは分離する。
この専用venvはamd64 Notebook用で、Jetsonの記録・推論イメージには作成しない。

「学習環境」のEVS encoder PT/JSONの既定パスはrepo内
`weights/evs/event_encoder.pt`と`weights/evs/event_encoder.json`。
利用者がEventStateで生成した対応するファイルを配置する。自動ダウンロードはしない。
「学習」の入力種別もEVSへ切り替えるとEVSデータセットだけが選択でき、
`train --modality evs`で制御head学習を実行する。epoch=100、workers=8、特徴量キャッシュONが既定。
全体fine-tuningにも対応し、この場合は特徴量キャッシュを使用しない。RGB Tiny蒸留はEVSに適用しない。

学習後の損失カーブ、ONNX export、転送、整理・ごみ箱は共通。
EVSのオフライン評価はモデル選択後に作成済みEVSデータセットを選択する。
RGBは従来どおりbagを直接評価する。特徴量可視化はRGB/20ch EVSに対応。
GUIの入力切替とジョブ生成・成果物公開をローカル検証済み。
実RAW・学習済み20ch重み・GPU/TensorRT・実車での一連の実行は別途検証する。

EVS抽出時にh5pyで`numpy.dtype size changed`が出る場合は、EVS RAW抽出用Pythonの
NumPyとapt版h5pyのABI不整合。kart_e2e READMEの「RAW readerのNumPy / h5py ABIエラー」
を参照。RAWパスやmanifestの変更では解決しない。

EVSのRAW順序乱れ許容幅は既定5000µs（0..100000、0は厳格）。
窓終端＋許容幅まで先読みし、元時刻のまま安定整列する。時間補間・イベント削除は行わない。
上限を超える逆行や負時刻は停止する。学習済みencoderの20ch定義は変更しない。

EVS抽出後の入力変更検査では、RAWに対応するSDK生成`*.raw.tmp_index`だけを除外する。
RAW/rosbag本体の変更監視は維持し、通常転送のファイル署名は索引も含む従来のまま。

EVS特徴量可視化は「特徴量を確認」でEVS実験を選び、同じ入力種別のデータセットを選択する。
正極性赤/負極性青の入力表示、パッチPCA、ステア感度に加え、正負各10binの20ch一覧を表示する。
入力表示のlog1pは可視化専用で学習・推論入力には使わない。RAW SDK不要、学習用Pythonで実行する。

制御学習の損失は既定Smooth L1（β=0.1）、MSEも選択可能。
スケジューラは既定Plateau。通常モデルは検証loss、Multi-headはhead平均の検証MSEの停滞でLRを下げる
（factor=0.5、patience=2、min LR=1e-7）。patience=2は3回の非改善で次epochのLRを低下する。
学習lossと比較用MSEは両方保存し、best.ptの選定は検証MSEを維持する。
損失表示はMSE/学習lossを切り替え、epoch表で使用LR/次epoch LRを確認できる。
既存のMSE履歴も表示可能。これらはRGB Small/Tiny・EVSの制御学習に適用し、Tiny特徴蒸留には適用しない。

### RGB baseline比較

RGBモデルにViT-B（768次元）と小規模CNN（192次元）を追加。
「初期化」でscratchを選ぶと既定100 epoch、全体学習、特徴量キャッシュ無効。
CNNはscratch固定。制御学習は初期化方式にかかわらず既定100 epoch。
ViT-B重みは「学習環境」の専用欄（既定repo内`weights/dinov3/dinov3_vitb16.pth`）へ設定する。
scratchは重み不要。CNNはDINOソースも不要。同じデータ分割・損失・seedを選んで比較する。
seedの既定42、scratch学習率の既定1e-4。epochとLRは上書き可能。
実験一覧にarchitectureと初期化を表示し、損失表示・export・評価・可視化・整理は共通操作。
入力前処理は既存RGBと共通。CNNの構成はkart_e2e READMEを参照。

制御学習の「学習精度」は既定AMP自動。CUDAではnative BF16優先、非対応ならFP16＋GradScaler、
CPUはFP32になる。FP32/BF16/FP16の明示指定も可能で、非対応モードはエラーにする。
実効精度はジョブログ・run.json・checkpoint・実験一覧へ保存する。
loss/比較指標・保存重みはFP32。初回特徴量キャッシュはFP32を維持する。
Tiny特徴蒸留の第1段階は対象外。変更は新しい学習ジョブから適用し、実行中ジョブは変更しない。

TensorRT build操作は「ONNX export」画面にまとめている。
「NotebookでTensorRT build・速度計測」を開き、buildするONNXとFP32/FP16を選択する。
既定FP32（TF32無効）、FP16は内部演算のFP16を許可。どちらもFP32入出力を検証する。
学習AMPとは独立した選択で、精度を成果物名・report・履歴・結果表示に残す。
Notebook engineはJetsonへ転用せず、Jetson側の`e2e_trt.sh`でモデル→FP32/FP16の順に選択して再buildする。

### INT8校正＋FP32比較

ONNX exportのTensorRT build精度にINT8を追加。元のFP32モデルと同じ入力種別の
校正用・比較評価用datasetを選ぶ。既定は各256件・seed42・warmup10回・3反復。
実行すると対称Q/DQ INT8校正、FP32/INT8の両engine build、同一実入力のnative TensorRT比較を行う。
ステア/スロットルのMAE・RMSE・P95とFP32との差、転送込み/GPU演算の中央値・P95、速度比、サイズを表示する。
INT8の浮動小数点層はFP16を許可し、FP32比較元はTF32を無効にする。入出力tensorはFP32。
校正・評価が同じ入力パスを含む場合はその旨を表示する。CPUによるGPU速度の代用はしない。
GPUのあるkart Dockerが必要。校正済みモデルと比較結果は日付・時刻別のmodels配下へ保存し、
比較成功時に結果を自動表示する。元モデルを残し、ライブラリから整理・削除可能。
JetsonへはQ/DQ ONNXとmetadataを転送し、`e2e_trt.sh`でINT8として再buildする。
詳細・CLIは[kart_e2e README](../../ros2_ws/src/kart_e2e/README.md)を参照。

### 制御学習の速度診断

通常のRGB/20ch EVS学習で診断値を計測し、実験内の`timing.jsonl`へ保存する。
「学習速度の診断ログ」でCUDA計測batch数（既定20、0でEventのみOFF）と
詳細JSONLの記録間隔（既定50batch）を変更可能。初回batchとphase終了時にも記録する。
DataLoader待ち、転送、encoder/head、loss、逆伝播、optimizer、指標集計を分離する。
CUDA Eventの時間とhostのdispatch/同期待ちは別に記録し、VRAM peakとsamples/sも保存する。
cache作成/再利用と入力ハッシュ、best.pt保存時間も記録する。
学習中は下部「ログ・履歴」で参照・コピーできる。次回開始の学習から適用され、Tiny特徴蒸留は対象外。
詳細と数値の読み方は[kart_e2e README](../../ros2_ws/src/kart_e2e/README.md)を参照。

### RAM全量読込を既定にする

学習画面の「学習データを最初にすべてRAMへ読み込む」は既定ON。
RGB/EVSの前処理済み入力、または凍結encoderの特徴量を最初にRAMへ読み込み、epochごとのI/Oを省く。
Tiny第1段階の蒸留も対応。RAM学習のworkerは実効0にし、全量データのworkerごとの複製を防ぐ。
worker設定（既定8）は初回特徴量抽出と、RAM OFFの逐次読み込みで使用する。
容量が大きければRAMチェックを外して開始する。必要量が利用可能RAM/container残量の80%を超える場合は
切替方法を表示して停止し、暗黙fallbackしない。読込準備時間とbyte数もログへ記録する。
CLIは`--ram-preload` / `--no-ram-preload`。詳細はkart_e2e READMEを参照。

### 学習ログの進捗表示

「ログ・履歴」の上段に最新の学習進捗（epoch、train/validation、bar、loss/MSE、samples/s、ETA）を
固定表示し、更新ごとに履歴へ積み上げない。epoch完了時は固定欄を消し、結果要約を1行残す。
詳細CUDA/stage計測は実験内のtiming.jsonlへ保持する。
「詳細計測ログの間隔」はJSONLの記録頻度で、画面更新は最大0.5秒ごと。
警告・例外は通常ログに残し、コピーは画面の履歴＋最新進捗。元のjob logも保持する。

## HDMap・自己位置を見ながら現場調整

地図画面のキャンバス上部に「実車調整」を追加した。接続先は既存「転送」の
IPプリセット・SSHユーザー・portを共用する。新しい接続設定や車両操作UIはない。

1. 地図一式を既存の「転送」でJetsonへ送り、同じ地図で自己位置推定を起動する。
2. Jetsonで`bash scripts/bringup.sh` → `drive`を選び、通常の地図・モデル選択後に
   「Map Studio 動的調整」を有効にする。調整用地図のパスは選択済み地図から自動設定され、YAML編集は不要。
   起動例は[kart_bringup README](../../ros2_ws/src/kart_bringup/README.md#map-studioからの現場調整)。
   JetsonのSSH先ホストから127.0.0.1:8782へ到達できる必要があり、Dockerはhost networkingを使う。
3. Notebookで同じ地図を開き、「自己位置を接続」。確定した位置・向き・直近1,200点の軌跡を橙で表示する。
4. 既存の境界／Custom waypoint／区間速度を編集し、ライン生成して保存する。
5. Center／Race／Customを選び「保存済み編集をプレビュー」。青が候補、白が実車の適用済みライン。
   Centerのみパネルの一定速度を使用し、Race/Customは既存エディターで生成した速度profileを使用する。
6. STOP・1秒以上の停車後に「停止中に適用」。Jetsonの適用版を確認し、既存操作系でAUTOへ戻す。
   問題があればSTOP後に「直前の版へ」で戻せる。

地図一致はcuvslam_mapの内容（なければsnapshot.json）で確認する。geometry編集では一致は変わらない。
資産hashは接続中に再計算しない。自己位置推定用地図を置換した場合はStudio・HDMapノードを再起動する。
約1 Hzの通信は位置・状態とrevisionだけ。HDMap全体は初回・適用版変更時のみ取得し、
点群・画像の常時転送やJetson側のライン最適化は行わない。
適用データは3 MiB・生成点20,000点まで。通信失敗時に適用を自動再送しない。
未保存編集・地図不一致・古い候補・適用元版の競合・STOP/速度情報不足は拒否する。
画面切断は車両を停止しない。停止・再開は既存の操作系で行う。
編集の正本はNotebookのmap.json。Jetsonへの適用はメモリ上のみで、再起動時は元の起動地図へ戻る。

検証コマンド:

```bash
PYTHONPATH=tools/app:ros2_ws/src/kart_mapping python3 -m unittest discover -s tools/app/tests -p test_tuning.py -v
PYTHONPATH=ros2_ws/src/kart_hdmap python3 -m unittest discover -s ros2_ws/src/kart_hdmap/test -v
node --test tools/app/tests/test_tuning.mjs
```

ROS結合・Jetsonの負荷・実車動作の確認は別途必要。


### sectionとPure Pursuit参照点

現在のsection分けはCustomlineの「区間速度」。開始／終了waypointと速度上限を
最大40区間設定でき、閉路の始点跨ぎ・重複区間の低い上限・区間前の減速に対応する。
名前付きsection・section別lookaheadは未実装。区間変更後はCustomlineを再生成して保存する。

「実車調整」の自己位置接続中、実際のPure Pursuit参照点を黄色い点、制御に使った車両位置からの
結線で表示する。パネルには設定lookahead（最近傍経路投影点からの経路長）と、車両から参照点までの
直線距離をm単位で表示する。参照点をブラウザで再計算せず、Jetsonの制御出力を読む。
無効通知・0.5秒以上古い計測や受信はbridgeで除外し、画面切断・通信停止時も消去する。
Foxgloveでは3Dパネルの`/control/tracking_markers`で同じ参照点を表示できる。


### Jetsonパラメータの動的調整

地図 → 実車調整 → 自己位置を接続 → 「パラメータ調整」で開く。
対象を選ぶとJetsonの実ノードから現在値・read_only属性・型を取得し、変更可能な項目だけ入力できる。

| 対象 | 動的に調整できる項目 |
| --- | --- |
| Pure Pursuit | lookahead、目標速度上限、経路からの許容距離、終点許容距離、目標減速度 |
| 参照ライン・Center速度 | Center / Race / Custom、Centerの一定速度 |
| 速度PID | kp / ki / kd / kff、積分上限、微分filter、前進／ブレーキ出力上限 |
| 復帰モジュール | 履歴・スタック閾値・停車確認・ブレーキ／後退出力と時間・後退操舵・回数等 |

STOP・1秒以上の停車後に「停止中に一括適用」。同じノードの変更だけをまとめて検証・適用し、
適用後の現在値を再取得する。既に他の操作で値が変わった場合は再取得を要求する。
通信timeoutでは適用済みか不明なことがあるため自動再送しない。現在値を再取得して確認する。
ノード未起動は未起動表示。起動時固定の項目は表示のみ。復帰の動的変更時は履歴・phaseをresetし、
次のAUTOで新しい値を使う。変更はメモリ上のみで、再起動後は起動YAMLへ戻る。

同じパネルの「実車ラインを切替」は、選択したCenter / Race / CustomへSTOP中に切り替える。
対象はJetsonの適用済みlaneの生成ラインで、地図全体を再転送しない。
Centerの速度も変える場合はパラメータ調整の「参照ライン・Center速度」で一括変更する。
境界／waypoint／区間速度を編集した場合は、従来のライン生成・保存・snapshotプレビュー適用を使う。

検証: `test_tuning_parameters.mjs`、`test_parameter_bridge.py`、復帰adapter test。
実ROS parameter service、Jetson上の追従と復帰、実車は別途確認が必要。


### ステア補正と車両状態パネル

既存のMap Studio実車調整を接続し、「ステア補正」から`steering_offset`を変更できる。
STOP・1秒以上の停車確認後に適用し、適用後の現在値を読み返す。変更はメモリ上だけで再起動時は戻る。
D-pad trimとは補正位置・符号が異なるので、パネルには別々に表示する。
車両状態パネルはAUTO/MANUAL/STOP/PROPO、HOST authority、現在の車両通信異常、
復帰の監視・接触/スタック疑い・制動・後退・再開待ち・中断と試行回数/理由を表示する。
疑いは画像/IMUによる推定であり衝突の確定ではない。ESC表示も指令履歴のモデルである。
接続やtopicが途絶した場合は状態を不明/受信停止に変え、古い正常表示を維持しない。
表示は既存の約1 Hzの状態取得に相乗りし、Jetson側に新しい画像処理を追加しない。

### Section headの学習UI

「学習 → Multi-head → 割当・区間抽出」で、次の工程を操作できる。

1. VSLAM地図を持つ既存mapと生成済みlane/ラインを選び、section→head割当を保存する。
   既存customline速度区間、距離指定、全周用`*`を選べる。同じhead名は共有する。
2. 専門head分は元datasetと同じbagを選び、offline VSLAM（VO/VIO）でmap座標poseを取得する。
   bagは既存の階層pickerを共用。ROS/CUDAコンテナで実行し、再生正常終了後に保存する。
3. 元の教師dataset・割当・採用sectionを選んでdatasetを作る。
   「全周head *」ではpose入力とoffline工程を隠し、VSLAMなしで全教師サンプルを割り当てる。
   区間分け済みデータから指定sectionを再抽出することもできる。
4. 「Multi-head学習」で専門datasetと全周datasetを複数選ぶ。
   routingは自動判定し、学習用・検証用それぞれに全headがあるか表示・検証する。
   未割当datasetとの混在、別routing、別bagのposeは処理前に拒否する。

元の専門datasetは「元の教師datasetを作成」で時刻基準headerを選ぶ。全周用はpose時刻対応不要。
教師の採用条件は従来の固定AUTO／通常MANUALの自動判定を維持する。
割当とposeは`e2e/routings/日付/時刻/名前`、`e2e/trajectories/日付/時刻/名前`へ保存し、
「整理・ごみ箱」で削除・復元できる。中止・失敗の一時出力はcatalogへ公開しない。
共有backbone・各headの学習設定はMulti-head側で指定し、ONNX exportへ進める。
sectionモデルの評価は作成済みsection datasetを選択する。
画像gate・曖昧時のstable切替は未実装。複数headの実走行はRGBのみ。
CPUテスト・ブラウザでの割当保存／全周dataset作成を確認済み。offline VSLAMのROS/GPU実行は未確認。

### 通常モデルとMulti-headのpipeline分離

学習画面上部で「通常モデル · 1 head」と「Multi-head · section別」を選択する。
通常側は通常dataset・実験・ONNX・評価だけを表示し、section操作やhead不足表示を隠す。
Multi-head側は割当・区間抽出から開始し、専門／全周datasetだけを学習候補に出す。
元の教師datasetが必要な場合だけ「元の教師datasetを作成」から共通の抽出画面へ進む。
学習、export、評価、転送の選択肢と履歴もpipelineごとに分離する。
Tiny特徴蒸留は通常側、蒸留済みencoderの再利用は双方で可能。
Multi-head未対応の特徴量可視化・INT8操作はMulti-head側に表示しない。
切替時は学習設定・選択値・表示工程をそれぞれメモリ上で保持する（ページ再読込時は初期状態）。
モデルやdatasetのファイルを移動・複製せず、既存metadataからpipelineを判定する。
HTTPのpipeline指定と実データが食い違う学習・export等はサーバーで拒否する。
内部のtrainer・ジョブ実行・前処理は共通で、学習実装を二重に持たない。

制御学習の既定は通常／Multi-head／RGB／EVSともSmooth L1（β=0.1）・100 epoch・
Plateau scheduler ON（factor=0.5、patience=2、min_lr=1e-7）。
初期化やモデルサイズを切り替えても、入力済みのepochは保持する。
Tinyの特徴蒸留epochは独立して指定し、既定10のまま。
[モデル比較計画](../../docs/e2e_model_comparison.md)にサイズ・初期化・更新範囲・蒸留予算の比較条件を記載する。

復帰パネルは`post_reverse_braking`を「後退後の制動・停車確認」と表示する。制動要求段階であり実ESCの制動成立を示すものではない。

復帰状態post_reverse_pulseを「後退後の制動準備（正パルス）」として表示する。復帰のpost_reverse_pulse_throttle / post_reverse_pulse_sもSTOP時の動的調整対象。

## 走行記録の時系列解析

上部の「解析」タブで、Notebookのrecordからrosbagを選び「選択したbagを解析」を押す。
Jetsonにあるbagは、既存の「転送」画面で取得してから選ぶ。RGB・左IR・EVS metadata・任意の入力topicを指定できる。
通常はbag選択だけで実行でき、検出設定を開くと遅延スパイクの閾値（既定100 ms）を変更できる。

処理は既存の中止可能なジョブで動き、下部の処理一覧でログを確認できる。
完了後は画面内に共通時間軸の折れ線レポートを表示する。ドラッグで拡大し、イベント行から前後1秒へ移動できる。
時系列の先頭には「操舵指令＋IMU角速度」「駆動・制動指令＋IMU加速度」の比較を表示する。
左右の軸は別単位で、IMU topic／sensor軸を選べる。指令の変化点A、応答の変化点Bをクリックすると
手動の時間差msを表示する。機械系や記録時計の差も含む定性的な比較で、時刻の自動補正は行わない。
「操舵と角速度から応答遅れをモデル推定」を開くと、選択区間で符号付きゲイン・オフセットとτを推定できる。
既定の角速度軸はgyro_y。先頭70%で適合し、後半30%で検証して、予測の破線・R²・誤差曲線・不安定な候補の警告を表示する。
範囲や軸を変えたら再推定する。「推定JSONを保存」で条件と結果を保存できる。
τは車体応答や記録のずれを含み、通信遅延の校正や元timestampの補正は行わない。
加速度・frame情報を得るには元bagを再解析する（旧レポートは自動書換えしない）。
「大きく開く」で別タブ表示、「イベントCSV」「集計JSON」で保存できる。
結果は`analysis/pipeline/YYYY-MM-DD/HH-MM-SS/audit_<id>/`へ保存し、解析履歴から再表示する。
元bagの書換え、ROS再生、走行指令publishは行わない。GPUも不要。

解析workerのPythonは`KART_AUDIT_PYTHON`、未指定時はStudioと同じPythonを使う。
DockerにはrosbagsとPlotlyを同梱する。既存イメージは再buildし、アプリを再起動する。
Docker外の環境では、StudioのPythonまたは専用venvへ次を導入する。

```bash
python3 -m venv /tmp/kart-audit-env
/tmp/kart-audit-env/bin/pip install -r tools/pipeline_audit/requirements.txt
KART_AUDIT_PYTHON=/tmp/kart-audit-env/bin/python bash tools/app/start.sh
```

既存のrecord root／`--data-root`を共用し、任意のshell command・実行環境・出力パスをHTTPから受け付けない。
途中・失敗・中止した結果は履歴へ公開せず、過去の結果を維持する。
レポートはsandbox化したiframe／CSPで実行し、アプリ本体のAPIと分離する。ダウンロードは固定の成果物名だけを許可する。

計測値はbag記録時刻ベースで、callback・GPU単独時間・実PWMの計測ではない。
再送・MANUAL・復帰・状態未確認のサンプルを分離し、未記録topicを0msや正常と扱わない。
詳しい指標と制約は[pipeline_audit](../pipeline_audit/README.md)。

公式offline地図生成の姿勢グラフ上限は`config/mapping/workflow.json`
（kart_bringup）の`slam_max_map_size=0`（無制限）を使用する。
300へ戻すと公式既定上限で比較できる。既存地図は変更せず、別名で再生成する。
ROS側cuVSLAM YAMLの`slam_max_map_size`とは別設定。実bag/GPUでの効果は未検証。

公式offline生成のノイズ除去は`kart_bringup/config/mapping/workflow.json`の
`denoising=true`で有効にし、`--override cuvslam.cfg_denoising=true`を渡す。
falseでOFF比較できる。UI再起動後の新規生成に適用し、同じbagから別名で比較する。
ROS側のenable_image_denoisingとは別設定。実bag/GPUでの改善効果は未検証。
