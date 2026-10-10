# Kart Map Studio

Notebookでbagを受信し、Isaac ROS cuVSLAMの地図・点群からHDMapと走行ラインを作成する。
**地図 / 転送 / 学習**の3画面。Jetsonのbringup・車両操作機能は持たない。
学習画面からデータセット作成・DINOv3学習・ONNX export・Jetsonモデル転送を実行できる。
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
| vehicle_length | 0.47 m | 長方形外形の全長 |
| rear_axle_to_rear | 0.1065 m | 後輪軸中心から後端。前後等オーバーハングの暫定仮定 |
| margin | 0.05 m | 車体左右それぞれの余裕 |
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

helper入力は周期スプラインで平滑化（残差二乗和の基準は点数×0.05² m²）し、等弧長で再配置する。
法線方向の実境界までの距離から左右幅を求め、車幅と片側marginを一度だけ最適化に適用する。
補間余裕を片側0.05 m追加し、最大0.05 m間隔で出力を検査する。
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

現在の地図生成は公式 `create_map_offline.py --steps_to_run edex compute_poses`。
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
任意地点からの位置合わせはこの工程の対象外。VGLによる位置合わせは別途実装予定。


### TT-02の暫定寸法

新規ライン生成の車幅既定値を0.19mへ変更した。余裕0.05mは維持し、
現行の線分距離判定は半車幅＋余裕=0.145mとなる。
[公式仕様と暫定外形](../../docs/policies/vehicle_geometry.md)参照。
既存地図に保存したline.settingsは書き換えない。古い地図を開くと旧車幅が復元されるため、
生成設定で車幅0.19を指定し、各ラインを再生成する。
全長0.47mを後輪軸基準の長方形footprintに使用する。


### 長方形の基本接触判定

全ラインの生成結果とexport時に、後輪軸基準の幅/全長/後端距離を使用して再検査する。
車体前端はvehicle_length−rear_axle_to_rear=0.3635m、左右は±0.095m。
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
サーバー終了ではジョブを停止する。中断からの学習再開機能はまだない。

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

保存先はdata-root下の`e2e/{datasets,runs,models}/<name>`、環境設定は`.kart-studio/e2e.json`。
作業中は同一ファイルシステムの非表示一時directoryを使い、成功時のみrenameして公開する。
同名成果物の上書きはしない。既存のCLI成果物の任意インポート・成果物削除は今回の対象外。
学習とexportは同時実行せず、UIのE2E計算ジョブを1件に制限する。
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
