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

既定の地図生成は公式 `create_map_offline.py --steps_to_run edex compute_poses`。
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

Jetsonのrecord探索はフォルダをツリーとして展開・折りたたみできる。親・兄弟フォルダを残して現在のフォルダと選択bagを強調し、上部のパス各階層から親へ戻れる。接続設定の変更時にはツリーと選択をリセットする。

「ログ・履歴」で処理を選び、「ログをコピー」で表示中のログをコピーできる。表示は末尾約64KBが対象で、ログが空の場合はコピーできない。

HDMap編集ではカーソル位置への追加結果を点線でプレビューする。laneセット編集は左右境界・幅・追加区間を表示し、2点目による最初の境界の向き変更も実際の追加処理と同じ計算を使う。左右個別・Custom・障害物は次の線分と閉路への接続を表示する。既存点付近は移動ガイドに切り替わり、プレビュー自体は地図やUndo履歴を変更しない。

### HDMapのリセット・waypoint区間

地図上部の「HDMapリセット」は全lane・障害物・生成ライン・区間速度を空にして、現在のlane IDの空laneを1つ残す。点群とVSLAM地図は保持する。確認後も保存前の編集状態で、Undo可能。「保存」で確定する。以前書き出したexportは自動更新されないため、ラインを作り直して再exportする。

Customの「区間速度」は番号付きwaypoint図と開始・終了WPの選択欄で指定する。開始欄を選んで図をクリックすると終了選択へ進み、選択区間が紫で表示される。閉路は末尾→先頭を跨ぐ区間と「WP 1（周回末尾）」に対応する。既存の距離指定は勝手に丸めず保持する。内部保存は従来の距離形式で、Custom点列を編集した後は区間を選び直す。

Raceline初期値はJetPilot `tools/app/frontend/app.js` と `python_ws/map_tools/generate_raceline.py` の `race-stacks` presetを参照。共通項目は点間隔0.10m、曲率上限1.0/m、余裕0.05m、最高速度3.0m/s、横加速度2.5m/s²、加速1.5m/s²、減速2.5m/s²。kartの車体幅0.19m・全長0.47m・後輪軸位置は既定のTT-02モデルを保持。JetPilotの最低速度0.8m/sは停止・制約と競合するため導入しない。helperの内部再サンプリング・平滑化はアルゴリズムが異なるため同一値を意味しない。保存済みラインの設定は維持する。

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

既存地図を開き「VGL地図を追加生成」を選択する。ALIKED/LightGlueモデルはmodelsとmap以下の候補から選ぶかパスを入力し、画像サイズ（既定424×240）と保存名（地図名-vgl-vN）を指定する。実行GPUでbuild済みのengineが必要で、download/buildは自動実行しない。公式offline処理の保存画像・posesがない場合は生成できない理由を表示する。

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
未保存編集は先に保存し、処理中はmap/GPU/ROS domainを既存ジョブ機構で排他する。

`GET /api/dense-steps`は工程の可否を返し、`POST /api/dense-step`はid/revision/step/resolutionを受け取る。
公式runを`dense_runs/<uuid>`へコピーし、VSLAM工程を呼ばずdepthまたはoccupancyだけを実行する。
元のofficial_outputは保持し、depth_output/dense_outputをmapping_result.jsonとmap.json.mappingへ登録する。
コピーには画像・中間成果物の分だけ追加ディスク容量が必要。旧成果物は保持する。
失敗・キャンセル時は未登録の作業コピーを削除する。成功時のみrevisionを進める。
深度を再生成した際は旧occupancy/meshを残し、「旧深度の成果物」と表示する。
新しいoccupancy工程が成功すると新成果物へ切り替える。独立したmesh-only工程はない。
テスト: test_dense_steps.py（公式処理はmock、GPUでの追加工程は未検証）。

### ジョブの停止と時間表示

画面下部の「処理を停止」は、同じ欄に表示されている最新の実行中ジョブを対象にする。
複数処理から選ぶ場合は「ログ・履歴」を開き、各行または選択ログの停止ボタンを使う。
停止要求後は「停止要求中」を表示し、実際の終了までリソースの排他を維持する。
子プロセス群へSIGINTを送り、応答しない場合は既存の8秒後SIGTERM・さらに3秒後SIGKILLで停止する。
Python内の作業は各workerのキャンセル確認地点で停止する。公開済み成果物は取り消さない。

経過時間はサーバーのmonotonic clockで計測し、約1.5秒ごとのUI更新で表示する。
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
現在はRGB専用。ONNX/TensorRT実行時の特徴量やEVSの表示は対象外。

データセット作成・rosbag評価のbag選択は「bagを階層から選択」でNotebookのrecord配下をツリー表示する。
転送画面と同様にフォルダを展開し、▣のbagを選択する。選択後は相対パスを表示し、
同名bagも親フォルダで区別する。データセット名候補は選択bag名から引き続き自動入力する。

学習の「全体をfine-tuning」はDINOv3 backboneと制御headを同時に更新する。
標準学習率はhead学習`1e-4`、全体fine-tuning`1e-5`（手動変更可）、epochは両方10。
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

「学習環境」のEVS encoder PT/JSONの既定パスはrepo内
`weights/evs/event_encoder.pt`と`weights/evs/event_encoder.json`。
利用者がEventStateで生成した対応するファイルを配置する。自動ダウンロードはしない。
「学習」の入力種別もEVSへ切り替えるとEVSデータセットだけが選択でき、
`train --modality evs`で制御head学習を実行する。epoch=10、workers=8、特徴量キャッシュONが既定。
全体fine-tuningにも対応し、この場合は特徴量キャッシュを使用しない。RGB Tiny蒸留はEVSに適用しない。

学習後の損失カーブ、ONNX export、転送、整理・ごみ箱は共通。
EVSのオフライン評価はモデル選択後に作成済みEVSデータセットを選択する。
RGBは従来どおりbagを直接評価する。特徴量可視化は現在RGBのみ。
GUIの入力切替とジョブ生成・成果物公開をローカル検証済み。
実RAW・学習済み20ch重み・GPU/TensorRT・実車での一連の実行は別途検証する。

EVS抽出時にh5pyで`numpy.dtype size changed`が出る場合は、EVS RAW抽出用Pythonの
NumPyとapt版h5pyのABI不整合。kart_e2e READMEの「RAW readerのNumPy / h5py ABIエラー」
を参照。RAWパスやmanifestの変更では解決しない。

EVSのRAW順序乱れ許容幅は既定5000µs（0..100000、0は厳格）。
窓終端＋許容幅まで先読みし、元時刻のまま安定整列する。時間補間・イベント削除は行わない。
上限を超える逆行や負時刻は停止する。学習済みencoderの20ch定義は変更しない。
