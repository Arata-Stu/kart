# Kart Map Studio 実装・検証記録

確認日: 2026-10-07。実行環境はmacOS。JetsonへのSSHは実行していない。

## 実装範囲

- 地図／転送の2画面、Canvasのパン・ズーム・境界／Custom点編集・Undo/Redo。
- 高さ付き点群の表示フィルタ、Centerline／Raceline／Customline生成、HDMap YAML・CSV出力。
- cuVSLAM 5.0用のオフラインlaunchとbag worker、終了時TFでの点群変換、保存来歴。
- Jetson record探索、SCP pull、map bundle push、一時領域からの公開、ジョブ停止とログ。
- E2E学習は未実装。Jetsonのbringup／実車操作は対象外。

## コード・ローカル検証

| 確認 | 結果 |
|---|---|
| App単体テスト | 22件成功。閉路の位相／向き、開路、曲率低減、車幅、領域逸脱、速度制約、保存競合、export、Host/Origin、転送引数など |
| TF単体テスト | 4件成功。後半TF、古い時刻の遅着、map二重変換防止、3D回転と高さ、TF不足 |
| 転送フロー | 上記22件のうち4件はSSH/SCPをローカルファイル操作へ差し替えたテスト。成功公開、中止時の非公開、更新bagの拒否、現revision＋cuVSLAM送信を検査 |
| Python | compileall、Ruff E4/E7/E9/F/Iチェック成功 |
| JavaScript | 各moduleの構文チェック、ブラウザconsole errorなし |
| launcher | bash構文、shebang・実行権限を確認 |
| VSLAM設定 | 公式release-5.0 commitの独自parameter 58件を照合。欠落0、追加はROS共通use_sim_time |
| ROS package | package.xmlのXML、CMake install先・Python構成を確認。colconは未実行 |

再現:

```bash
PYTHONPATH=tools/app:ros2_ws/src/kart_mapping python3 -m unittest discover -s tools/app/tests -v
PYTHONPATH=ros2_ws/src/kart_mapping python3 -m unittest discover -s ros2_ws/src/kart_mapping/test -v
python3 -m compileall -q tools/app ros2_ws/src/kart_mapping ros2_ws/src/kart_bringup/launch/mapping.launch.py
bash -n tools/app/start.sh
```

## ブラウザ操作

検証サーバーは`./tools/app/start.sh --data-root /tmp/kart-studio-review`。
通常の`record/`・`map/`にはテストデータを追加していない。

- **合成点群6,667点**を読み込み、画面で左右各8点の境界を描画して保存。
- Centerline 178点、Raceline 178点を画面から生成。
- Custom制御点8点を画面から追加し、Customline 157点を生成。
- revision 6でHDMap／CSVを出力し、各CSV行数・列数とJSON/YAML構文を確認。
  Centerlineは4列、Raceline／Customlineは7列。
- Z上限を1.8 mから0.6 mへ変更し、表示点が6,667→4,800点になることを確認。
  保存revisionは変わらず、境界・生成ラインは維持。
- 1280×720の地図と転送画面を目視確認。ページ全体のscrollサイズはviewportと同じ。
  転送左右パネルは各height=495px / scrollHeight=495px。
- 1440×900の地図ライン画面では、inspector height=738px / scrollHeight=738px。
- 生成後に工程が点群へ戻ってしまう挙動を修正。高さ範囲と工程を維持する。
  毎回ログを自動展開せず、下部バーで状況を示す。

合成データはUI・geometryの検証用で、実コースの認識精度・走行性能を示さない。
再利用可能な合成デモは`tools/app/make_demo.py --data-root <新規検証先>`で生成できる。

## 未確認と次の実環境確認

1. CUDA対応LinuxでDocker依存更新・`colcon build --packages-up-to kart_mapping`。
   ROS packageと上流Componentが実際にロードされることを確認する。
2. 左右画像・CameraInfo・静的TFを含む短い停止済みbagで地図作成。
   `save_map`応答、非空cuVSLAMデータ、点群、SLAM path、採用TF時刻を確認する。
3. 実点群の床・壁・コース境界を目視照合。高さとmap座標、ループ閉じ込み後の整合を確認する。
4. 利用者がJetson接続設定・SSH鍵を用意し、停止済みbagの受信と新規map bundle送信を確認する。
5. kartの将来の地図publisher／plannerが`kart.hdmap.v1`とCSVを読み込む統合は別実装。
   現在のexportだけで実車走行可能とは扱わない。

Racelineは境界内の局所曲率低減候補。最短ラップ、実車の操舵応答・制動・安全性は検証していない。


## 2026-10-07: 区間速度・位置合わせ・Raceline方式切替

### 実装

- Customline区間上限を正本へ保存し、始終点をサンプリング点列へ挿入。閉路跨ぎ・重複最小値・事前減速・Undoに対応。
- 元地図から適用先地図へのXY＋yawプレビュー／保存。旧正本backup、元／先fingerprint検査、両地図lock。
- local / mincurv / mincurv_iqpを切替。helperは固定forkを別Pythonで実行し、停止とtimeoutに対応。
- helper補間後の境界違反は試験中に検出。周期平滑化と片側0.05 mの追加補間余裕を採用し、最終境界・曲率検査を維持。
  数学的な最適性や、簡易方式に対する改善を全コースで保証するものではない。

### 確認済み

- アプリ32テスト（既存22＋新規10）成功。TF既存4テスト成功。
- 新規テスト: 短い速度区間の端挿入、事前減速、閉路跨ぎ・重複、無効区間、速度変更の限定的な無効化、
  SE(2)によるXY/headingと速度等の保存、VSLAM／点群不変、backup、古いプレビューの拒否、
  未知方式／開路拒否、依存不足時にfallbackしないこと、中止時の子プロセス終了、実helper両方式。
- 実helper試験環境: macOS / Python 3.14.8の一時venv、NumPy 2.4.6、SciPy 1.17.1、Matplotlib 3.11.0、
  quadprog 0.1.13、helper 0.80（固定commit aa950f6045680366b789dbb855db8d59d54b1db5）。
  Docker運用のPython 3.12とは別環境であり、Docker結合確認とは扱わない。
- 合成点群6,400点のブラウザ操作: Custom区間4〜7 m・上限0.6 m/sを保存して再生成。
  別snapshot地図へX=0.3 m/yaw=8度でプレビュー・保存、3ラインの移行を確認。
  同じ画面からmincurv（463点）→mincurv_iqp（464点）生成完了を確認。
- 1280×720: 文書scroll/client=1280×720、inspector=568/568。
  1440×900: 文書scroll/client=1440×900、inspector=738/738。全体・右パネルとも縦スクロール不要。
- 変更PythonのRuff E4/E7/E9/F/I、compileall、全frontend JSのnode --checkを実施。

### 再現

```bash
KART_TEST_HELPER=1 KART_RACELINE_PYTHON=/path/to/helper-env/bin/python \
  PYTHONPATH=tools/app:ros2_ws/src/kart_mapping python3 -m unittest discover -s tools/app/tests -v
PYTHONPATH=ros2_ws/src/kart_mapping python3 -m unittest discover -s ros2_ws/src/kart_mapping/test -v
```

`KART_TEST_HELPER`未指定時は実helperテスト1件をskipする。残りは標準ライブラリ環境で実行できる。
ROS/GPUによる実bag地図生成、実Jetson転送、実車追従、実会場間での位置合わせは未確認。


## 地図の削除・復元の追加

- 地図を選択 → 上部「削除」→ 対象名と範囲を確認 → ごみ箱へ移動。左下「ごみ箱・復元」で復元。
- 新規3テストで地図一式の復元、revision不一致・処理中の拒否、同名復元・範囲外パス拒否を確認。
- アプリ全体35テスト中34成功、実helper試験1件はこの回ではskip。変更PythonのRuffとJS構文検査成功。
- 合成地図new_sessionをブラウザから削除し、一覧から消えること、ごみ箱に表示されること、復元を確認。
- ごみ箱は地図フォルダを保持するため容量解放にはならない。完全消去は未実装。


## 2026-10-08: IDで管理する複数lane

- `kart.hdmap.v2`の`lanes[]`へ境界・開閉路・Custom点／区間速度・生成結果を分離。
  旧形式はlane_001へ読み込み時に正規化し、閲覧だけでは元ファイルを変更しない。
- GUIのID selector、追加／境界コピー／削除／Undo、他laneの灰色境界表示を追加。
- 全laneをIDを保持して位置合わせし、exportはID別CSVと`offline_lines`参照を出力。
  生成物の役割は`offline_reference`。オンラインselector/planner・障害物回避は未実装。
- アプリ40テスト成功（実helper両方式を含む）。TF4テスト成功。
  追加の5テストは旧形式非破壊読み込み、ID重複／不正ID拒否、lane単位の生成と無効化、
  ID別exportと未生成lane拒否、全laneの位置合わせを検証。
  転送テストも複数laneの正本・ID別CSVを含むbundleに更新し、疑似SCPで確認。
- ブラウザで旧demo_courseのlane_001からshortcutを複製→保存→shortcutだけCenterline生成→
  lane_001の既存Raceline/Customline保持→全lane出力→shortcut削除とUndo→保存・再読み込みを確認。
- 合成地図のexportはrevision-11-v2で、lane_001の3 CSVとshortcutのCenterline CSVを確認。
- 1280×720: ページscroll/client一致、右パネル508/508。
  1440×900: ページscroll/client一致、右パネル678/678。全体／右パネルの縦スクロールなし。
- Ruff E4/E7/E9/F/I、Python compileall、全frontend JSのnode --check成功。
- 実ROS/GPU・実Jetson転送・実車の確認は行っていない。中心線＋幅で左右境界を一括編集するUIは未実装。

- 表示倍率変更時の点群レイヤーをcanvas実バッファ倍率に合わせ、境界と点群の表示スケール一致を確認。

## 2026-10-08 セット編集・走行不可能領域

- アプリ55テスト成功（固定helper環境を指定）。禁止領域の内部・線分横断・接触・余裕・閉路終端、
  不正polygon、保存時の全lane無効化、export、位置合わせ、生成失敗時の正本保持を含む。
- `node tools/app/tests/test_paired.mjs`成功。組の移動・幅・直線の新規作成・左右の方向／閉路位相合わせ。
- kart_hdmapは7テスト中6成功、ROSメッセージ1件skip（ROS未導入）。領域IDと座標の保持を確認。
- 分離デモ `/tmp/kart-set-obstacle-demo` にてブラウザで幅変更、ドラッグ、Undo、
  禁止領域の4頂点配置、保存・再読込を確認。1280×720で領域パネル展開時も操作ボタンが表示範囲内。
- 証跡 `/tmp/kart-studio-evidence/paired-obstacles.jpg`。実bag・GPU・ROS通信・実車は未検証。
- VGL/Joy連携は方針のみ。VGL実装、オンライン回避、実機検証は今回追加していない。

## 2026-10-10: オフライン試走・方向反転

- Python app suite: 67件、66成功・1 skip。
- Node: `test_paired.mjs`、`test_simulation.mjs`成功。
  閉路／開路の反転往復、左右交換、巻き跨ぎを含む区間速度の位置保持、
  正逆円周の1周完了、速度profile付き開路の終点到達を確認。
- 合成地図を隔離データルートで起動し、ブラウザでCenterline試走、方向反転、再生成、
  逆方向試走を確認。正逆とも24.3秒で1周完了、最大追従誤差の表示は0.03m。
  保存された再生成Centerlineの符号付き面積も逆方向になったことを確認。
- 狭幅のブラウザでも試走パネル内に設定・地図・操作・計測値を収める配置を確認。
- 上記数値は合成地図・簡易モデルの結果。実車追従、ROS/GPU、VSLAM再現、
  試走軌跡の衝突判定は検証対象外。
