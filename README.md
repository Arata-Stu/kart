# kart

JetPack導入直後の初回準備は [JetsonセットアップREADME](docs/setup/README.md) を参照。

## 地図UI: Kart Map Studio

[使い方・構成](tools/app/README.md) / [設計方針](docs/policies/ui_app.md)

```bash
./tools/app/start.sh
# ブラウザ: http://127.0.0.1:8766
```

「地図」「転送」の2画面で、Isaac ROS VSLAM地図作成、点群の高さフィルタ、HDMap境界編集、
Centerline・Raceline・Customline生成、Jetsonのrecord探索・SCP受信・地図送信を行う。
Macは編集・転送、VSLAM処理はCUDA対応Linuxのkartコンテナで使う。
Jetsonのbringup操作は含めない。E2E学習は今後追加予定。
ROSを使う場合は`colcon build --symlink-install --packages-up-to kart_mapping`でビルドする。
実ROS/GPUと実Jetson転送の確認状況は[検証記録](docs/ui_app_validation.md)を参照。


タミヤRCカーをJetson Orin Nanoで自動走行させるためのROS 2プロジェクト。
実行環境は **JetPack 7.2 / Isaac ROS 5.0 / ROS 2 Lyrical / Docker**。
開発環境、入力専用の`kart_joy`と車両操作用の`kart_joy_manager`、動作モード管理、command mux、JPBB USB bridgeを実装。
自動運転アルゴリズムは未実装。ROS結合ビルドおよびRealSense・Joy・車両の実機動作は未検証。

Joyノードのビルド・GUI/CUI設定手順は [kart_joy README](ros2_ws/src/kart_joy/README.md) を参照。

車両連携の構成・ビルド・起動・操作は [vehicle基盤](docs/vehicle.md) を参照。

開発時の共通ルールは [ROSパッケージ設計・記述ルール](docs/policies/ros_packages.md) を参照。
ルートの [AGENTS.md](AGENTS.md) からも案内する。

## ROSパッケージのREADME

各READMEに既定ノード名、入出力topic名・型、パラメータ／launch引数を記載する。

- [kart_joy](ros2_ws/src/kart_joy/README.md): 入力取得・配置設定・GUI/CUI
- [kart_system](ros2_ws/src/kart_system/README.md): Joy manager・mode manager・command mux
- [kart_vehicle](ros2_ws/src/kart_vehicle/README.md): USB bridge・トリム・基板状態
- [kart_bag_manager](ros2_ws/src/kart_bag_manager/README.md): L1/R1録画操作・rosbagプロセス管理
- [kart_interfaces](ros2_ws/src/kart_interfaces/README.md): msg定義
- [kart_bringup](ros2_ws/src/kart_bringup/README.md): 全体起動・launch引数

## 外部リポジトリ

`packages.repos` はCLI forkの検証済みcommitを固定する。
開発ブランチは `carbuncle01/isaac-ros-cli` の `kart`。

```bash
# kartルート、Ubuntuホスト上
sudo apt install python3-vcstool
./scripts/repos.sh import
./scripts/repos.sh status
```

`tools/isaac-ros-cli` は外部Gitとして管理し、kart本体へ内容を追加しない。
CLIを更新する際はforkでテスト・commit・pushした後、manifestのSHAを更新する。
SHA固定で取り込んだcheckoutはdetached HEADになる。開発は専用checkoutのkartブランチで行う。

## Jetsonへのセットアップ

公式のJetPack 7.2用Docker / NVIDIA Container Toolkitセットアップを済ませる。
次の操作はUbuntuホストで行う。macOS上では.deb作成やJetson GPU動作は検証できない。

```bash
sudo apt install build-essential dpkg-dev debhelper dh-python
cd tools/isaac-ros-cli
make build
sudo make install
sudo isaac-ros init docker
cd ../..
# 初回は下記「jtop setup」に従ってホスト側も準備する
./scripts/dev.sh --build-local
```

`make install` は親ディレクトリにあるCLI .debが一つであることを要求する。
複数世代をビルドした場合は古い成果物を別の場所へ保管してから実行する。
このインストールはシステムの `isaac-ros-cli` をkart版に置き換える。
JetPilotなど別プロジェクトと同じホストで使う場合は切替が必要になる。

CLI更新は、kart本体・`packages.repos`を更新後に`./scripts/update-isaac-ros-cli.sh`を実行する。
ビルド依存導入・固定commit取得・debビルド・インストールを行う。
詳細は[セットアップREADME](docs/setup/README.md#cliを後から更新する)を参照。

通常の起動は `./scripts/dev.sh`。コンテナ内では次の配置になる。

```text
/workspaces/                # ホストのkart全体を1回だけマウント
├── .git/                   # 通常のgit cloneならGit管理情報も共有
├── record/
├── map/
├── docker/
├── scripts/
├── tools/
└── ros2_ws/                # ISAAC_ROS_WS / ISAAC_DIR / 作業ディレクトリ
    └── src/
```

プロジェクトルートは読み書き可能なbind mountで、`.git`も含まれるため、
通常のcloneではコンテナ内の `git -C /workspaces pull` がホスト側にも反映される。
SSH/HTTPS認証は別途用意する必要がある。git worktreeで`.git`が外部パスを指す場合は
この単一マウントだけでは不十分なので、車載ホストでは通常のcloneを使う。
`record/`・`map/`はホストと共有し、中身は既定でGit管理から除外する。
この作業時点のローカルkartには`.git`とremoteがなく、Git操作の実行確認はまだ行っていない。

コンテナ内の対話BashはLyricalと、存在する場合はワークスペースのinstallをsourceする。
ワークスペースのビルドはコンテナ内で次を実行する。スクリプトが`ros2_ws`へ移動し、
`colcon build --symlink-install`を実行する。追加引数はそのままcolconへ渡す。

```bash
/workspaces/scripts/build.sh
# 車両関連だけビルドする場合
/workspaces/scripts/build.sh --packages-up-to kart_bringup
source /workspaces/ros2_ws/install/setup.bash
```

プロジェクトルートからは`./scripts/build.sh`、`ros2_ws`からは`../scripts/build.sh`でも実行できる。
初回や依存追加時は、先に`ros2_ws`で`rosdep install --from-paths src --ignore-src -r -y`を実行する。

## カスタマイズの分担

- CLI fork: プロジェクト単一マウント、ROS workspace環境変数、ホストのシェル設定を継承しない変更。
- `docker/Dockerfile.kart`: kart固有の依存とシェル初期化。
- `docker/dockerargs`: 追加マウント・デバイス設定。ホスト `/dev` 全体の追加マウントは行っていない。
- `scripts/.isaac_ros_common-config`: 自作Dockerfileの探索先。
- `scripts/.build_image_layers.yaml`: レイヤー順序。センサー追加時はここにも反映する。
- `ros2_ws/.isaac-ros-cli/config.yaml`: レイヤー選択とコンテナ名。

RealSenseは `isaac_ros → realsense → kart` の順でビルドする。
`docker/Dockerfile.realsense` は固定CLI commitの公式Dockerfileを基に、
`LIBREALSENSE_BUILD_TOOL_OPTIONS="--force_rsusb --no_cuda"` を既定値にしたもの。
librealsense v2.56.3 / realsense-ros r/4.56.3と公式Lyricalパッチを使用する。
ビルド用スクリプトとパッチは `context_overrides.realsense` で固定CLIのdockerディレクトリから参照する。
無効にするのはlibrealsense内部のCUDA処理のみで、Isaac ROS側のCUDAは維持する。
SilkyEvCamは未導入。

## Ubuntuの基本ツール

`docker/Dockerfile.kart`で、普段の開発・調査に使うAPTパッケージも導入する。

| 用途 | 追加ツール |
| --- | --- |
| ターミナル・編集 | tmux、vim、nano、less、bash-completion、man-db |
| ファイル・検索 | file、tree、ripgrep (`rg`)、jq、zip、unzip、xz-utils |
| 転送・Git | rsync、openssh-client (`ssh` / `scp`)、git-lfs |
| プロセス・デバッグ | htop、procps (`ps` / `top`)、psmisc (`pstree` / `fuser`)、lsof、strace、gdb |
| ネットワーク | iproute2 (`ip` / `ss`)、iputils-ping、dnsutils (`dig`)、netcat-openbsd (`nc`)、socat、tcpdump、iperf3 |
| センサー確認 | usbutils (`lsusb`)、v4l-utils (`v4l2-ctl`) |
| ビルド | cmake、ninja-build、pkg-config |

Git・build-essentialはkartの既存依存、curl・wget・sudo・CA証明書などは基底イメージの既存依存を使う。
追加ツールはPython環境構築より後ろの独立したRUNに置くため、一覧の変更時にも
それ以前のDockerキャッシュを利用できる。導入先はコンテナ内。
反映は、既存`kart_dev`を停止してから`./scripts/dev.sh --build-local`で再ビルド・再作成する。

## Isaac ROS・推論・Python依存

追加先は `docker/Dockerfile.kart`。以下をAPTで導入する。

| 用途 | パッケージ |
| --- | --- |
| VSLAM / VIO | `ros-lyrical-isaac-ros-cuvslam`（5.0で名称変更） |
| VGL / mapping | `ros-lyrical-isaac-ros-visual-global-localization`、`ros-lyrical-isaac-ros-visual-mapping`、`ros-lyrical-isaac-mapping-ros` |
| E2E推論の前処理・推論 | `ros-lyrical-isaac-ros-dnn-image-encoder`、`ros-lyrical-isaac-ros-tensor-rt`、`ros-lyrical-isaac-ros-launch-utils` |
| エンジン生成 | `tensorrt`、`python3-libnvinfer`（`trtexec`とPython API） |
| Jetson診断（arm64のみ） | `ros-lyrical-isaac-ros-jetson-stats`、固定版の`jetson-stats` |

2026-10-04にIsaac ROS 5.0の`noble`（amd64）および`noble-jetpack`（arm64）の
公式APT一覧で確認した。mappingも今回のJetson構成に含める。
`isaac_mapping_ros`はESS・nvbloxなども依存に持つため、イメージ容量・初回ビルド時間は増える。
APT配布の存在はOrin Nanoでの処理速度・メモリ容量の検証とは別。
5.0では旧NITROS依存をコピーせず、各APTパッケージの新しい依存関係を使う。
APTパッケージの完全なバージョン固定はまだ行っていない。

Python依存は **uv 0.12.0 + Python 3.12** を使用する。
Docker内のPython本体はUbuntuの`/usr/bin/python3`を使い、uvによる別Pythonのダウンロードは禁止する。

| 環境 | 定義・lock | 用途 |
| --- | --- | --- |
| `/usr/bin/python3` | ROS / TensorRTはAPT、jtopは`docker/jetson-stats.env` | ROSノード・jtop。jtopの導入には`uv pip`を使用 |
| `/opt/inference` | `docker/python/inference/{pyproject.toml,uv.lock}` | ONNX、ONNX Runtime CPU、ONNX Slim、Polygraphy、OpenCV、量子化・校正 |
| `/opt/env`（Notebook / amd64のみ） | `docker/python/raceline/{pyproject.toml,uv.lock}` | NumPy、SciPy、Matplotlib、quadprog、trajectory helper |

`/opt/inference`のみ`--system-site-packages`でAPT版TensorRTを参照する。
この環境のNumPy/OpenCV等はROSと別バージョンになり得るため、ROSノード起動には使わない。
TensorRT/CUDAをPyPIから重複インストールせず、GPU推論はTensorRT、ONNX校正はCPUで行う。
INT8は代表データで校正したQ/DQ ONNXを作り、対象JetsonでTensorRTエンジンへ変換する想定。
モデル固有の校正データ読み込み・前処理・精度評価は未実装。PyTorch学習・QAT環境は含めない。

Raceline用の`/opt/env`は**NotebookのLinux x86_64（Debian名: amd64）イメージだけ**に導入する。
Docker内の`dpkg --print-architecture`で判定し、Jetson（arm64）ではRacelineの`uv sync`とimport検査をスキップする。
依存定義・lockファイルは両アーキテクチャへ配置するが、JetsonにRaceline環境は作成しない。
推論環境やROSが別途必要とするNumPy等は引き続き導入する。
NotebookではMap Studioが`/opt/env/bin/python`を自動利用するため、追加のpip操作は不要。

trajectory helperは指定forkのcommit `aa950f6045680366b789dbb855db8d59d54b1db5` を固定。
提示された`uv pip install --no-deps git+...`で導入するライブラリを、
今回は`tool.uv.sources`に記述して依存ごとlockする。別途同じライブラリを二重インストールする必要はない。

環境は自動activateせず、利用するPythonを明示する。コンテナ内の確認例:

```bash
uv --version
/usr/bin/python3 -c 'import tensorrt; print(tensorrt.__version__)'
/opt/inference/bin/python -c 'from onnxruntime.quantization import quantize_static, CalibrationDataReader'
# 以下のRaceline確認はNotebook / amd64のみ
/opt/env/bin/python -c 'import trajectory_planning_helpers, quadprog'
trtexec --help

# Notebook / amd64ではuv経由でも実行可能。起動時に依存を更新しない。
UV_PROJECT_ENVIRONMENT=/opt/env uv run --no-sync \
  --project /opt/kart/python/raceline python -c 'import trajectory_planning_helpers'
```

依存追加時は該当`pyproject.toml`を編集し、uv 0.12.0でlockを更新する。

```bash
uv lock --python 3.12 --project docker/python/raceline
# 推論側を変更した場合:
uv lock --python 3.12 --project docker/python/inference
```

`pyproject.toml`と`uv.lock`をセットで管理する。Dockerは`uv sync --locked`を使うため、
lockの更新漏れはビルドエラーになる。racelineのCOPY/RUNは推論環境より後ろに配置し、
racelineだけの依存更新では推論側のDockerキャッシュを維持する。

### jtop setup

ホストとDockerは**7.2.0**に揃える（最新追従ではなくJetPilotと同じ固定版）。
`docker/jetson-stats.env`が共通のバージョン・commit定義。
JetPilotで使っていた任意ライブラリのロード失敗対策パッチも適用する。
ホストにGitと[uv](https://docs.astral.sh/uv/getting-started/installation/)を準備したうえで、
必要な場合だけ次をJetsonホストで実行する。この操作はホストのjtopを置き換え、サービスを再起動する。

```bash
sudo ./scripts/setup-jtop.sh "$(command -v uv)"
sudo usermod -aG jtop "$USER"
# グループ追加後はログインし直す
/usr/bin/python3 scripts/check-jtop.py
```

`dev.sh`はJetson上でバージョンと稼働中サービスを確認してからCLIを起動する。
自動インストール・サービス再起動は行わない。標準jtopの比較に加え、
固定版のハンドシェイク情報を使ってパッチ番号まで比較する。
CLI既存処理が`jtop`グループと`/run/jtop.sock`をコンテナへ引き継ぐ。
イメージ再ビルド・コンテナ再作成後、コンテナ内でも次を実行する。

```bash
/usr/bin/python3 /workspaces/scripts/check-jtop.py
ros2 pkg prefix isaac_ros_jetson_stats
```

古いイメージ・起動中のコンテナはホスト更新だけでは更新されない。
固定版を変更する場合はホストとDockerを同時に更新し、ホストサービスとコンテナを再起動する。
このチェックは監視データのみ読み、ファン・電力モード・クロックを変更しない。

参考: [Isaac ROS 5.0 cuVSLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_cuvslam/index.html)、
[Isaac Mapping ROS](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_mapping_and_localization/isaac_mapping_ros/index.html)、
[TensorRT node](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_inference/isaac_ros_tensor_rt/index.html)、
[ONNX Runtime量子化](https://onnxruntime.ai/docs/performance/model-optimizations/quantization.html)。

設定変更後は `./scripts/dev.sh --build-local` でイメージを作り直す。
ビルドログで `Build with CUDA: false` と `BUILD_WITH_CUDA=false` を確認し、
実機でRGB・Depth・IMUと使用するalign／点群処理を検証する。
CLIのイメージハッシュ・キャッシュ処理は公式のまま。COPY元だけの変更は
CLIのイメージ名へ反映されない場合があるため、再ビルド・新しいIMAGE IDと
コンテナ内ファイルの反映を確認する。JetPilotのビルド最適化は未移植。

## DDS設定と変更の影響

CycloneDDS (`rmw_cyclonedds_cpp`) を最後のkartレイヤーで導入する。
`docker/dockerargs` でコンテナの環境変数を設定し、
`CYCLONEDDS_URI` はプロジェクトマウント内のXMLを直接参照する。
XMLはDockerfileへCOPYしないため、値だけの変更ではイメージ再ビルド不要。

- `lo` / IPv4のみで通信し、探索先は127.0.0.1。multicastは使わない。
- `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`。外部static peerは指定しない。
- `ParticipantIndex=auto`。多数のプロセスで探索上限に当たる場合は実測して調整する。
- `MaxMessageSize`・送受信バッファ・WHCはCycloneDDS既定値。
- ホストの `rmem_max`・`ipfrag_*`・loopback multicastフラグは変更しない。
- `ROS_DOMAIN_ID`は既存のCLIによるホスト環境変数の引き継ぎを維持。

| 変更内容 | 必要な反映操作 | 再ビルドへの影響 |
| --- | --- | --- |
| 初回CycloneDDS導入 | `./scripts/dev.sh --build-local`、コンテナ再作成 | kart最終レイヤーを変更。Isaac ROSとRealSenseの入力は不変 |
| XMLのインターフェース・バッファ値 | 対象ROSプロセスを再起動 | 不要。起動済みDDSには自動反映されない |
| `docker/dockerargs`の環境変数 | コンテナ再作成 | 不要。既存コンテナへのattachでは更新されない |
| CycloneDDSパッケージの変更 | Dockerビルド、コンテナ再作成 | kartレイヤー以降 |

CLIは動作中のコンテナがあると先にattachするため、再ビルド時は走行・記録を停止し、
既存の `kart_dev` が停止していることを確認する。ビルド時は通常のキャッシュを使い、
`--no-cache`は付けない。下位レイヤーのキャッシュが削除済み、または基底イメージや
CLI commitを変更した場合まで、下位ビルドの省略を保証するものではない。

コンテナ内のROSノード・rosbag・CLIツールに適用される。別PCからROS topicを直接
購読する構成は対象外。host networkなので、同じJetson上のホストプロセスや別の
host-networkコンテナは、互換DDS設定・同じdomainなら通信できる。これはセキュリティ境界ではない。
ホスト側で起動するROSには、このコンテナの環境変数は自動適用されない。
RMW切替後のROS CLIは、同じ環境で `ros2 daemon stop` してから使う。

このDDS設定だけではzero-copyにならない。composable nodeの配置、
`use_intra_process_comms`、QoS、GPUバッファ対応は各ROSノードの実装で設定する。

コンテナ内での確認例:

```bash
printenv RMW_IMPLEMENTATION CYCLONEDDS_URI ROS_AUTOMATIC_DISCOVERY_RANGE
ros2 pkg prefix rmw_cyclonedds_cpp
ros2 daemon stop
# 対象ノードを起動して、別プロセスから発見・購読できるか確認
ros2 node list
ros2 topic list
```

XMLの構文・レイヤー解決はローカルで確認する。DDSによる実通信、D455のフレーム欠落、
記録併用時の遅延はJetson上で別途検証する。

参考: [CycloneDDS discovery](https://cyclonedds.io/docs/cyclonedds/latest/config/discovery-config.html)、
[CycloneDDS設定仕様](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html)。

## 検証範囲

CLIのPython単体テストと、manifestの取り込みをmacOSで確認する。
.deb作成、Dockerイメージビルド、Jetson上のセンサー・GPU動作は別途実機確認が必要。
追加Python環境はuvのLinux arm64/amd64向け`sync --locked --dry-run`で依存解決を確認した。
これはDockerビルド・ネイティブ拡張のimport・GPU実行の成功を意味しない。

構成案は [docs/tree.md](docs/tree.md)。
公式手順: https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html

### DINOv3 E2E learning

[学習・推論pipeline](ros2_ws/src/kart_e2e/README.md)を追加。
ステア＋スロットル、またはsteer-only＋固定スロットルに対応する。
公式encoderソースは任意導入の`vcs import . < e2e.repos`、モデル重みは別途取得。
[設計方針](docs/policies/e2e.md)と`kart_bringup/config/e2e/`を参照。
実行時は公式Isaac ROS GPU画像encoder＋TensorRT＋C++ decoder。PyTorchは学習・ONNX変換専用。
既定は車両に接続しない推論出力。ROS/GPU・実機検証は未完了。
