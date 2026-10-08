# JetPack導入直後からのkartセットアップ

Jetson Orin NanoにJetPackを入れ、初回ログイン・ネットワーク接続を済ませた状態から、
kartのDocker環境を作り、ROSパッケージとJoy入力を確認する手順。
**NVMeへ直接フラッシュしてroot filesystemをNVMe上に置く前提**。保存先の移行は不要。
対象はこのリポジトリの **JetPack 7.2 / Isaac ROS 5.0 / ROS 2 Lyrical** 構成。
JetPackの書き込み自体は[Orin Nano公式Quick Start](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html)を参照する。
別バージョンへの互換性は未確認。公式の最新手順とkartの固定構成を混ぜず、導入版を記録する。

## 1. Jetsonホストの確認・基本ツール

ここから第6節までは **JetsonホストのBash** で実行する。
ホストへROSやlibrealsenseを手動導入する必要はない。kartではDockerイメージに導入する。

```bash
uname -m                         # aarch64
cat /etc/os-release              # Ubuntu 24.04系
cat /etc/nv_tegra_release         # JetPack 7.2のL4TはR39系
apt-cache policy nvidia-jetpack nvidia-l4t-core
lsblk -f
df -h / /var/lib
locale                          # UTF-8であること

sudo apt update
sudo apt install -y software-properties-common ca-certificates curl gnupg git
sudo add-apt-repository -y universe
sudo apt update
sudo apt install -y python3-vcstool build-essential dpkg-dev debhelper dh-python \
  python3-venv python3-click python3-pydantic python3-termcolor python3-yaml \
  git-lfs usbutils bluez
```

NVMeへ直接フラッシュするため、kartは`~/workspaces/kart`、Dockerは既定の保存先を使う。
`findmnt -T /`と`findmnt -T /var/lib`でNVMe上のroot filesystemにあることを確認する。
RealSenseのソースビルド、mapping依存、イメージキャッシュとrecordで容量を使う。
公式のJetson要件は128 GB以上のNVMe。初回ビルド前に空き容量を確認する。
JetPack SDKの追加が必要なら、導入したL4Tに対応する公式JetPack手順で補う。
GPUドライバを一般PC向け手順で置き換えない。

### UTF-8でない場合

日本語UTF-8 localeが設定済みなら変更不要。UTF-8でない場合だけ:

```bash
sudo apt install -y locales
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

### BSPのみをフラッシュした場合のJetPack SDK

BSP（OS・ドライバ）だけではJetPack SDK一式の導入済みとは限らない。
SDK ManagerでSDKも導入済みなら重複操作は不要。未導入なら、APT候補が導入済みL4Tに対応することを確認して:

```bash
apt-cache policy nvidia-jetpack nvidia-l4t-core
sudo apt install -y nvidia-jetpack
```

対応するAPT候補がない場合は別リリースのrepositoryを混ぜず、フラッシュした版のJetPack SDK導入手順を確認する。

## 2. DockerとNVIDIA Container Toolkit

まず既存導入を確認する。

```bash
command -v docker || true
command -v nvidia-ctk || true
apt-cache policy docker.io nvidia-container-toolkit
```

Docker未導入の場合、公式Jetson Docker Setupに合わせDocker公式配布を使用する。
既にDockerが導入済みならこのインストールは省略する。

```bash
# JetPack repositoryからJetson向けcontainer依存を導入
sudo apt install -y nvidia-container curl
curl -fsSL https://get.docker.com -o /tmp/kart-install-docker.sh
less /tmp/kart-install-docker.sh
sh /tmp/kart-install-docker.sh
```

```bash
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

NVIDIA Container Toolkit未導入でAPTに候補がある場合:

```bash
sudo apt install -y nvidia-container-toolkit
```

候補がない場合は[NVIDIA公式のAPT導入手順](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian)で
production repositoryを追加してから導入する。Toolkitのバージョンはkartでは固定していない。
導入した版は`dpkg-query -W nvidia-container-toolkit`で記録する。

DockerへNVIDIA runtimeを登録する。

```bash
sudo nvidia-ctk runtime configure --runtime=docker --set-as-default
sudo systemctl daemon-reload
sudo systemctl restart docker
```

**ログアウトしてログインし直す**（dockerグループを反映）。以降は一般ユーザーで実行する。

```bash
id
docker version
docker run --rm hello-world
docker info --format '{{json .Runtimes}} {{.DefaultRuntime}}'
# runtimesにnvidiaがあり、DefaultRuntimeもnvidia
docker run --rm --gpus all ubuntu:24.04 bash -lc 'echo "NVIDIA runtime OK"'
nvidia-ctk --version
```

`hello-world`はDocker起動、`--gpus all`のコマンドはGPU runtimeでのコンテナ起動確認。
CUDAの計算を実行する検証ではない。GPU可視性は第7節で別に確認する。
既定runtimeのNVIDIA設定はJetPilotでも行っている。`nvidia-ctk`で既存daemon設定へ反映する。
出典: [公式Jetson Docker Setup](https://docs.nvidia.com/jetson/agx-orin-devkit/user-guide/latest/setup_docker.html)。

### 電力モード・クロック（性能確認時）

公式Isaac ROSはMAXNと最大クロックを案内している。JetPilotではMAXN_SUPERのmode 2と
最大クロック・ファンを使用しているが、kartではモード番号を機種・フラッシュ構成から確認する。
Jetsonホストで、実機に対応する定義と現在値を表示する:

```bash
sudo nvpmodel -q --verbose
rg 'POWER_MODEL|NAME=' /etc/nvpmodel.conf
sudo jetson_clocks --show
```

十分な電源・冷却を用意して性能測定する場合、実機のMAXN/MAXN_SUPERに対応するIDを選ぶ。
番号2と定義名が一致することを確認できた場合のみ、`sudo nvpmodel -m 2`を使える。
再起動を要求された場合は再起動後、選択されたモードを再確認して:

```bash
sudo nvpmodel -q
cd ~/workspaces/kart
./scripts/jetson_max_performance.sh
# 設定を変更せず確認だけ:
./scripts/jetson_max_performance.sh --check
```

この操作はホストの電力・クロック・ファン設定を変更する。再起動後は必要に応じて再適用する。
kartのlaunchやjtopチェックはこれらを変更しない。
`jetson_max_performance.sh`はJetsonホスト専用。MAXN / MAXN_SUPERを名前で確認してから
最大クロック・ファンを適用し、CPU・GPU・EMCの固定とファン状態を検査する。
電力モード変更・再起動は自動実行しない。`--check`は変更せず検査し、不一致なら非ゼロで終了する。
検査は`jetson_clocks --show`の表示形式に依存し、実機での確認は未実施。
モードの出典: [Jetson Orin電力・性能設定](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html)。

### RealSenseのホスト側udevルール

RealSense使用時は、コンテナ側librealsenseと同じv2.56.3のルールをJetsonホストへ導入する。
既存ルールがある場合は内容を比較し、必要ならバックアップしてから置き換える。

```bash
curl -fsSL https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.3/config/99-realsense-libusb.rules \
  -o /tmp/kart-99-realsense-libusb.rules
less /tmp/kart-99-realsense-libusb.rules
sudo install -m 0644 /tmp/kart-99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

適用後、RealSenseを抜き差しする。コンテナ内にルールを置くだけではホストのUSB権限へ適用されない。

## 3. kartを配置・外部CLIを取得

例では`~/workspaces/kart`を使う。NVMe上に別の場所を使う場合は以降のホストパスを読み替える。
`KART_REPO_URL`には実際のkartリポジトリURLを設定する（このcheckoutにはremote情報がないため固定URLは記載しない）。

```bash
mkdir -p ~/workspaces
read -r -p 'kart repository URL: ' KART_REPO_URL
git clone "$KART_REPO_URL" ~/workspaces/kart
cd ~/workspaces/kart
./scripts/repos.sh import
./scripts/repos.sh status
git -C tools/isaac-ros-cli rev-parse HEAD
mkdir -p record map config
```

既にkartを配置済みならcloneせず、そのルートからimportを実行する。
外部CLIのSHAは`packages.repos`の`version`と一致すること。
既存の外部checkoutがある場合、importだけで指定SHAへ切り替わったと仮定せず照合する。
車載ホストでは通常のcloneを使う。Git worktreeの外側を指す`.git`は単一マウントでは参照できない。

## 4. kart版Isaac ROS CLIを導入

```bash
cd ~/workspaces/kart/tools/isaac-ros-cli
make build
sudo make install
sudo isaac-ros init docker
isaac-ros status
cd ../..
```

`.deb`はUbuntuホストで作る。`make install`は親ディレクトリにある対象`.deb`が一つであることを要求する。
複数ある場合は古い成果物を別ディレクトリへ保管してから実行する。
この操作はシステムのCLIをkart版へ置き換える。別プロジェクトとホストを共有する場合も同じCLIを使うことになる。
`ISAAC_ROS_WS`の設定と起動は`dev.sh`が担当するため、公式例のworkspaceを別途作る必要はない。

## 5. ホスト側jtopを準備

kartはホストとコンテナのjtopを固定版7.2.0＋リポジトリ内パッチへ揃える。
通常の`pip install -U jetson-stats`ではこの構成にならない。

ホストにuvがなければ、インストーラを保存して内容を確認してから導入する。

```bash
curl -LsSf https://astral.sh/uv/0.12.0/install.sh -o /tmp/kart-uv-install.sh
less /tmp/kart-uv-install.sh
sh /tmp/kart-uv-install.sh
export PATH="$HOME/.local/bin:$PATH"
uv --version
```

```bash
cd ~/workspaces/kart
sudo ./scripts/setup-jtop.sh "$(command -v uv)"
sudo usermod -aG jtop "$USER"
```

再度ログアウトしてログインし直し、次を確認する。

```bash
cd ~/workspaces/kart
systemctl is-active jtop.service
/usr/bin/python3 scripts/check-jtop.py
```

期待値はサービス`active`とチェック成功。スクリプトはホストのjtopを置き換えてサービスを再起動する。
ファン・電力モード・クロックの変更はこの手順に含めない。

## 6. 初回Dockerビルド・起動

```bash
cd ~/workspaces/kart
./scripts/dev.sh --build-local
```

初回はIsaac ROS → RealSense → kartの順にビルドされる。
インターネット接続が必要で、RealSenseとmapping依存の導入には時間・ディスク容量を使う。
完了するとコンテナ内のBashに入り、作業ディレクトリは`/workspaces/ros2_ws`になる。
ホストのkart全体が`/workspaces`へ読み書き可能でマウントされる。

通常の再入場はホストのkartルートから`./scripts/dev.sh`。
別ターミナルから同じコンテナへ入る場合:

```bash
docker exec -it kart_dev bash
```

Dockerfile変更後に再ビルドする場合は、走行・記録プロセスを終了してホストで
`docker stop kart_dev`を実行してから`./scripts/dev.sh --build-local`。
稼働中コンテナがあるとCLIは先にattachするため、ビルドが行われない場合がある。

## 7. コンテナ内で依存確認・ROSビルド

ここからは **kartコンテナ内** で実行する。

```bash
cd /workspaces/ros2_ws
printenv ROS_DISTRO ISAAC_ROS_WS RMW_IMPLEMENTATION
# lyrical / /workspaces/ros2_ws / rmw_cyclonedds_cpp
/usr/bin/python3 /workspaces/scripts/check-jtop.py
ros2 pkg prefix isaac_ros_jetson_stats
ros2 pkg prefix isaac_ros_cuvslam
ros2 pkg prefix realsense2_camera
/usr/bin/python3 -c 'import tensorrt; print(tensorrt.__version__)'
nvidia-smi

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select kart_system kart_vehicle kart_joy
colcon test-result --verbose
```

`rosdep`が未初期化と表示された場合だけ、コンテナ内で`sudo rosdep init`、`rosdep update`を実行して再試行する。
TensorRTのimportはPython依存確認、`nvidia-smi`はGPU可視性確認であり、実際のcuVSLAM・推論実行成功とは別。
ROSノードはシステムPythonで起動する。`/opt/inference`は推論用で、自動activateしない。

## 8. DualSense接続・初回profile

USB接続またはBluetoothペアリングはJetsonホスト側で行う。
Bluetoothの場合はDualSenseのCreate＋PSを長押ししてペアリングモードにし、UbuntuのBluetooth設定から接続する。
接続後、ホストとコンテナ双方で`ls -ln /dev/input/event*`を確認する。

コンテナ内:

```bash
mkdir -p /workspaces/config
# 初回だけ実行。既存profileを上書きしない。
ros2 run kart_joy kart_joy_config init --profile /workspaces/config/joy.yaml
ros2 launch kart_bringup vehicle.launch.py
```

既定ではbridge無効、STOPで起動する。別のコンテナ内ターミナルで:

```bash
ros2 topic echo /joy/ready
ros2 topic echo /joy
# 配置調整が必要なら上記echoを終えて:
ros2 run kart_joy kart_joy_config cui
```

入力権限が不足する場合は、ホストで`getent group input`と対象eventの数値GIDを確認し、
`docker/dockerargs`へ`--group-add 数値GID`を追加してコンテナを再作成する。
CLIは`/dev/input`をマウントするが、ホストユーザーの補助グループをすべて自動継承するわけではない。
詳細・GUI・入力確認は[kart_joy README](../../ros2_ws/src/kart_joy/README.md)を参照。

## 9. USB bridge・RealSenseの確認へ進む

USB bridgeを使う場合、まずホストとコンテナで`ls -l /dev/serial/by-id/`と
`ls -ln /dev/ttyACM*`を照合する。基板IDは実物から選び、最初のttyACMを自動選択しない。
コンテナ内にby-idがない場合は、対応を確認したttyACMパスを明示する。
デバイスが見えない場合は`docker/dockerargs`に対象の`--device=/dev/ttyACM0`等を追加して再作成する。
権限不足なら対象デバイスの数値GIDを`--group-add`で追加する。
追加した固定デバイスの抜き差し後のアクセスは別途確認する。

bridge無効のlaunchを終了し、車輪を浮かせた状態で、実物のパスに置き換えて起動する。

```bash
ros2 launch kart_bringup vehicle.launch.py \
  enable_bridge:=true device:=/dev/serial/by-id/実際の基板ID
```

操作・STOPの意味・基板firmware前提は[vehicle基盤](../vehicle.md)を参照する。
ROSのSTOPはHOST経路を解除して中立指令を送る。物理RC経路の停止や車体の停止確認を意味しない。

RealSenseはコンテナ内で`rs-enumerate-devices`を実行して認識を確認する。
続いて[公式RealSenseセットアップ](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/hardware_setup/sensors/realsense_setup.html)と
[kartのlocalization設定](../../ros2_ws/src/kart_bringup/config/localization/README.md)に従ってRGB・Depth・IMUを確認する。
vehicle launchだけではRealSense・VSLAMを起動しない。

## よくある失敗

| 症状 | 確認・対処 |
| --- | --- |
| Docker socketのpermission denied | ホストでdockerグループ追加後、ログインし直す。`sudo dev.sh`で回避しない |
| `dev.sh`でjtopチェック失敗 | 第5節の固定版・サービス・グループを確認。ホストとイメージの版を揃える |
| GPU指定で起動失敗 | Toolkit導入、runtime登録、Docker再起動を確認。ホストのJetPack版も確認 |
| apt/COPY/Git fetch失敗 | ネットワーク・空き容量・CLIの固定SHAを確認 |
| `Package ... not found` | colcon成功と`source install/setup.bash`を確認 |
| Joyがreadyにならない | profile、デバイス識別、入力権限、設定モード、複数台接続を確認 |
| 別PCからROS topicが見えない | kartのDDS既定はlocalhost限定。詳細は[ルートREADME](../../README.md#dds設定と変更の影響) |

## 公式Isaac ROS 5.0との対応

[公式Getting Started](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html)の
Jetson＋Docker経路を対象とする。Thor、venv、baremetal、未使用センサーは対象外。

| 公式項目 | kartでの対応 |
| --- | --- |
| JetPack・SDK・L4T確認 | 第1節。NVMeへのフラッシュ自体は実施済みの前提 |
| Jetson Storage Setup | NVMeへrootfsを直接配置するため移行不要。容量・配置のみ確認 |
| UTF-8 locale | 第1節で確認し、必要時のみ設定 |
| MAXN・最大クロック | 第2節に性能測定時の操作。機種ごとのmode IDを確認 |
| Docker・Toolkit・runtime・pre-flight | 第2節。Docker公式配布、NVIDIA既定runtime、GPU指定の起動確認 |
| workspace・ISAAC_ROS_WS | 第3節と`dev.sh`でkart配置へ置換 |
| ホストのIsaac ROS APT repository・公式CLI導入 | 第4節でローカルdebへ置換。ホストのIsaac ROS APT repository登録は不要 |
| ROS・Isaac ROS APT repository・rosdep | コンテナ基底の`Dockerfile.isaac_ros`で設定。ホストへROS導入不要 |
| activate・追加センサーイメージ | 第6節。repo管理のレイヤー設定を`dev.sh`から利用 |
| RealSense | 第2節のホストudevと第9節の認識確認・公式センサー手順へのリンク |
| PREEMPT_RT | 公式の推奨検討項目。kart初回セットアップでは導入しない。必要時に[公式RTカーネル手順](https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/SD/Kernel/RealTimeKernel.html)で対象L4Tとの対応を確認 |
| AI Agent Skills | 開発支援用の任意項目。kart実行に必須でないため導入しない |

JetPilotの`docs/setup_jetson.md`（2026-10-09参照）から、runtime設定、電力・クロック確認、
固定jtop、RealSense udevを参考にした。JetPilot固有のxrdp設定、Wi-Fiドングルdriver、
SilkyEvCam、CLIレイヤーや環境変数はkartへ転記していない。
この表は対応・省略理由の説明であり、実Jetsonでの完走を保証するものではない。

## 検証範囲・出典

2026-10-09: repo内のスクリプト・Dockerfile・CLI設定・launchとの整合、リンク先ファイルの存在、
Bashコードブロックの構文をローカルで確認。
JetPack導入直後の実Jetsonでの通し実行、Dockerビルド、ROS結合、GPU・USB・Bluetooth・車両動作は未確認。
実行時はJetPack/L4T、Toolkit版、kart/CLIのcommitと失敗ログを残す。

公式情報（2026-10-09参照）:

- [Isaac ROS 5.0 Getting Started](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html)
- [NVIDIA Container Toolkit導入・Docker設定](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- [uvインストール](https://docs.astral.sh/uv/getting-started/installation/)

kart固有の版・マウント・依存は[ルートREADME](../../README.md)、`packages.repos`、
`docker/`、`scripts/`が基準。ここでは公式CLIのAPT導入をkart版CLIのビルド・導入に置き換えている。
