# scripts

固定操作の収録例: `vehicle_tuning.sh collect fixed_command:=true throttle:=0.10 steering:=0.0`。
STOP→AUTO中だけ保持し、MANUALは手動操作。解析には`analyze --mode auto`を指定する。

`vehicle_tuning.sh collect`は地図なしVSLAMを伴う手動記録、`check` / `analyze` / `tune`は
読み取り専用のbag解析・PID候補試算の入口です。
[記録要件・操作手順・検証範囲](../tools/vehicle_tuning/README.md)を参照してください。
録画開始と走行操作は独立し、候補設定を実車へ自動適用しません。

普段使う入口を直下、補助的なshを役割別に配置します。コマンドはproject rootから実行します。

| 場所 | 用途 | 実行環境 |
|---|---|---|
| dev.sh | 開発Dockerへ入る（--evs/--no-evsの選択を保存） | Linuxホスト |
| screen.sh | SSH用screenの作成／再接続／デタッチ | Linuxホスト |
| update-isaac-ros-cli.sh | Isaac ROS CLI更新 | Linuxホスト |
| webui.sh | Map Studio起動（ros2_wsから../scripts/webui.sh） | Notebook／ROSコンテナ |
| bringup.sh | 起動構成のTUI／CLI | ROSコンテナ |
| open-foxglove.sh | Foxgloveを開く | ホスト |
| jetson_max_performance.sh | Jetson性能設定・確認 | Jetsonホスト |
| jetson_display_mode.sh | GUI/CUIの起動モード切替（status/cui/gui、--nowで即時反映） | Jetsonホスト |
| build.sh | colcon build | ROSコンテナ |
| repos.sh | vcs import／status／pull | ホスト |
| setup/bluetooth.sh | コントローラーのBluetooth接続 | Linuxホスト／Docker |
| setup/setup-jtop.sh | jtopの初期導入 | Jetsonホスト |
| setup/foundationstereo.sh | 深度推定モデル取得・TensorRT engine生成 | 対象GPUのDocker内 |
| sensors/evs-bias.sh | EVS bias調整ツールのbuild／起動 | EVSコンテナ |
| tests/test-vehicle.sh | 車両制御のportableテスト | C++コンパイラのある環境 |

bringup.shの補助実装はlib/bringup.pyへ配置し、直下のTab補完候補を重複させません。
check-jtop.py、input-dockerargs.pyも入口から呼ぶ補助実装です。
隠し設定ファイルはIsaac ROS CLIが場所を参照するため直下に保持します。

```bash
./scripts/dev.sh
bash scripts/build.sh --packages-up-to kart_bringup
bash scripts/sensors/evs-bias.sh --build
bash scripts/tests/test-vehicle.sh
```

`dev.sh --evs --build-local`でEVSを選ぶと、次回は`dev.sh`だけでEVS環境へ入れます。
選択はGit対象外の`.kart-dev-profile`へ保存。`--no-evs`で通常環境へ戻し、その選択を保存します。

Docker起動時はrootのentrypoint拡張`50-kart-device-groups.sh`が、adminへ追加GIDを登録し、
Jetson GPUの既知のデバイス（`/dev/nvhost-{gpu,ctrl-gpu,power-gpu}`と
`/dev/nvgpu/igpu0/{power,ctrl}`）を存在する場合だけ`video:0660`へ設定します。
adminをvideoグループへ追加してから一般ユーザーへ移行するため、毎回の手動chmodは不要です。
デバイス全体やdebug/profiling用ノードの権限は変更しません。
このhookはイメージへCOPYするため、修正を同期しただけ／古いコンテナを再起動しただけでは反映されません。
初回反映はホストで`bash scripts/dev.sh --evs --build-local`を実行し、新しいイメージからコンテナを作成してください。
起動ログの`kart GPU permissions: /dev/nvgpu/igpu0/ctrl -> video 0660`でhookの実行を確認し、
コンテナ内の`python3 -c 'import ctypes; print(ctypes.CDLL("libcuda.so.1").cuInit(0))'`が`0`になることを確認します。
起動済みコンテナへのexec接続ではentrypointは再実行されません。

## SSHとDocker内のセッション維持

JetsonのGUIを停止するには、SSH接続したホストで`./scripts/jetson_display_mode.sh cui --now`。
次回起動もCUIになる。GUIへ戻すには`./scripts/jetson_display_mode.sh gui --now`。
`--now`なしでは次回起動設定だけを変更する。`status`は設定された起動モードを表示する。
CUIへ即時移行するとGUIセッションとその配下の作業は終了するため、SSH側から実行する。
このスクリプトはJetPilotの同名スクリプトを基にし、Jetsonホスト限定の実行ガードを追加した。
構文とヘルプは確認済み。実機のGUI/CUI切替は未確認。

SSH接続先のLinuxホストでscreenを使い、その中からDockerへ入り、Docker内でtmuxを起動します。
ホストにscreenがない場合は`sudo apt-get install screen`で導入します。tmuxはDocker imageに含まれます。

```bash
# Linuxホスト（SSH接続後、kartのproject root）
./scripts/screen.sh             # 既定名kart。既存sessionへ再接続、なければ作成
./scripts/dev.sh

# Docker内
tmux new-session -A -s kart -c /workspaces
```

tmuxのデタッチは`Ctrl-b`→`d`、screenは`Ctrl-a`→`d`。
screen内で`screen.sh`を再実行してもデタッチします。
SSH再接続後は`./scripts/screen.sh`で元の画面へ戻ります。
Dockerへ入り直した場合は同じtmuxコマンドで既存sessionへ再接続します。
screenとtmuxはSSH切断後も作業プロセスを維持しますが、Dockerの停止・再作成やホスト再起動では終了します。

`sensors/evs-raw-timing.sh --build FILE.raw`はRAW読込み時のtimestamp shiftを取得します（録画停止後、EVSコンテナ内）。

外部リポジトリの更新は `bash scripts/repos.sh pull`。対象は`packages.repos`記載のgit repoのみ。
kart本体や未登録repoは含めません。Python3/PyYAML（vcstoolの依存）を使います。
40桁commit固定はSKIPし、HEADとの差があれば表示します。固定SHAの変更・checkoutは自動で行いません。
branch指定はoriginからfetchしてfast-forwardのみ適用します。detached HEADでも、変更がなく
HEADが追従先の祖先であれば指定branchへ復帰します。別branch・未コミット変更・独自detached commit・分岐は停止し、reset/stashはしません。

`dev.sh`はinput/dialout/plugdevと接続中のevent/ttyACM/ttyUSBデバイスの所有GIDを
Docker補助グループへ追加します。反映はコンテナ作成時のため、既存コンテナへのattachでは変わりません。

fzfによる選択は上下キーで移動、文字入力で絞り込み、Enterで確定、Esc/Ctrl-Cで中止。
既定候補を先頭に表示します。run_nameは自由入力です。
fzfはDockerfile.kartで導入します。既存コンテナでは一時的に
`sudo apt-get install -y fzf`で追加可能。非対話の`--mode`指定ではfzfは不要です。

Dockerから渡した補助GIDは、image内のroot entrypoint extension
`docker/scripts/kart-device-groups.sh`がadminの所属グループへ登録してからgosuへ切り替えます。
このextensionを追加・変更した場合はイメージ再ビルドが必要です。

bringupの車両基板は既定で/dev/ttyACM0を使い、TUIで質問しません。
変更時は--device、基板なしの確認は--no-bridgeを指定できます。


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

## E2E TensorRT事前build

Jetsonのkartコンテナ内で`bash /workspaces/scripts/e2e_trt.sh`を実行する。`/workspaces/models`以下の転送済みONNX bundleをfzfで選択する。直接指定は`bash /workspaces/scripts/e2e_trt.sh /workspaces/models/<モデル名>`、探索先変更は`--models-root <dir>`。`KART_TRT_PYTHON`既定は`/opt/inference/bin/python`。ROS起動・車両指令publishは行わない。

FP32が既定。TensorRT 10では`--fp16`も指定可能、TensorRT 11以降ではこのフラグを拒否する。`--force`で再buildする。モデル配下の`model_<ONNX SHA先頭12桁>.plan`と同名`.json`にハッシュ・GPU UUID/名前・アーキテクチャ・CUDAドライバ・TensorRT版を保存する。`.build.log`と`.timings.json`には詳細ログと合成入力による速度計測を保存。warmup 500ms、計測3秒で、GPU compute/latencyの平均・中央値・P95が得られる場合に表示する。FP16の精度一致やrosbag走行性能はこの計測では検証しない。稼働中の推論を止めてから実行する。

E2E launchは有効なmanifestとハッシュ・ハードウェア一致、およびTensorRT deserializeとFP32入出力binding/形状検証が成功したengineだけを使用する。`force_engine_update=false`が既定で、true指定は拒否する。engineが未build・不適合なら案内を出して起動を停止し、自動buildしない。事前に`scripts/e2e_trt.sh`でbuildする。Notebookで作ったengineをJetsonへ流用しない。build失敗時は以前の成功engineを保持する。

## VSLAM地図だけでbag評価

bringup.shのevalでは「VSLAMのみ」（既定）と「VSLAM＋VGL」を選択する。VSLAMのみはMap Studio出力の`cuvslam_map/*.mdb`が非空の地図を探索し、VGLモデル・vgl_profile.jsonを要求しない。VGL併用は従来通りprepare_vgl_map済みbundleと実行GPU用モデルが必要。非対話CLIは`--eval-localization vslam|vgl`。

evaluation.launch.pyの`enable_vgl`既定はfalse、localization.launch.pyでは互換性のためtrue。false時は`model_dir`不要、VGL node/専用containerを起動せず、VSLAMのみ`localize_on_startup=true`・`enable_request_hint=false`で起動する。map_dirは地図ルートまたはcuvslam_map自体。TFのpublisherはVSLAMのみ、bagの古いTFは再生しない。HDMapと同じ地図座標系を選ぶ。初期探索範囲内に位置がない場合はlocalizationが成立しないことがあり、実際の一致をRVizで確認する。

## VGLモデルをkartで生成

`scripts/vgl_model.sh`は424×240 ALIKEDのソース取得・ONNX再export・公式TensorRT engine生成の入口。
`prepare --source-only` → `doctor --stage export` → `export --name 424x240` → `build --name 424x240`。
完成モデルは`models/vgl/424x240/runtime_models`でWeb UIから選択できる。
Pythonは`KART_VGL_PYTHON`（既定`/opt/inference/bin/python`）。既存成果物は上書きしない。
GPUごとにbuildが必要。詳細と依存・検証範囲は[tools/vgl](../tools/vgl/README.md)。

VGL engineは地図ごとの生成物ではない。地図追加・転送だけなら再build不要。
Jetsonでの初回buildと以後の検証は`bash scripts/vgl_build.sh`（既定424x240）だけで実行できる。
別名は`bash scripts/vgl_build.sh 424x240-v2`。ONNXとmanifestは先に配置する。
既存engineは両engineのdeserialize・ALIKED shape・ONNXハッシュを検査し、buildをスキップする。
初回検証時にGPU UUID/driver/TRT等とengineハッシュを記録し、以後の不一致は自動上書きせず停止する。

## sim.sh

`./scripts/sim.sh --map minicar_2026`で室内sim専用launch。`--list`はマップ一覧。
`--check`はROS不要の物理smoke、`--preview`は手動MuJoCo viewer。
`SIM_PYTHON`でMuJoCo導入済みinterpreterを指定する。詳細は[kart_sim](../ros2_ws/src/kart_sim/README.md)。

`--preview --sensors`は左右mono8/RGB（424×240）を取得し、表示されたlocalhost URLで
画像・真値姿勢・IMUを確認できる。`--stereo-hz`/`--rgb-hz`は30〜90（既定60/30）、
`--imu-hz`は30〜1000（既定200）、`--rig JSON`で校正を指定。
macOS GUIにはvenvのmjpythonをSIM_PYTHONへ指定する。ROS起動のHz/TF変更は
`ros2 launch kart_bringup sim.launch.py`の引数を使う。

### 固定軸の対話切替

`tuning_collect.launch.py fixed_command:=true fixed_axes:=throttle`でスロットルのみ固定できる。
`fixed_axes`の既定は`both`、他に`steering`。非固定軸はJoy操作。
別端末で`bash scripts/vehicle_tuning.sh control`を実行し、STOP中に固定対象・
steering/throttle/brake/duration_sを一括変更する。再起動や録画停止は不要。
AUTOへの移行と録画はJoyで操作する。変更はYAMLへ保存しない。
`teleop_timeout_s`の既定は0.15秒、片軸固定の入力途絶でSTOP要求。
条件は`/control/fixed_config`（std_msgs/msg/String、JSON、1Hzと変更時）へ記録する。
詳細と全parameterは[kart_control README](../ros2_ws/src/kart_control/README.md)。

## FoundationStereoの初回setup

NotebookのGPUコンテナで実行する。Dockerには公式installerのみを導入し、
モデル取得・TensorRT変換はDocker build中には行わない。

```bash
cd /workspaces
bash scripts/setup/foundationstereo.sh
# 高解像度を使う場合
bash scripts/setup/foundationstereo.sh --model-res high_res
```

既定は`low_res`（320x736）、`high_res`は576x960。
`ISAAC_ROS_WS`が未指定ならcheckoutの`ros2_ws`を設定する。
成果物はその配下の`isaac_ros_assets/models/foundationstereo/deployable_v2.0/`に保存される。
読み取り可能な非空engineがある場合は、ダウンロード・GPU初期化・利用条件確認をスキップする。
新規生成時はCUDA初期化を確認し、利用条件への同意後に公式installerを実行する。
非対話実行では同意済みの場合のみ`--eula`を指定する。apt installは自動実行しない。
存在チェックはengineのGPU/TensorRT互換性や破損検査を保証しない。
別GPUやTensorRT変更後は既存engineを別名に退避してから再実行する。
地図生成はdense有効時だけengineを事前確認し、不足時はVSLAM開始前にsetupコマンドを表示する。

公式手順: https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_dnn_stereo_depth/isaac_ros_foundationstereo/index.html

## 学習条件の一括比較

```bash
# Notebook Docker。validation省略時はtrainと同じdatasetを使う。
bash scripts/e2e_sweep.sh \
  --train /workspaces/e2e/datasets/2026-10-19/18-24-00/e522-rgb90-fixed0_1
```

既定はRGB ViT-S・全体fine-tuning、batch=32/64/128 × Smooth L1(β=0.1)/MSE × schedulerなし/Plateauの
12条件、各100 epoch。LR=1e-5、seed42、RAM ON、AMP auto、mode=steer_only。
Plateauはfactor0.5/patience2/min_lr1e-7（LRが小さい場合はそのLR以下）で検証lossを監視する。
比較指標とbest.pt選択は共通の検証MSE。特徴量キャッシュは全体学習なので使わない。
固定/max throttleのcheckpoint metadataは既定0.15/0.2。ONNX export・TensorRT build・走行は実行しない。

Pythonは`KART_E2E_PYTHON`（既定`/usr/bin/python3`）。既定repoはprojectの`python_ws/dinov3`、
重みは`weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth`。別の場所は`--repo`/`--weights`で指定する。
結果は`e2e/sweeps/YYYY-MM-DD/HH-MM-SS/`、`--output /path/new-directory`でも指定可能。

- `summary.md` / `summary.csv` / `summary.json`：条件別status、最良MSE・epoch、実所要時間。
- `epoch_budgets.csv`：5/10/20/30/50/75/100 epoch（実行上限以下）の各時点までの最良MSE・epoch・時間。
- `<条件名>/training/{best.pt,metrics.jsonl,timing.jsonl,run.json}`：checkpointと詳細。
- `<条件名>/train.log` / `command.json` / `status.json`：コマンド・ログ・状態。

最終epochで最良になる場合は、収束・最適epochが確定したとはみなさない。
最良epochが上限の最後10%にある条件をsummaryの`best_in_last_10_percent`で示す。
`elapsed_seconds`はtrainer開始から当該epochの検証完了まで（初期化/RAM読込/過去checkpoint保存を含むが、
Python importと当該epochのcheckpoint保存は含まない）。`wall_seconds`はprocess全体の実測。
同じtrain/valも許可する。その場合のMSEは同じデータに対する当てはまりの比較であることをsummaryに表示する。

```bash
# 条件を小さくする、または200epochまで調べる
bash scripts/e2e_sweep.sh --train /path/dataset --epochs 200 \
  --batches 32 64 --losses smooth_l1 mse --schedulers none plateau
# コマンド一覧とplanだけ作成する
bash scripts/e2e_sweep.sh --train /path/dataset --dry-run
# 表示された結果ディレクトリを指定して中断後に再開
bash scripts/e2e_sweep.sh --resume /path/to/sweep
# 学習せず再集計
bash scripts/e2e_sweep.sh --summarize /path/to/sweep
```

`--learning-rates 3e-6 1e-5 3e-5`でLR軸も追加可能。`--stream`で逐次読み込み。
`--validation`で別dataset群も指定可能。`--help`に全設定を表示する。
各条件は別processで逐次実行し、GPUメモリを解放してから次へ進む。OOM/失敗も記録して続行する。
再開時は成功/失敗/OOM確定済み条件を飛ばし、実行中だった条件は先頭から再実行する（optimizer状態の途中再開ではない）。
途中成果物はinterrupted日時ディレクトリへ退避する。同じ出力への上書き・同時実行は拒否する。
重み・manifest/metadataのhash、入力ファイルのsize/mtime、実装のhashが変わった場合は同じ比較への追記を拒否する。
既定12条件×100epochは、25秒/epochなら学習・検証だけで概算約8時間。実際はbatchで異なる。

### 高解像度で環境を記録する

`bash scripts/bringup.sh`のRGB / Infra選択は「解像度 × FPS」のプロファイル一覧。
通常のYAML既定（RGB 424×240×30、Infra 424×240×60）を最初に表示する。
用途は従来の`collect`を使用し、記録はR1開始・L1停止のまま。

```bash
bash scripts/bringup.sh --mode collect --rgb-profile 1280x720x30 --infra-profile 0
```

RGBだけ高解像度でbagへ記録する例。Infraも必要なら`--infra-profile 424x240x60`などを指定する。
候補は424×240（30/60/90）、640×480（30/60）、848×480（30/60/90）、
1280×720（15/30）、1280×800（30）とOFF（`0`）。
USB3 D455向けの候補であり、全stream・機種・firmwareでの対応保証ではない。
対応状況はRealSense起動ログの実際のstream profileで確認する。
高解像度ではUSB帯域とbag容量が増加する。
旧`--rgb-fps` / `--infra-fps`も利用できるが、同一streamのprofile指定との併用はエラー。
未指定の非対話起動はYAMLを保持する。

### 環境記録モード（Depth・IMU付き）

```bash
bash scripts/bringup.sh --mode environment
# カメラを手持ちで記録する場合は --no-bridge を追加可能
```

対話起動の「環境記録」でも選択できる。RGB=1280×720×30、左右InfraとDepth=848×480×30、
加速度=250Hz・gyro=200Hz、統合IMU=線形補間を既定とする。
RGB・左右Infra・Z16 Depth・各camera_info・IMU・TFをMCAPへ記録する。
Infraを有効にしてprofileを変更すると、同じセンサのDepthも同じprofileになる。
InfraをOFFにしてもDepthは848×480×30で継続する。
`--rgb-profile`等の既存選択は利用可能。`--evs`併用では従来どおりRAW連動記録も行う。
通常collectのDepth無効設定は変更しない。点群やRGBへのDepth整列は行わない。
R1開始・L1停止で記録し、起動だけでは録画・AUTOを開始しない。
Jetson上の実際の解像度・Hz・Depth欠落は起動ログと`ros2 bag info`で確認する。

### bringupのMap Studio動的調整

`bash scripts/bringup.sh` → `drive` → 通常のHDMap・地図・モデル選択 →
「Map Studio 動的調整」で有効／無効を選ぶ。地図は既存選択を再利用し、調整用の再選択は不要。
HDMapの転送フォルダーと選択したVSLAM/VGL地図の内容を照合し、調整用パスを自動設定する。
CLIでは通常のdrive引数へ`--tuning`を追加（`--no-tuning`で無効化、未指定はhdmap.yamlの既定値）。
`--dry-run`は接続せず、地図の一致・採用値と生成された起動コマンドを確認する。
操作手順は[Map Studio](../tools/app/README.md#hdmap自己位置を見ながら現場調整)を参照。
