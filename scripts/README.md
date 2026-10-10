# scripts

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

外部リポジトリの更新は `bash scripts/repos.sh pull`。対象はstatusと同じ
`tools`と`ros2_ws/src/sensing`内のリポジトリで、kart本体は含めません。
commit固定のdetached HEADは追従ブランチがないためpullで更新できません。
固定SHAの更新はpackages.reposで管理します。

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
