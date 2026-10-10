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
