# scripts

普段使う入口を直下、補助的なshを役割別に配置します。コマンドはproject rootから実行します。

| 場所 | 用途 | 実行環境 |
|---|---|---|
| dev.sh | 開発Dockerへ入る（--evs/--no-evsの選択を保存） | Linuxホスト |
| update-isaac-ros-cli.sh | Isaac ROS CLI更新 | Linuxホスト |
| bringup.sh | 起動構成のTUI／CLI | ROSコンテナ |
| open-foxglove.sh | Foxgloveを開く | ホスト |
| jetson_max_performance.sh | Jetson性能設定・確認 | Jetsonホスト |
| build.sh | colcon build | ROSコンテナ |
| repos.sh | vcs import／status／pull | ホスト |
| setup/bluetooth.sh | コントローラーのBluetooth接続 | Linuxホスト／Docker |
| setup/setup-jtop.sh | jtopの初期導入 | Jetsonホスト |
| sensors/evs-bias.sh | EVS bias調整ツールのbuild／起動 | EVSコンテナ |
| tests/test-vehicle.sh | 車両制御のportableテスト | C++コンパイラのある環境 |

bringup.py、check-jtop.py、input-dockerargs.pyは入口から呼ぶ補助実装。
隠し設定ファイルはIsaac ROS CLIが場所を参照するため直下に保持します。

```bash
./scripts/dev.sh
bash scripts/build.sh --packages-up-to kart_bringup
bash scripts/sensors/evs-bias.sh --build
bash scripts/tests/test-vehicle.sh
```

`dev.sh --evs --build-local`でEVSを選ぶと、次回は`dev.sh`だけでEVS環境へ入れます。
選択はGit対象外の`.kart-dev-profile`へ保存。`--no-evs`で通常環境へ戻し、その選択を保存します。

`sensors/evs-raw-timing.sh --build FILE.raw`はRAW読込み時のtimestamp shiftを取得します（録画停止後、EVSコンテナ内）。

外部リポジトリの更新は `bash scripts/repos.sh pull`。対象はstatusと同じ
`tools`と`ros2_ws/src/sensing`内のリポジトリで、kart本体は含めません。
commit固定のdetached HEADは追従ブランチがないためpullで更新できません。
固定SHAの更新はpackages.reposで管理します。
