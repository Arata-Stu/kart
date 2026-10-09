# kartセットアップ

実行するホストに合わせて手順を選ぶ。

| ホスト | 手順 | 前提 |
| --- | --- | --- |
| Jetson Orin Nano | [Jetsonセットアップ](jetson.md) | JetPack 7.2をNVMeへ直接フラッシュ済み |
| x86_64 PC | [x86_64 Ubuntuセットアップ](x86_64.md) | Ubuntu 24.04＋対応NVIDIA GPU |

両環境ともIsaac ROS 5.0 / ROS 2 Lyricalのkart Docker環境を使用する。
ダウンロードしたインストーラshは使用せず、Docker・ToolkitはAPTで導入する。

CLIの後日の更新は[更新手順](jetson.md#cliを後から更新する)を参照。
車両操作は[vehicle基盤](../vehicle.md)、地図編集は[Map Studio](../../tools/app/README.md)を参照。
