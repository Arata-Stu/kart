# Bluetooth操作の配置

Bluetooth daemon（bluetoothd）とペアリング情報はLinuxホストが所有する。
`bluetooth.sh`はホスト、またはkart開発用Docker内のBlueZ clientとして実行可能。
Dockerにはbluezを導入し、ホストの/run/dbusを共有する。コンテナ内にdaemonを二重起動しない。
D-Bus共有はホストsystem busへのアクセスを与える。read-only bindはファイル変更を防ぐだけで、
D-Bus API操作を読み取り専用に制限しない。開発用コンテナの信頼範囲として扱う。
コンテナからの操作と/dev/input経由のROS Joy受信は別経路として確認する。
Connected: yesはBluetooth接続状態の確認であり、入力鮮度や実車停止の保証ではない。
JetPilotのBlueZ導入・D-Bus共有を参考にした。実Docker/実Bluetoothは別途検証する。

ユーザー指定により、scripts/bluetooth.shはJetPilot版をそのままコピーする。
独自の自動判定・引数・接続確認を追加しない。選択したMACのremoveを含む元の動作を維持する。
