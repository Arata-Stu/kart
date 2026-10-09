# Notebook共通接続設定

`jetson_hosts.json`はUIとFoxglove起動スクリプトのJetson IPプリセットの正本。

| ID | IP |
|---|---|
| notebook（既定） | 10.42.0.1 |
| lan | 192.168.11.190 |
| usb | 192.168.55.1 |

IDは選択用の名前。ネットワーク自動判定・IP到達確認・SSH接続は行わない。
UI「接続設定」で選択し保存すると、bag探索・SCP pull・map/モデル送信に適用する。
ユーザー名・認証・保存先はIPプリセットと分けて既存接続設定を維持する。

```bash
./scripts/open-foxglove.sh                # 10.42.0.1:8765
./scripts/open-foxglove.sh --preset lan
./scripts/open-foxglove.sh --preset usb
./scripts/open-foxglove.sh --list
```

CLIは明示したIP/URLまたは`--preset` > `KART_FOXGLOVE_URL` > JSON既定値の順。
UIの保存済み接続とCLIの一時選択は別。定義を増やす場合は一意のid/host/labelを追加する。
