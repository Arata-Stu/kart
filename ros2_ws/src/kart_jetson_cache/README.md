# kart_jetson_cache

JetsonホストのLinux **clean file pagecacheだけ**を任意に解放する。
`kart_lap_manager`から周回ごとに呼ぶための独立ROSプロセスで、センサーcontainerへloadしない。
ROS側は非rootで動作し、権限のある固定操作はroot所有のホストhelperへUnix socketで要求する。
ホストhelperはROSや第三者Pythonライブラリを必要としない。

通常はOFF。Linuxは必要に応じてキャッシュを自動回収するため、これは原因切り分け用の選択肢である。
解放後は再読込みのI/O・CPU負荷が増え得る。
cuVSLAMのpose graph reindexエラーを解消する根拠は確認できていない。
[Linux公式drop_caches説明](https://docs.kernel.org/admin-guide/sysctl/vm.html#drop-caches)を参照。
CUDA/TensorRT/cuVSLAMプロセス内のヒープ・GPUメモリは解放せず、地図・モデル・engine・bag・ファイルも削除しない。

## ノード・インターフェース

- package: `kart_jetson_cache`、実行ファイル・既定node名: `jetson_cache_service`。
- Python独立プロセス。Composable pluginはなし。サービス待機とsocket I/Oをセンサー処理から分離する。
- 独自topicの入出力なし。標準`/rosout`・`/parameter_events`は記載省略。

| 方向 | 既定サービス名 | 型 | 意味 |
|---|---|---|---|
| 提供 | `/kart/cache/clear` | `std_srvs/srv/Trigger` | helperのkernel書込み完了でのみ`success=true`。busy・権限・timeout・helper不在はfalse |

`Trigger.success`はキャッシュ解放操作の完了であり、実際の解放量、VSLAM回復、車両の停止・安全を保証しない。
複数要求をキューに貯めない。ROS側の同時要求とhelper側の同時操作をそれぞれ拒否する。
timeout時は完了不明であり、同一周回では再送しない。
helperのkernel書込みがtimeoutを超えて続く場合、実終了まで次の要求もbusyになる。

## 全ROSパラメータ

起動時固定。package基準値は`config/jetson_cache.yaml`、実運用は
`kart_bringup/config/system/jetson_cache.yaml`。launch引数・動的parameter更新なし。

| 名前 | 既定値 | 型・意味 |
|---|---|---|
| `use_sim_time` | `false` | ROS共通bool。helperの制限・timeoutは常にmonotonic実時間 |
| `service_name` | `/kart/cache/clear` | string、絶対サービス名 |
| `socket_path` | `/run/kart-cache/cache.sock` | string、絶対Unix socketパス。root所有・0660・非root groupおよび書換不可の親directoryを検証 |
| `request_timeout_s` | `3.0` | float、0.1..30秒。socket通信の総待機期限 |

## Jetsonホストの準備

このセットアップは**Jetsonホスト上で明示的に一度実行**する。Docker内やnotepcでは実行しない。
コードコピー・サービス登録・起動を行うが、セットアップ時にキャッシュ解放操作は行わない。

```bash
sudo bash ros2_ws/src/kart_jetson_cache/host/install.sh
```

`/usr/local/libexec/kart-cache-helper/cache_helper.py`へroot所有0644のコピーを置く。
root-owned0755 directoryを用い、ユーザーが編集できるcheckoutをrootサービスから実行しない。
systemdの`kart-cache-helper.service`はrootで起動し、Jetson Linuxホストであることを毎起動時に検査する。
`/run/kart-cache`はroot:kart-cache0755、socketはroot:kart-cache0660。
directoryはホストの非所属ユーザーからもsocketのGIDを調べられるが、作成・削除・変更できない。
所属groupによるアクセスをkernelで検査するため、Dockerのsupplementary groupも利用できる。
systemdはdirectoryを再起動でも保持し、停止時はsocketだけ除去するため、Dockerのdirectory bindを維持できる。
公開ネットワークsocket・シェル実行・任意パス/値・sudoersは使用しない。

helperの固定操作は`/proc/sys/vm/drop_caches`へ`1\n`を書くだけ。
`2`/`3`によるdentry/inode解放はせず、`sync()`を強制しない。
録画のdirty pagesは通常のkernel writebackに残す。
helperの処理期限は実時間2秒、開始間隔は最低1秒。
これらはROS parameterではなくhelper実装の固定値である。

セットアップ後、`scripts/dev.sh`でdevコンテナを作り直す。
起動スクリプトは有効なhelperが存在するときだけdirectory bindとsocket GIDを追加する。
既存コンテナに単に`docker exec`するだけでは新しいmount/groupは反映されない。
ホスト上の非root ROSから直接利用する場合は、管理者が対象ユーザーを`kart-cache` groupへ追加し、ログインし直す。

停止・無効化:

```bash
sudo systemctl disable --now kart-cache-helper.service
```

## ビルド・確認

ROS環境のworkspace内で:

```bash
colcon build --symlink-install --packages-select kart_jetson_cache
source install/setup.bash
ros2 run kart_jetson_cache jetson_cache_service --ros-args --params-file \
  src/kart_bringup/config/system/jetson_cache.yaml
```

実操作を明示的に要求する場合:

```bash
ros2 service call /kart/cache/clear std_srvs/srv/Trigger '{}'
```

周回利用はbringupの地図走行設定「周回Jetsonキャッシュ解放」をONにする。
VGL周回補正とは独立してON/OFFできる。

PortableテストはJetson識別、固定kernel書込み、socket権限・protocol、timeout/busy重複防止、
例外・不正要求・応答を検証する。実際の`drop_caches`への書込みはmockし、ホストキャッシュは変更しない。
ROS結合・systemd起動・Docker実接続・Jetsonの解放量/負荷・実車走行は未確認。
