# RAW時刻シフト取得

OpenEB HALでRAWを読んで、decoderのget_timestamp_shift()を取得するROS非依存C++17ツール。
録画を停止した後、EVS Docker内で実行します。

```bash
bash scripts/sensors/evs-raw-timing.sh --build /workspaces/record/session/camera.raw \
  > /workspaces/record/session/camera.raw.time_shift.json
```

`raw_sensor_shift_us`は、RAWを0始まりで読む時に差し引かれるセンサ時刻です。
RAW SPLITごとに各ファイルで実行すること。JSONは同じdirへ保存する。
SDK get_timestamp_shiftと同じtime shifting設定のreaderで学習する。
既にシフトを無効化したreaderの時刻へ二重加算しない。

```
sensor_us = raw_relative_us + raw_sensor_shift_us
ros_ns = sensor_us * 1000 + sensor_to_ros_offset_ns
```

offsetはbagの/event_camera/tensor_timingから対象windowの値を使う。
Tensor集積窓と最新イベント・header stampは異なるため窓の開始/終了フィールドを使う。
RAWのtimestamp reset/rolloverがある記録はepochを分離し、単一shiftで全記録を対応付けない。
空ファイル・unsupported decoderはエラー。CLIは必須RAWパスだけ、ROS topic/parameterなし。
OpenEB 5.2.0 APIのソースを確認済み、SDKビルド・実RAW処理は未確認。

## Python readerの不正時刻診断

時刻に負値やint64上限付近の値が混ざる場合は、通常の微小な逆行として整列しない。
SDKの`RawReader`（Python側リングバッファ）と`RawReaderBase`
（SDK batch経路）を同一RAW・time shifting無効で比較する。

```bash
/usr/bin/python3 /workspaces/tools/evs_raw_timing/diagnose_reader.py \
  /workspaces/record/e522/openeb_camera_19700101_091112_000000.raw
```

既定は各reader先頭3 batch、1 batch要求100000 events。`--batches`は1..100。
標準出力JSONにNumPy版・dtype/stride・時刻範囲・負値数・逆行数・最初の逆行付近を出す。
記録ファイルは変更しない。差が出ればbuffer経路、両方不正ならSDK/plugin/RAW自体を
追加調査する手掛かりになるが、これだけで原因を断定しない。後半の異常は先頭検査では分からない。
Python/SDK実行はEVSコンテナ内で行う。実RAWでの比較は未確認。
