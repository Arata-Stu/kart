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
