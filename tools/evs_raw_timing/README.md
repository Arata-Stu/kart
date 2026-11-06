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


## EVS / action / RGBの時間軸（2026-11-06確認）

- RAW reader: `do_time_shifting=False`のデコード時刻（us）。オンラインdecoderと周回原点が一致する保証はない。
- TensorTimingのwindow_start/end_sensor_us、latest_event_sensor_us: オンラインdecoderのセンサー時刻（us）。
- Tensor header / tensor_stamp_ros_ns: ROS時刻（ns）。窓終了ではない。最新適用packetのROS時刻と
  予定出版時刻の小さい方を基に、前回headerより1ns以上進める。
- packet header: driverがpending packetを作成した際の`now()`。露光時刻・ハードウェア同期時刻ではない。
- sensor_to_ros_offset_ns: packet header ROS時刻 − 最新decoded eventセンサー時刻×1000。
  受信由来の推定で、パケットごとに変わり得る。
- action（/teleop/control_cmd）header: Joy managerが指令を出版するときのROS `now()`。
  モードheaderもROS `now()`。実車応答の時刻ではない。
- RGB: 比較ツールではImage.header.stampをROS時間軸の値として使用する。
  実機ドライバの時計変換・露光との遅延は別途確認が必要。

manifestのepoch_offsets_usをΔとすると:

```
raw_decoder_us = online_sensor_us + Δ
ros_ns = (raw_decoder_us - Δ) * 1000 + sensor_to_ros_offset_ns
```

RAWをshift有効で読む別経路では、まずSDK timestamp shiftを足してshift無効の座標へ戻す。
SDK shiftとΔは異なる値であり、二重加算しない。既存RAW shiftツールのsensor_usという名前は
RAW decoder側の座標を意味し、オンライン時刻への一致を保証しない。
学習教師はtensor_stamp_ros_ns以前の最新指令を選び、既定100ms以内。
新しいsamples.jsonlにはcommand_stamp_ns / command_age_nsも保存する。
`--clock header`を比較の基準とし、bag時計はcommand/modeにbag受信時刻を使う別方式として扱う。
固定AUTO設定のStringにはheaderがないためbag受信時刻を使う。

最初の検証目標はoffsetレベルの整合。窓の重なり、処理・露光・受信遅延をゼロと仮定しない。
RGBとEVSに共通して写る物体を左右に数回動かし、開始・中間・終了で位置と動きの対応を見る。
一定のずれはoffset、増大するずれはclock drift、途中の飛びはreset等として切り分ける。
視点・解像度・40ms集積の差を考慮し、これをマイクロ秒同期や空間校正の証明としない。


### RGB / EVS並列表示

```
/workspaces/.venv/evs-raw/bin/python /workspaces/tools/evs_raw_timing/compare_rgb.py BAG DATASET /workspaces/e2e/rgb_evs_check.html
```

BAGはRGBとEVSを同時記録したディレクトリ、DATASETはそのbagから抽出したsamples.jsonlを含む
ディレクトリ。NumPy、Pillow、rosbag2_py、sensor_msgsが必要。
RGB topic既定/realsense/color/image_raw、比較12点、最大header差100ms。
`--samples`、`--rgb-topic`、`--max-delta-ms`で変更可能。
自己完結HTMLを出力し、RGBなし・範囲内候補なしを明示する。画像はsensor_msgs/Imageの
rgb8/bgr8/rgba8/bgra8/mono8に対応。入力は変更せず、出力の上書きは禁止。
ROSノードではないのでCMake install対象外。Pythonで直接実行する。
