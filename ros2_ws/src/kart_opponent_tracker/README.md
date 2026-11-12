# kart_opponent_tracker

指定した単一クラスの2D検出をByteTrackで関連付ける。
`tracked_class_id`はノード既定`"0"`（相手車両専用モデル）。COCOのcarは`"2"`を指定する。
`tracking_enabled=false`では全クラスの検出表示だけを続け、ByteTrackを更新しない。
MITの[ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp)を
`docker/Dockerfile.kart` の `BYTETRACK_COMMIT` で固定し、イメージ作成時にビルドする。
ヘッダは `/usr/local/include/ByteTrack`、共有ライブラリは `/usr/local/lib/libbytetrack.so` に配置する。
ノード・実行ファイルは `opponent_tracker`、componentは `kart_opponent_tracker::OpponentTracker`。

|方向|既定topic|型|QoS|内容|
|---|---|---|---|---|
|入|/detections|vision_msgs/msg/Detection2DArray|best effort / volatile / depth 1|撮影時刻のbbox、class_id、score。空配列も1フレーム|
|出|/tracks|vision_msgs/msg/Detection2DArray|同上|入力と同じheader、確認済みの追跡bboxとscore、元のclass_idを維持。Detection2D.idに世代とID|
|出|/detection_boxes|visualization_msgs/msg/ImageMarker|同上|全クラスの生検出を緑のLINE_LISTで表示。購読者がいる間、入力ごとに出版|
|出|/tracking_boxes|visualization_msgs/msg/ImageMarker|同上|確認済みの対象クラスの追跡bboxを青のLINE_LISTで表示。購読者がいる間、入力ごとに出版|

名前は相対。bringupはnamespace `/racing` を使う。追跡有効時は空フレームもtrackerを進め、喪失中の予測bboxは出版しない。
受信途絶時は出版も停止する。IDはプロセス内の識別子。重複stampは無視し、時刻逆行・長い途絶・frame_id変更で世代を更新する。
track IDはステレオ距離・世界座標・実車IDを表さない。標準ROS topicは省略。独自service/actionなし。
指定class_idと完全一致しscore下限を満たす検出だけを1つのtrackerへ渡し、クラス間の関連付けを行わない。
`/detection_boxes`には追跡対象の選別前の全クラスのbboxを表示する。
追跡無効時も`/tracks`を入力と同じheaderの空配列、`/tracking_boxes`を空の点列として出版し、
前の追跡表示を消す。追跡有効時も対象を見失ったフレームでは空の点列を出版する。

|起動時固定parameter|既定|意味|
|---|---:|---|
|tracking_enabled|true|ByteTrackを有効にする。falseは検出表示のみ。起動後の変更不可|
|tracked_class_id|"0"|追跡対象のclass_id文字列。空文字不可。出力にも同じIDを保持|
|low_score|0.1|入力score下限。decoder側の閾値も合わせる|
|association_score|0.5|高信頼度の一次照合と低信頼度の二次照合の境界|
|new_track_score|0.6|新規track開始の最低score|
|match_threshold|0.8|一次割当てで許す最大コスト|
|lost_frames|30|見失ったtrackの保持フレーム数（処理した入力基準）|
|reset_gap_s|0.5|連続入力の撮影時刻差がこの秒数を超えるとリセット|

パッケージ基準値はconfig/tracker.yaml。ROS共通use_sim_timeはfalse。
ホストで `bash scripts/repos.sh import`、ByteTrack-cppを含むkartイメージを再ビルドし、そのコンテナ内で、
`bash scripts/build.sh --packages-up-to kart_bringup`。単体起動はsource後に
`ros2 run kart_opponent_tracker opponent_tracker --ros-args -r detections:=/racing/detections`。
RGBからの起動とengine準備は[bringup設定](../kart_bringup/config/racing/README.md)。

選別ロジックのportable C++テストは、ROSなしで次のように実行できる。
COCOのcarと他クラスの混在、検出のみの無効化、専用モデルのclass 0、score下限・不正値・空class設定を検証する。

```bash
c++ -std=c++17 -Wall -Wextra -Werror -Iros2_ws/src/kart_opponent_tracker/src \
  ros2_ws/src/kart_opponent_tracker/test/test_tracked_class.cpp -o /tmp/kart_test_tracked_class
/tmp/kart_test_tracked_class
```

同じテストは`BUILD_TESTING`有効時のCTestにも登録する。これはクラス選別の単体検証であり、
ROS component・ByteTrackの結合ビルド、実画像推論、Jetson・実車動作は別途確認が必要。

### 車体座標の路面接地点（任意）

Python実行ファイル・ノード名 `opponent_ground`。画像と同時刻のCameraInfo、
光学frame→base_linkのTF、車体座標での路面高さを使い、bbox下端中央の光線と水平路面の交点を推定する。
単体既定topic（型）: 入力 /tracks（vision_msgs/msg/Detection2DArray）、/camera_info
（sensor_msgs/msg/CameraInfo）、出力 /ground_points（kart_interfaces/msg/OpponentGroundPoints）。
画像関連はsensor data QoS（best effort/volatile/depth 5）、入力stamp完全一致時に出版。
TFはtf2標準の/tf・/tf_static。独自service/actionなし。bringupでは/racing/tracks、
/realsense/color/camera_info、/racing/ground_points。
起動時parameterはtarget_frame=base_link、ground_z_m=NaN（実測値必須）、
max_range_m=10.0（水平距離上限、m）。RGB rawのplumb_bob/rational_polynomial、
640×480、ビニング・ROIなしを想定。地平線・画像下端・距離外・TF欠落は推定対象外。
遮蔽、傾斜、取付姿勢のずれで誤差が出る。track IDを保持し、2D bboxの画像座標はそのまま残す。

投影診断の出力は相対`ground_diagnostics`（単体`/ground_diagnostics`、bringupでは
`/racing/ground_diagnostics`）、型`diagnostic_msgs/msg/DiagnosticArray`、
reliable / volatile / depth 1、steady clockで固定1Hz。追加の設定parameterはない。
最新tracks・CameraInfo・完全同期の受信鮮度をmonotonicで固定1秒とし、入力途絶時は成功を維持しない。
`status[].values`の`reason`は`waiting_tracks` / `tracks_stale` / `waiting_camera_info` /
`camera_info_stale` / `not_synchronized` / `invalid_stamp` / `invalid_frame` /
`invalid_image_shape` / `invalid_calibration` / `tf_unavailable` / `zero_tracks` /
`all_rejected` / `projected`。`status[].message`には日本語の理由を示す。
受信・同期・投影の累計、最新track/投影数、正確な撮影stamp、frame、受信経過秒も記録する。
`zero_tracks`は追跡0件であり推論失敗の断定ではない。`all_rejected`はtrackあり・投影全除外。
`projected`で地図表示できない場合は、mapまでのTFとFoxglove表示設定を確認する。
診断自体が来ない場合は投影ノードの未起動・起動エラー・接続も確認する。

可視化用ROS topic（bringupでは/racing配下。ImageMarkerは通常のFoxglove Bridgeでは非公開）
- opponent_tracker出力 /detection_boxes（visualization_msgs/msg/ImageMarker、best effort/volatile/depth 1）。
  各decoder入力のbboxを撮影stampの緑のLINE_LISTとして、購読者がいる間に出版する。空検出は空の点列。
- opponent_tracker出力 /tracking_boxes（同型・QoS）。確認済みの追跡bboxを青のLINE_LISTで表示する。
  画像を使うPC内デバッグ向け。通常のFoxgloveへは画像・ImageMarkerを送らない。
  ID・class_id・scoreはRaw Messagesパネルの`/racing/tracks`で確認する。
  ImageMarkerはテキスト表示に使用せず、画像上の線枠と追跡データを分けて提供する。
- opponent_ground出力 /ground_markers（visualization_msgs/msg/Marker、sensor data QoS）。
  base_linkで推定した接地点を直径6 cmのSPHERE_LISTで表示する。空配列時DELETE、lifetime 0.3秒。

### bbox付きJPEGデバッグ（任意）

Pythonノード・実行ファイル名`opponent_debug_image`。既定では起動せず、
bringupの`--perception-debug-image`または`ros2 launch kart_bringup racing_debug.launch.py`で明示起動する。
Python/OpenCVによる確認用の別プロセスとし、推論・車両制御から独立させる。
通常bringupのdebug ONと単独debug launchを重複起動しない。

|方向|単体既定topic|型|QoS|内容|
|---|---|---|---|---|
|入|/rgb|sensor_msgs/msg/Image|best effort / volatile / depth 1|640×480のrgb8/bgr8/mono8。bringupは認識と同じimage_topic（既定/realsense/color/image_raw）へremap|
|入|/detections|vision_msgs/msg/Detection2DArray|同上|全クラスの検出。空配列も実際の0件として扱う|
|入|/tracks|vision_msgs/msg/Detection2DArray|同上|対象車両の追跡。空配列も実際の0件として扱う|
|出|/debug_image/compressed|sensor_msgs/msg/CompressedImage|同上|緑の検出bbox、青の追跡bbox、クラス名/ID・score・追跡IDを描いたJPEG|

名前は相対。bringupはnamespace`/racing`を使用し、出力は`/racing/debug_image/compressed`。
独自service/actionなし。TF・CameraInfoは不要、ROS標準topicは省略する。
画像はmax_fpsで間引き、bboxは撮影stampとframeが両方一致した結果だけを使う。
到着順が逆でも有界cacheで待つ。待ち時間内に結果が来なければ`MISSING`と表示し、
実際の0件`DETECTIONS: 0` / `CAR TRACKS: 0`と区別する。検出class名はモデルprofileから設定する。
実画像のheaderは撮影stamp/frameを保持する。入力不正・未受信・途絶時は
`NO SOURCE IMAGE`と明記した待機画像にし、古い撮影画像・bboxを再配信しない。
待機画像は現在のROS stamp・空frameで、カメラ撮影画像や推論成功を表さない。
出力購読者がいなければ画像変換・JPEG圧縮を行わない。
購読中でも画像rate、cache枚数・寿命、抽出・描画数を制限する。

全parameterの基準値は`config/debug_image.yaml`、実運用は`kart_bringup/config/racing/debug_image.yaml`。
以下の独自parameterは起動時固定。ROS共通`use_sim_time=false`もYAMLに明記する。

|parameter|既定|意味・範囲|
|---|---:|---|
|max_fps|5.0|入力画像の採用・JPEG出力の上限Hz。wall clock、0.1..30|
|jpeg_quality|70|JPEG品質、整数1..100|
|match_wait_s|0.20|同一撮影stamp/frameの検出・追跡を待つ秒数、0..2|
|cache_size|8|採用画像・各結果channelの上限件数、整数1..32|
|cache_ttl_s|1.0|結果cacheの有効秒数、0.05..10、match_wait_s以上|
|max_boxes|100|各結果配列から抽出・描画するbbox上限、整数1..1000。元の総件数も画像に表示|
|image_timeout_s|1.0|入力途絶・採用画像の期限とする秒数、0.05..10、match_wait_s以上|
|class_names|['']|class IDに対応する名前の文字列配列。空要素・範囲外はclass ID表示。bringupはCOCO/専用モデルの名前一覧を設定|

FoxgloveのImageパネルでJPEG topicだけを選ぶ。別annotation・Sync timestamps・RectifyをOFFにし、
Calibrationを未選択にすればCameraInfoは不要。元RGB・CameraInfo・ImageMarkerはBridgeへ公開しない。
ホストでの検証（OpenCV・NumPy・pytestが必要）:

```bash
PYTHONPATH=ros2_ws/src/kart_opponent_tracker python3 -m pytest \
  ros2_ws/src/kart_opponent_tracker/test/test_debug_image.py \
  ros2_ws/src/kart_opponent_tracker/test/test_ground_status.py
```

ROS結合・Jetsonの処理負荷・実画像推論の描画・実車精度は別途確認が必要。
