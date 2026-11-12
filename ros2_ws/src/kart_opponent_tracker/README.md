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

Foxglove用topic（bringupでは/racing配下）
- opponent_tracker出力 /detection_boxes（visualization_msgs/msg/ImageMarker、best effort/volatile/depth 1）。
  各decoder入力のbboxを撮影stampの緑のLINE_LISTとして、購読者がいる間に出版する。空検出は空の点列。
- opponent_tracker出力 /tracking_boxes（同型・QoS）。確認済みの追跡bboxを青のLINE_LISTで表示する。
  FoxgloveのImageパネルで画像`/racing/rgb`へAnnotationsとして重ね、Sync timestampsを有効にする。
  ID・class_id・scoreはRaw Messagesパネルの`/racing/tracks`で確認する。
  ImageMarkerはテキスト表示に使用せず、画像上の線枠と追跡データを分けて提供する。
- opponent_ground出力 /ground_markers（visualization_msgs/msg/Marker、sensor data QoS）。
  base_linkで推定した接地点を直径6 cmのSPHERE_LISTで表示する。空配列時DELETE、lifetime 0.3秒。
