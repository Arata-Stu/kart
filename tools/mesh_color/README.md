# 公式meshのRGB着色（Notebook後処理）

Map Studio「占有地図・3Dメッシュ」→「3. meshをRGB着色」から実行する。
元bag欄は空なら地図作成時のパス。移動した場合はrecordルートからの相対パスを入力。
metadata.yamlのSHA256が生成時と一致するbagのみを許可する。
成功後「3D重ね表示」を開くと着色meshが選択される。点群・HDMapは従来どおり。
PLYをダウンロードして外部viewerでも確認できる。CPU互換表示は面平均色・間引き表示。

## 入力と座標契約

- 現在のdense出力のmesh_before_adjustment.plyとmap_frames/rectified/frames_meta.jsonを同じrunから読む。
- 公式metadataから T_world_base = T_world_camera * inverse(T_base_camera) を得る。
  bagの静的TFで T_world_rgb = T_world_base * T_base_rgb_optical を計算する。
  base frameは元jobのworkflow.overrides.base_frame。名前からframeを推測しない。
- metadata形式の出典: https://github.com/nvidia-isaac/pyCuSFM/blob/main/docs/tutorial.md
  axis-angleはdegree、timestampはmicroseconds、camera_to_worldはcamera→world。
- 元bagの /realsense/color/image_raw (sensor_msgs/msg/Image)、
  /realsense/color/camera_info (sensor_msgs/msg/CameraInfo)、/tf_staticを使用。
  RGB8/BGR8、固定CameraInfo、plumb_bob/rational_polynomial歪みに対応。
  圧縮画像、魚眼、binning/crop、動的取付TFは未対応。歪み補正して投影する。
- 公式キーフレームとRGBの時刻差20ms以内だけ採用。時刻シフトや別軌跡の代入はしない。
- 最大120画像、最小間隔0.5秒。meshとのレイ交差で手前の面を確認し、
  交差距離と頂点距離の差2cm以内を可視とする。最も近距離の観測色を採用。
  未観測頂点は灰色。時刻差・可視判定の許容誤差があり、境界の精度保証ではない。
- 元PLYの座標・面index・法線等を維持し、RGBだけ追加／置換する。
  color_runs/<uuid>/mesh_rgb.plyと報告JSONへ保存。公式出力とHDMapは変更しない。
  全頂点が未着色なら失敗として公開しない。job停止・失敗時は未公開ディレクトリを削除する。
  mesh再生成でdense_sourceが変わると旧着色を表示しない。

## 依存・CLI

Notebook用amd64 Dockerにtrimesh、rtree、plyfile、opencv-python-headless、scipyを追加。
Jetson側への追加はしない。rosbagsは既存依存。
既存NotebookコンテナでDocker全体を再buildせず試す場合:

    sudo uv pip install --python /usr/bin/python3 --break-system-packages 'trimesh>=4.4,<5' 'rtree>=1.3,<2' 'plyfile>=1.1,<2' 'opencv-python-headless>=4.10,<5' 'scipy>=1.11,<2'

CLI（パスを実データに置き換える）:

    python3 tools/mesh_color/run.py --bag /path/to/bag --mesh /path/to/dense_run/mesh_before_adjustment.ply --metadata /path/to/dense_run/map_frames/rectified/frames_meta.json --base-frame camera_link --output /tmp/mesh_rgb.ply

画像topicの --image、CameraInfoの --info、--max-framesも指定可能。
出力横の.ply.jsonに入力hash、座標・TF、着色率、画像数、投影条件を記録する。

## 検証と限界

    python3 -m unittest discover -s tools/mesh_color

合成RGBの実rosbag2を読み、着色PLY出力・座標と面の保持を検証。
前後二枚の三角形で遮蔽による不着色、画像投影、公式axis-angleを検証。
実bagの公式metadataとの結合、Docker build、実環境の処理時間・着色品質は未確認。
頂点カラーなので粗いmeshでは細かい模様が失われる。
テクスチャUV生成、動体除去、露出補正、RGBによる幾何補正は行わない。
