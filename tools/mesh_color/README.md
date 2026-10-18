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

### metadata形式の差異

pose参照カメラは、keyframes_metadataとcamera_params_id_to_camera_paramsのIDを照合し、
pose・時刻・sensor_to_vehicle_transformが揃うカメラから選ぶ。
観測数が最多のカメラを選び、同数ならID順で固定する。選択IDはログと結果JSONに記録する。
左／右の意味は投影に不要なので、バージョンによって表記が異なるstereo_pairのキーに依存しない。
必ず同じカメラのposeと取付変換を組み合わせ、取付変換がなければidentity補完せず拒否する。

### メモリと強制終了の切り分け

終了コード-9はSIGKILLであり、OOMと断定はできない。ホストのkernelログで確認する。
ログにbag index読込・校正走査・PLY読込・RGB投影・保存の段階とpeak RSSを記録する。
投影は8192頂点単位、ray/triangle候補展開は既定32 ray単位に分割する。
CLIの --ray-batch は1〜256（既定32）。減らすと候補の一時メモリを抑えるが時間が増える。
meshの空間index自体のメモリは残るため、どのサイズでも動く保証ではない。
binary PLYの三角形は固定長memory mapを利用し、面のPython list複製を避ける。
入力SHA256は分割読込とし、PLY全体をhash用に再度メモリへ読み込まない。
すでに近いRGB観測で着色済みの頂点は、遠い観測の遮蔽計算を省略する。
頂点や面の削減は行わず、元meshの形状を維持する。

## GPU深度バッファ版（既定）

UIとCLIはGPU backendが既定。画面を作らずModernGL/EGLでmeshを描画し、
撮影カメラから見える最前面の深度を得る。meshは一度だけGPUへ転送する。
CPUで全頂点へのray交差を列挙する従来方式は --backend cpu の明示指定時だけ使用する。
EGL失敗・llvmpipe等のsoftware renderer検出時は停止し、CPUへ自動切替しない。
GPU device番号はCLIの --gpu-device（既定0）。CUDAのdevice番号との一致は保証しない。

- 深度はRGBの縦横2倍で描画し、near=0.05m、far=1000m。
- optical +Z前/+Y下、OpenGL上下反転、画像pixel centerを明示的に変換。
- 投影先3×3近傍の最小深度と頂点Zとの差2cm以内だけ着色。
  遮蔽輪郭からの色漏れを避けるため保守的に判定する。輪郭では未着色が増えることがある。
- GPUを使うのはmesh投影・深度描画による遮蔽処理。
  bag読込、画像歪み補正、頂点への色の割当、PLY保存はCPUで行う。
- ログにGPU vendor/renderer、1画像の投影・着色時間、peak RSSを記録。
  出力JSONにもbackend、renderer、supersamplingを保存する。
- 終了／例外時にGL資源を解放。UIの停止は既存のsubprocess停止を使用。

### 既存Notebookコンテナへの追加

Dockerfileのamd64依存へmodernglとlibegl1/libgl1を追加済み。
再buildを避けたい場合、既存のRGB着色依存が入ったコンテナ内で:

    sudo apt-get update
    sudo apt-get install -y libegl1 libgl1
    sudo uv pip install --python /usr/bin/python3 --break-system-packages 'moderngl==5.12.0'
    cd /workspaces
    python3 tools/mesh_color/gpu.py

GPU depth OKとGPU名が出れば描画の確認完了。その後UIを再起動して着色する。
この確認は実bagの座標整合性や着色品質の検証ではない。
EGLにはNVIDIA Container Toolkitのgraphics capabilityが必要。
既存isaac-ros-cliのrun_dev.pyはNVIDIA_DRIVER_CAPABILITIES=allを渡している。
別の起動経路ではcompute,utility,graphicsを指定してコンテナを再作成する。
コンテナ内で環境変数だけ変更しても、ホストのdriver library mountは追加されない。

出典:
- https://moderngl.readthedocs.io/en/latest/techniques/headless_ubuntu_18_server.html
- https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html

### 検証

portable testはprojection/pixel center、深度復元、前後面遮蔽、未観測頂点を検証。
GPUを備えたNotebookでの統合テスト:

    KART_TEST_EGL=1 python3 -m unittest discover -s tools/mesh_color -p test_gpu.py -v

開発環境では実EGLテストは未実行（既定skip）。実meshの処理時間・VRAM量は未測定。
