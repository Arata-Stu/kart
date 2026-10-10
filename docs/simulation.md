# 室内MuJoCoシミュレーション

`kart_sim`に車体・室内コース生成・ROS bridgeを実装。
rc-simのTT-02/CADモデルと任意センサを移植し、元checkoutには依存しない。

![大会コースの暫定MuJoCoモデル](assets/sim/minicar_2026.png)

## 配置と切替

- マップ正本: [maps/sim](../maps/sim/README.md)。JSONを追加して選択する。
- 実装・topic/parameter・起動: [kart_sim](../ros2_ws/src/kart_sim/README.md)。
- 運用設定: [bringup sim](../ros2_ws/src/kart_bringup/config/sim/README.md)。
- `scripts/sim.sh --list`、`scripts/sim.sh --map minicar_2026`。
- ROS不要のpreview/checkも同じJSONを使う。

PDF △3の24ページを骨格、22ページの写真と25/27/29/30/32/33ページを補助にした。
幅10.30m、板断面19×89mm、狭路の中央隙間0.10mなどを反映。
全体写真に合わせた赤白障壁・灰色床・黒幕・駐車枠と、仮の室内外壁/天井/照明を生成する。

**指定寸法を満たす部分と、補間した部分を混同しない。**
未記載箇所と参考値は設置でばらつくことが原資料にも明記されている。
モデルの奥行・斜辺・一部の端点は仮定であり、各JSONのassumptionsに保存している。
原資料の206cmと140cmの下側直線の高さは図だけから同時に拘束できず、現モデルは140cmを採用。
会場寸法、摩擦、凹凸高さ、室内の照度は未計測。
ショートカットは斜面を含め黒色。右上カーブは緑の多角形面に白い低摩擦面を重ね、
矩形配置で残っていた床の隙間を解消。白面は緑面より1mm高く、摩擦の優先度も高い。
赤白板は提示された図の色区間と継ぎ目で分割し、直線接合部に4mmの仮の隙間を作る。
接合点の黒支柱・灰色固定具は近似寸法。46個の正確な支持具配置は未再現。
ステージライト、動的矢印信号、柔らかい素材の変形は未再現。

駐車枠の外側3辺の壁を残し、コース側開口には幅5cmの白い床線を追加。
枠内の色付きテープ矩形とP1/P2/P3も写真に合わせて緑・赤・青で描く。
文字・床線は接触なし。文字の大きさ・字体はテープ線による近似。

![駐車枠の白線と文字](assets/sim/parking.png)
![板の接合部](assets/sim/board_joints.png)

## 車体・TF

base_linkはkartの後輪軸中心。rc-simのchassis中心をそのままROS原点にしない。
位置とtwistの原点を変換し、rear_axleはbase_linkにidentityで接続する。
カメラmountはkart_bringup/config/vehicle/transforms.yamlの暫定値を反映するが実測校正ではない。
仮想カメラは理想pinhole。実機D455/EVSの内部処理・ノイズ・露出特性は再現していない。
カメラ光学中心は床から133mmの仮値をユーザーが承認した。
後輪軸高さ33mmを仮定し、base_link→camera_linkのzを30mmから100mmへ更新。
画像だけ車体を隠す方法ではなく、描画とTFを同じ取り付け位置へ修正した。
フレーム前端のメッシュはYMax=112.5mmの旧版から85.5mm版へ再export。
フレーム上面は床から70mmへ近似（ユーザー情報60〜70mm）、
支柱45mmを保持し、簡略ローワーデッキ厚みを24mmから仮の9mmへ変更。
カメラ/IMUの配置と内部値はassets/d455.json。実機校正値を別JSONへ保存して起動時に指定できる。
左右mono8とRGBは424×240、各30〜90Hz。IMUは200Hz既定、姿勢・角速度・加速度は真値。
macOSでは`--preview --sensors`で表示されたlocalhost URLを開き、3画像と真値を確認できる。
VSLAMはLinux/NVIDIAのsim_vslam.launch.pyでVO/VIOを切り替える。
その際はsimのpublish_truth_tf=falseで推定TFとの競合を避ける。

## 検証（2026-10-10、macOS）

MuJoCo **3.3.7**（Docker固定版）/ NumPy 2.4.6の一時venvでsimの20件と取付TF設定の4件、計24件成功。
既存rc-sim環境のMuJoCo 3.14.0でも変更前の物理9件成功。

- 大会/空室マップのcompile、静止、有限値、MuJoCo警告なし。
- 前進・後退・左旋回、制動で速度が低下し後退へ切り替わらないこと。
- 室内壁の接触で車体が壁を突き抜けないこと。
- トンネル横断黒幕の撤去、駐車枠は外側3辺の壁あり・コース側開口。
  rayで入口から奥の壁まで通り、入口障壁がないことを検査。
  白線・文字・色枠の接触なしと、直線板の接合部の隙間を確認。
- 高さ1cm/幅5cmの四辺斜面と四隅の三角斜面をrayで高さ検査。
- 車体が東西南北の四方向から斜面・平台を通過し、MuJoCo警告がないこと。
- rear axle初期位置/reset、速度と位置差分、カメラ位置/光学座標。
- 指令stamp/steady受信の期限、mode許可/STOP、pause/resetでの指令無効化。
- stereo/RGBの30・60・90Hzを独立に設定し、左右同stamp、IMU200Hz、周期誤差1ms以内を検査。
- 仮想左右mono8/RGBの424×240実描画、左右の視差、バッファの独立性を確認。
- 中心からずらした主点と非等方fx/fyの既知点投影を検査し、CameraInfoと描画の一致を確認。
- camera/IMUのTFと描画/サイト位置・光学座標が一致すること。
- 静止後のレンズ高さ約133mm・印刷フレーム上面約70mm、実機/仮想mount設定の一致。
- 車体描画を有効にしたまま、左右/RGBのsegmentation画像に車体pixelがないことを確認。
- 静止IMUの加速度約9.81m/s²、自由落下約0、回転時の光学座標角速度を検査。
- MuJoCo全景PNGと左右/RGB画像を描画・目視確認（overviewだけ天井/照明器具を非表示）。
- localhostモニタのHTMLと真値JSONをHTTPで取得確認。
- shell構文、map一覧/切替smoke、Python構文、git diff --check。

再実行:

```bash
PYTHONPATH=ros2_ws/src/kart_sim /tmp/kart-sim-env/bin/python -m unittest discover -s ros2_ws/src/kart_sim/test -v
SIM_PYTHON=/tmp/kart-sim-env/bin/python scripts/sim.sh --check --map minicar_2026
PYTHONPATH=ros2_ws/src/kart_sim /tmp/kart-sim-env/bin/python ros2_ws/src/kart_sim/test/render_smoke.py
```

**未確認:** ROS 2 / colconのbuild・launch・DDS/TF通信、Linux Docker/JetsonのGL、実車との一致。
このホストにはROS 2/colconがなくDocker daemonも稼働していない。
物理テスト・画像取得成功はROS結合や実機性能の成功を示さない。
VSLAM接続launch/configを用意したが、cuVSLAMのload・VO/VIO追跡は未実行。
描画は実IRの反射/投光を再現せず、RGB/IMU内部extrinsicsも仮値。
撮影Hzはsim時刻基準。macOSで100ms分の初回描画が約0.36〜0.45秒を要したが、
GL初期化込みの単発smokeであり、定常性能・30〜90Hzの壁時計達成は未検証。
本入口はsimのみを起動し、AUTO要求・走行指令を自動発行しない。
