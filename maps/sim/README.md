# Simulation maps

`*.json`がkart_simのマップ資産。`scripts/sim.sh --list`で一覧、`--map NAME`で切替。
`minicar_2026`は大会PDFの暫定再構成、`indoor_empty`は車体検証用。

任意の`autodrive`はデータ取得用の閉ループwaypoint設定。
`waypoints`は後輪軸XY座標の配列（m、3点以上、末尾で先頭を重複しない）、
`speed_mps`既定0.45（0.05..1.5）、`lookahead_m`既定0.35（0.15..1.0）。
`minicar_2026`に通常周回の経路を配置し、ショートカット/駐車は通らない。
mapを編集した場合は経路も再検証する。controllerは姿勢真値を使用し、知覚/位置推定の評価結果とは独立。

schema_version=1、units=m。座標はコース左下付近を原点、+X右、+Y上、+Z上。
車両は自身の+X前、+Y左。spawn=[後輪軸x,後輪軸y,yaw rad]。
必須: name、room(bounds=[xmin,ymin,xmax,ymax], height)、spawn、wall_height、
wall_thickness、polylines(color,points)。色はred/white/green/blue/gray/black。
polylinesのheightを省略するとwall_height。隣接pointsごとに独立した接触可能な板を生成する。
同一直線でも接合位置にpointを追加する。色が変わる場所ではpolylineを分ける。
任意wall_joint_gap（既定0m）は各板の両端を半分ずつ短くする。大会は0.004mの仮値。
一直線の継ぎ目は指定幅となり、斜め・角の隙間形状は板の厚みと角度に依存する。
任意joint_supports（既定false）はpolylineの重複しない端点に黒い支柱と灰色固定具を追加する。
支柱は板の横に寄せた暫定形状（半径8mm、高さ110mm、接触あり）、固定具は表示のみ。
図中の黄色い丸は物体として描かない。parking壁と黒幕にはこのgap/support設定を適用しない。

任意: parking(x,y,length,width,color,walls,label)、black_walls(点列、暫定高さ1.33m)、
patches(name,boundsまたはpolygon,friction,color,height,priority)、ramp(bounds,height,slope_length,color)、
bumps(bounds,height,spacing)、gate(x,y,width,clearance)、start_lines(xの配列)。
patchesのpolygonは頂点順の凸多角形。heightは既定0.001m、priorityは既定1。
大会カーブは一体の緑polygonを敷き、白い低摩擦区間をheight=0.002m/priority=2で重ねる。
これにより床の隙間と同一高さの表示ちらつきを避け、白い領域の摩擦を優先する。
rampのcolorは省略時grayで、平台・四辺・四隅に共通。大会マップではblack。
parkingのwallsは省略時true。外側3辺だけ板を生成し、コース側は開口。
開口辺には幅5cmの白線を描く。内側に5cm insetした4辺の色付きテープ枠を追加。
falseなら板を生成せず、白線・色付き枠は残す。
labelは省略時文字なし、P1/P2/P3を指定でき、colorと同じ色の5cm幅の平面線で描く。
文字は左25cm位置、外形高さ約30cmの近似。外部フォントや画像ファイルに依存しない。
白線・色枠・文字は接触なしで仮想カメラにも写る。
rampのboundsは斜面を含む外形。四辺をslope_lengthだけ内側へ下げ、
中央はheight、周縁は0。四隅も同じ幅の三角斜面を接合する。
sourceとassumptionsは出典・採用した仮定を記録する。実測更新時も残す。

roomはコースの外側を囲う。表示用だけでなく壁は接触あり、照明/天井も生成する。
室内寸法は現在仮値。会場の壁位置をPDFから確定したとは扱わない。
全boardの接続点、マット/芝摩擦、凹凸形状は現場計測で差し替える。
任意オブジェクトの寸法不正はモデル生成時にエラーとなる。

JSON変更を直接反映する場合はlaunchのmap_dirでこのdirectoryを指定する。
未指定時はビルド時コピーのshare/kart_sim/mapsを使う。

## VSLAM比較用の模様付きmap

`minicar_2026_features`は`minicar_2026`と同じ形状・摩擦・spawn・Auto lap経路を持ち、
床・室内壁・赤白の仕切り・駐車場の外側壁に、不規則な複数サイズの濃淡模様を追加する。
室内壁/各板で異なるPNGを使い、単純な周期checkerboardは使わない。
床は模様を見やすくするため基調の明度も変更する。板の赤白などの色分けは保持する。
黒幕、色付きテープ、P1/P2/P3、芝・段差の特殊面には模様を付けない。
追加形状・接触はなく、特徴抽出の評価用の人工的な見た目であり、現場の再現ではない。
特徴が検出/追跡されることやVSLAM改善は、実際のbagで確認する。

```bash
scripts/sim.sh --map minicar_2026_features --preview --sensors
# ROS (source後): source側JSONを直接指定
MUJOCO_GL=egl ros2 launch kart_bringup sim.launch.py \
  map:=minicar_2026_features map_dir:=/workspaces/maps/sim \
  camera_enabled:=true monitor_enabled:=true viewer_enabled:=false \
  publish_truth_tf:=false record_dir:=/workspaces/record/sim_features
```

`visual_textures`（既定なし）は`geom`（既存world geom名）、`file`（このJSONからの相対PNG path）、
任意`rgba`（4成分、0..1）、`type`（既定2d、2d/cube）の配列。
床は2d、垂直壁はcubeを使い、壁の高さ方向にも模様を表示する。既存geomにtexture/materialだけを付ける。
`file`はmap読込み時に解決するので、installed mapも同じ構成で動く。
`textures/`はkart_simのmapsと一緒にインストールされる。
静的資産の再生成は`python3 tools/sim/generate_feature_map.py`（NumPy、固定seed 20261011）。
生成結果はmap JSONとPNGを上書きする。入力のminicar_2026.jsonは変更しない。
元mapの形状を変更した場合は比較用mapもこのコマンドで再生成する。
