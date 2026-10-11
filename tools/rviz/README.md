# RViz評価画面のdock配置

`generate_vslam_layout.py`はPyQt5でRViz互換のQMainWindow Stateを作り、
Window GeometryのYAMLを標準出力へ出す。ROS/OGREなしで同じQt5へ復元し、
左右の配置・左右画像と再投影tabの対応を検査する。画面サイズの基準は1600×900。
Viewsパネルなし。状態blobの直接編集を避けるための開発用toolで、
Dockerの実行依存やROSノードを追加するものではない。

開発用のPyQt5・PyYAML環境で実行する。

```bash
QT_QPA_PLATFORM=offscreen python3 tools/rviz/generate_vslam_layout.py > /tmp/vslam-window.yaml
```

出力を`ros2_ws/src/kart_bringup/config/sim/vslam.rviz`のWindow Geometryへ反映する。
既存のtopic・表示設定は保持する。
実際のRVizは`sim_vslam.launch.py`の`--fullscreen`で起動する。Escで通常windowへ戻る。
Qtの復元検査と、Linux RVizで実際に画像が表示されることは別の検証である。
