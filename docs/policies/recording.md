# ROS bagとnative sensor記録の連携

- session directoryの所有者はkart_bag_manager。rosbag子プロセスの出力directory生成後にのみ
  native recorderへ絶対パスを渡す。driverからも同じパスが見えるmountを使用する。
- native RAWはOpenEB writer、MCAPはrosbag writerが別ファイルとして保存する。
  EventPacketやGPU tensorをnative RAWの代わりにrosbagへ重複保存しない。
- sensing repoにkart_interfaces依存を持ち込まず、manager側のadapterで標準ROS parameter/serviceに接続する。
  保存先設定の成功応答を待ってSTART。START/STOP/SPLITは非同期で順序を保つ。
- RAWの成功・失敗はbag/raw_diagnosticsで別途報告する。bag/status.recordingだけでRAW成功とは判定しない。
  RAW失敗によって既存rosbagを自動停止しない。
- RAWを単独で手動開始した場合と、bag managerが所有するRAW sessionを区別する。
  active RAWの保存先変更を拒否し、開始を拒否されたmanagerが手動writerをSTOPしない。
- 時間分割の正本はbag manager。連携時はdriverの独立分割timerを無効化する。
  同じ周期でもMCAP/RAWの分割境界が厳密同期するとは扱わない。
- ROS contextを生かしたままRAW STOP応答・rosbag終了処理を待つ。launchの終了猶予は
  bagのstop/terminate timeoutとRAW service timeoutに余裕を加える。
- 可視化画像はsensor側の独立した任意経路。packet配信OFFでも画像出力が可能。
  画像の時刻はpacket受信時刻への推定対応であり、hardware同期を保証しない。
