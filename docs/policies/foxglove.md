# Foxglove公開範囲

Foxglove Bridgeはhostごとに独立プロセスを一度起動する。車両component containerに混在させない。
静的設定の正本はkart_bringup/config/visualization/foxglove.yaml。
地図・pose/path・TF・制御状態・録画状態・jtopを明示allowlistで公開する。
画像・特徴点・tensorは公開せず、connectionGraph経由の列挙も無効にする。
書込みはlocalization/pose_hintとVGL triggerのみ。車両指令・モード・録画・TFの書込み、
パラメータ変更、任意service呼出し、asset取得は公開しない。
初期姿勢はmap座標のbase_link姿勢としてcuVSLAMの再localizationへ入力する。
TFは引き続きcuVSLAMが所有する。VGL triggerのpose値は初期姿勢として使われない。

Pure Pursuitの`/control/tracking_reference`・`/control/tracking_markers`も読み取り公開する。
参照点は制御計算から出力し、UIで再計算しない。設定lookaheadの経路長と車両からの
直線距離を区別する。無効通知でmarkerを消し、lifetimeで残像を制限する。
