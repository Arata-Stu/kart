# 地図・走行時localizationの責務

- オフライン地図生成はNVIDIA公式create_map_offlineを使用する。
  HDMap編集用点群capture、VGL特徴地図生成、走行時localizationを別工程にする。
- VGL地図生成は元の保存画像/posesを再利用する。cuVSLAM地図と対応づけた別bundleへ保存し、
  元地図を上書きしない。モデル/設定/座標系が異なる地図を暗黙に混用しない。
- Jetsonの可視化はpose/odometry/Path/診断を基本とし、画像・特徴点表示は無効化する。
  offlineのHDMap用点群captureのみlandmark可視化を有効にする。
- 走行時の動的TFはVSLAMがmap→odom→base_linkを専有し、VGLはpose hintを供給する。
  カメラ取付TFは実校正値を外部供給し、仮のidentityで補わない。
- VGLの探索受付、VGL pose算出、VSLAM地図への再局在成功、走行許可は別状態。
  serviceのsuccessだけでAUTO開始や走行再開を判断しない。
- VGLは起動時探索/成功後停止/明示再要求を基本とする。Joy managerからの要求接続は別機能。
  大型モデルへのfallbackはしない。軽量モデルのONNXとGPU依存engineを区別する。
- 実運用値はkart_bringup/config/localizationへ集約し、launchは明示引数だけ上書きする。
  [実行契約・検証範囲](../../ros2_ws/src/kart_bringup/config/localization/README.md)を参照。

## launchの記述規約

localizationはIsaac ROS 5.0のmapping/localization系に合わせ、
`isaac_ros_launch_utils`と`isaac_ros_launch_utils.all_types`を使う。
`generate_launch_description`で`ArgumentContainer.add_arg(..., cli=True)`を宣言し、
評価後のcallbackで設定・資産検証、ComposableNode定義、container作成、
`load_composable_nodes`による読込みを分ける。ログは`lu.log_info`でlaunchに渡す。
静的parameterは引き続きYAMLに置き、引数既定値で重複上書きしない。
空文字は未指定、明示されたfalseは有効な上書きとして保持する。
上流のサンプルlaunch自体はincludeせず、kartのTF/可視化/モデル検査契約を適用する。

### containerの所有とmoduleの責務

- 最上位launchだけがcontainerを作成する。`create_*_container=false`は外部所有を意味する。
- `launch/modules/localization/`のmoduleは指定containerへのloadのみ。暗黙のcontainer生成はしない。
- container配置の正本は`config/localization/containers.json`。同一load先は生成を重複排除し、
  所有権/executor設定の矛盾は拒否する。
- 下位には検証済みparameter/remapを明示的に渡す。includeの引数scopeを分離する。
- 外部所有containerは停止・自動unload・既存Component置換をしない。ライフサイクルは所有側の責務。
- VSLAMを入れるcontainerにはmultithread executorが必要。別moduleを追加する際も、
  executor要件・TF配信者・processを共有する影響を最上位で判断する。
