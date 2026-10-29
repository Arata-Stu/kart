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

## 平坦路の平面制約

RCカーの平坦路走行を標準とし、地図生成・capture・localization・tuning・simの
VSLAMはodometry/SLAM双方の平面制約を既定ONにする。
公式offline工程はworkflowのground_constraint=trueをcfg_planarへ明示的に渡す。
これは推定中の制約であり、生成後の地面合わせとは別。既存地図は自動変更しない。

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

## Offline bag確認

evaluation.launch.pyはsensor/vehicle/controlを起動せず、左右画像・CameraInfo・/tf_staticだけ再生する。
/clockでlocalization/HDMap/RVizの時間を統一し、入力購読準備後に1倍速開始。
過去の/tf（動的TF）は使わずVSLAMがTFを所有する。校正TFが必要。
Notebookのみvisualize=true。RVizは公式cuVSLAM設定をベースとする。正解軌跡なしに精度値は主張しない。

## 固定参照地図の独立localization

`kart_visual_localization`はNVIDIA Isaac ROS release-5.0のコピーを基準とする。
ROS側のsrc/includeは上流commitとバイト一致を維持する。独自Pythonノード、照合周期、
有効期限、補正上限、診断、速度推定、センサ同期を追加しない。
変更は固定参照地図に不可欠なcuVSLAM内部パッチと別pkg/libとしてのビルド分離に限定する。
上流commit、パッチ、照合手順をpackageに保持し、差分を監査できるようにする。

標準LocalizeInMap成功後、読み取り専用の参照地図を保持し、ライブ軌跡とのUnionWithをしない。
以後の照合対象・地図可視化はその参照地図。参照へのキーフレーム追加、統計更新、PGOを禁止する。
VO内部とライブ軌跡グラフは上流どおり更新する。初期localization前は上流の一時SLAM動作を保つ。
地図照合の解をライブ軌跡へ適用し、平面制約・tailによる現在時刻への伝播を維持する。
固定参照取得後のsave_mapは失敗を返す。Resetは上流同様に初期状態へ戻る。

apt版を上書きしない。同じC++ namespace、node/topic/service名を保持するため、
apt版との同時起動や同一containerへの混載はしない。単体processで切り替える。
運用parameterは既存cuvslam.yamlと同値のfixed_localization.yamlを使う。
ソース一致と構文確認、ROS/CUDA結合、bagでの地図不変性・精度、実車確認を区別する。

固定マップのoffline評価も既存evaluation/replay/RVizを再利用する。
`vslam_backend=apt|fixed`は起動pkgの選択だけを担い、同一VSLAM containerには一方のみをロードする。
CLI/TUIの`--eval-localization fixed`は既存VSLAM＋VGL評価へ接続し、VSLAM pkgだけ固定参照版に差し替える。
VGL bundle/model検証、両ノードの再生待ち合わせ、TF所有・hint接続・RVizを既存構成と共通化する。
特徴点群の可視化は既存visualize引数の実行時overrideで有効化し、通常のノード既定を変更しない。

固定版ではenable_vgl=falseは「このlaunchがVGLを起動しない」の意味であり、
別起動VGLの不在とは解釈しない。localize_on_startup=false、enable_request_hint=trueを
YAMLから維持する。原点からの自動初期探索へ暗黙に変更しない。
別起動VGLとはuse_sim_time、map/base frame、画像topic、hint/request remapを一致させる。
