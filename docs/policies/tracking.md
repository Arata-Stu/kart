# オフラインライン追従

- HDMap serverがlane IDとline種別を選択し、速度付きReferenceLineを配信する。
  これは将来のオンラインplannerへの参照。障害物回避の完了を意味しない。
- Pure Pursuitは独立C++ Componentとし、後輪軸中心で舵角rad・速度m/sを計算する。
  containerの作成は最上位、module launchはloadのみを担当する。
- ControlCommandの正規化actuator値と物理速度を混同しない。
  speed_controllerが校正した舵角の正規化と速度PIDを担当しauto/control_cmdへ出力する。
  ノード単独のgain既定値は0、最大舵角は未校正0。実機YAMLは承認された定常速度→
  スロットル表と暫定P補正を使い、計測の意味・出典・未計測域を明記する。
  表の線形補間をFFとし、表なしは従来kffを使う。最小点未満は仮定の(0,0)補間、
  最大点超は末尾値保持。目標速度上限は独立して検査する。
  表と出力増減制限は起動時固定。増減制限にもanti-windupを適用し、
  停止・異常・brake時は正スロットルの減少を遅らせない。
  simは実車表を暗黙に流用しない。未校正舵角の起動後ゲートは維持する。
- TF、pose、再局在診断、参照ラインの有効性を確認し、欠落時は速度0指令と理由を配信する。
  速度0指令は物理停止の確認ではない。走行モードの判断は既存mode manager/muxが担当する。
- 静的設定はbringup/config/control、車体寸法は校正値を使い、仮値で実機を駆動しない。

- PID係数/出力上限は実行中変更可能とする。不正値を拒否し、確定変更時にはI/Dをresetする。
- 速度はodom child frameから後輪軸へ剛体速度変換する。座標回転だけで取付位置の効果を省略しない。
- AUTO以外でI/D reset。AUTO中の異常は中立＋STOP要求を出し、有効な非AUTOを観測するまでラッチする。
- 負のPID出力はreverseに割り当てない。brake_limitの明示設定時のみbrakeへ渡し、ESC処理はbridgeへ集約する。

- line種別は起動時launch引数と実行中ROS parameterで選択できる。select_line topicも同じparameter経路へ集約する。
  変更確定後に即時再配信し、不正変更は以前の選択を保持する。切替時の軌道接続は別機能。

- 基本外形はrear_axle基準の向き付き長方形とする。暫定寸法はvehicle_geometry.mdを正本として記録。
  オフライン生成/exportでは点間の保守的な掃引領域を含めて境界・禁止領域を検査する。
  走行中のfootprint監視/回避とは区別する。

## 車両特性解析への公式offline pose利用

車両同定は`tools/vehicle_tuning/`へ集約し、公式mapping実装は変更しない。
公式`poses/odom_poses.tum`と元bagの指令・状態・静的TFを再利用できる。
SLAM補正／キーフレーム最適化後の軌跡を応答微分へ暗黙に流用しない。
poseのbody frameを明示し、後輪軸へ変換してから微分する。取付TFの仮定は精度保証ではない。
offline入力ではオンラインtracking診断を捏造せず、時刻・欠落・幾何的跳躍を検査して限界を記録する。
旋回解析は低速・低滑りの等価二輪モデルによる実効舵角の評価とし、
左右輪実舵角やタイヤ動特性の同定と区別する。候補の自動適用は行わない。


## 共通の局所復帰

明示opt-inの履歴操舵復帰はrule/E2E共通のmux前段に集約する。
責務・時刻・ESC・中断・検証範囲は[recovery.md](recovery.md)を正本とする。

## 走行中の現場調整

Map Studioのライン・区間速度・参照ライン・追従5項目・速度PIDは、最新AUTO/MANUALと
有限で新鮮な速度情報がある場合に走行中も変更できる。STOP時の1秒停車確認は維持する。
地図hash・revision競合・ノード単位atomic set・適用後readback・timeout時の自動再送禁止を維持する。
車両補正・復帰と起動時固定項目の制約は変更しない。ライン切替の連続軌道生成は行わない。
PID動的変更ではI/Dをresetするが、throttle変化率制限の前回出力を保持する。
