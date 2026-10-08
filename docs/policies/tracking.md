# オフラインライン追従

- HDMap serverがlane IDとline種別を選択し、速度付きReferenceLineを配信する。
  これは将来のオンラインplannerへの参照。障害物回避の完了を意味しない。
- Pure Pursuitは独立C++ Componentとし、後輪軸中心で舵角rad・速度m/sを計算する。
  containerの作成は最上位、module launchはloadのみを担当する。
- ControlCommandの正規化actuator値と物理速度を混同しない。
  speed_controllerが校正した舵角の正規化と速度PIDを担当しauto/control_cmdへ出力する。
  gain既定値は0、最大舵角は未校正0とし、仮の校正値では駆動しない。
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
