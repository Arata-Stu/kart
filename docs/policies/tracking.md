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
- odometryとTFの配信順序差には、実機で同じodom stampのTFを既定50 msまで待つ。
  最新TFへの代替やstampの付け直しで欠落を隠さない。待機後も全入力の鮮度を再検査し、
  取得失敗・期限切れ時は既存の無効通知とSTOP条件を維持する。sim timeでは待機しない。
- 静的設定はbringup/config/control、車体寸法・舵角の出典と校正状態を明記する。
  利用者が暫定の運用値を指定した場合は、実測値と区別して記録し、変更はSTOP後の再起動で反映する。

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

Map Studioのライン・区間速度・参照ライン・追従項目・速度PIDは、最新AUTO/MANUALと
有限で新鮮な速度情報がある場合に走行中も変更できる。STOP時の1秒停車確認は維持する。
地図hash・revision競合・ノード単位atomic set・適用後readback・timeout時の自動再送禁止を維持する。
車両補正・復帰と起動時固定項目の制約は変更しない。ライン切替の連続軌道生成は行わない。
PID動的変更ではI/Dをresetするが、throttle変化率制限の前回出力を保持する。

## 速度連動の先読み

Pure Pursuitは固定距離と、1 m/s・5 m/s時の距離を指定する線形補間を選択可能とする。
既定は固定距離。速度連動は同じodomのtwistから後輪軸へ剛体速度変換した前後実速度の絶対値を使う。
速度範囲外は端の距離で一定とし外挿しない。目標速度・throttle・位置差分を代用しない。
速度が無効な場合は参照無効を通知し、古い速度での継続や固定距離への暗黙復帰はしない。
方式・固定距離・低速距離・高速距離は既存の動的調整経路で変更し、全候補値の整合性をatomicに検査する。
可視化は制御計算に使用した実効先読み距離を表示する。短い閉路の既存拒否条件は維持する。

## 地図に保存した速度と暫定舵角（2026-10-26）

Centerlineの一定速度はlaneに保存し、Studio編集→保存→export→転送→参照ラインまで保持する。
既定は1.5 m/s、0は停止目標。ROS parameter -1は保存値に従い、0以上の明示値だけで上書きする。
Race/Customの点ごとの速度profileと停止点は維持し、新規生成の速度上限の既定を1.5 m/sとする。
実運用舵角は利用者の推定22〜27°から暫定22°を採用し、Pure Pursuitと速度正規化で同値にする。
ノード単独の未設定0ゲートは維持する。暫定値を実測校正済み・実機走行確認済みと扱わない。

## レース後の前進駐車

明示opt-inの`kart_parking`をHDMapとPure Pursuitの間に置く。通常周回は透過し、
レース終了後のstart要求で順方向の閉路＋局所進入経路へ切り替える。HDMap出力を
`planning/course_reference`へremapし、`planning/reference_line`のpublisherを1つにする。
P1〜P3の実座標・区画・入口・進入領域・進行方向はHDMapのparking_areasへ明示する。
壁は共通obstacles。走行領域はleft/right bound間と選択区画のmaneuver・polygonの和集合とし、閉路の中島を保持する。区画の輪郭を自動的に壁と扱わない。parking_drivableは旧形式の明示上書きに限定する。
保存・export・地図位置合わせ・Undoで全属性を保持し、入口の辺を通る進入を要求する。
export時に未定義のP1/P2/P3を注意表示し、駐車を行わない地図の書き出しは妨げない。
未知領域を自由空間と仮定せず、明示走行可能多角形の外側を禁止する。

局所探索は前進primitive、曲率制限、車体長方形の掃引検査、入口進行方向への非負進捗を要求する。
入口通過後は順方向に次周へ進み、後退やUターンによる逆走を生成しない。
後退recovery・固定AUTO throttle・動的HDMap編集との併用を拒否する。
探索失敗で直線を代用しない。探索上限による失敗と物理的不可能を区別する。

到達後は空参照とSTOP要求をラッチし、区画内の外形・位置・yaw・推定並進／角速度の
継続成立で駐車成功とする。目標外の停滞は失敗。位置ずれやAUTO再要求で自動再発進しない。
解除は新鮮なSTOPでのreset。AUTO／録画の開始は従来の操作者の責務を維持する。
実装の幾何テスト・理想二輪追従・ROS結合・実地図・実機停止の検証を区別する。
詳細と全設定は[kart_parking](../../ros2_ws/src/kart_parking/README.md)を参照。

駐車の領域結合はShapely（ROS/Docker: `python3-shapely`、ローカルStudio: 起動Python環境へ`pip install shapely`）を使用する。
閉路boundは包含する内外周、開路boundは両端を結んだ帯として検証する。接続していない進入範囲・交差boundは拒否する。
和集合の継ぎ目は壁にせず、残った中島・外周・共通obstaclesに対して車体と余裕を検査する。
生成結果を別の手描き属性として保存せず、保存したboundとparking属性から再構成する。ROS topic・parameterの追加はない。
