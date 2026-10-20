# 壁接触・スタックからの局所復帰

- 復帰はrule/E2Eの後段、既存command muxの前段へ置く。通常AUTO producerを
  `/recovery/input_cmd`へremapし、`/auto/control_cmd`のpublisherはrecovery一つにする。
  明示opt-in時のみ組み込み、録画開始やAUTO要求は送らない。
- 検出と復帰モジュールは地図・VSLAM・odom・TFへ依存しない。rule自体の既存localization監視は維持する。
- 基本は片側IRまたはRGBの短時間フロー＋IMU角速度。特徴不足はスタックと区別して中立＋STOP要求。
  EVSは既存red_blue_whiteイベント画像の空間支持付き活動率による補助veto。
  イベントの少なさだけでスタック／停止を断定しない。raw packet配信・新規カメラ占有は追加しない。
- 通常AUTOで採用した操舵要求を元stamp付きで履歴化し、同じ入力の再配信で履歴を水増ししない。
  スタック疑い開始時刻よりoffsetだけ前のspanを使い、中央値×倍率を上限で制限する。
  接触後の操舵を混ぜず、符号反転せず一定操舵・一定後退出力・一定時間の後退を行う。
- ESC出力履歴と遷移の正本はbridge。復帰はbrake要求→画像/角速度による静止傾向の継続確認→中立→reverse要求。
  保持時間の満了や中立指令を実測停止として扱わない。画像判定も校正済み停止計ではない。
- 後退終了後は中立と静止傾向の確認を挟み、復帰終了後に受信した新しい通常指令から再開する。
  速度PIDは復帰active中resetし、中立指令を出しつつ既存入力の健全性監視を維持する。
- 初回AUTOと異常復旧は新鮮なSTOPからarm。STOP/MANUAL/PROPOで途中中断。
  入力のstamp/steady期限・publisher競合・制御gap・時計逆行・過大な累積回転・復帰回数上限は中立＋STOP要求。
  復帰回数は通常前進の持続した進展を観測した場合のみresetする。
- 前進軌道を短く戻る近似であり、衝突後の滑り・実舵角との差・後方死角・順走を保証しない。
  EVS活動率や画像フローを物理速度／後退方向の確定観測として扱わない。
- 実機設定正本は`kart_bringup/config/recovery/`。simは同じ有効スイッチと
  `config/sim/drive_recovery.yaml`を使い、ROS時間とsteady受信期限を分離する。
  調整値は未校正候補。portable検証とROS結合・sim接触・実車検証を区別する。

- bringup.shのdrive/e2eで復帰無効/IR/RGB/EVS補助を選択する。CLIは`--recovery`、
  launch引数は`recovery`。空文字はYAML保持、offは今回のみ無効、profile指定は今回のみ有効。
  画像topicはrecovery/launch.yamlのprofilesを正本とし、選択で変更するnode値は
  sensor_profile/image_topicのみ。細かな復帰parameterやYAMLファイルをTUIから書き換えない。
  使用画像/角速度/EVSの有効性と、手動ステアの固定AUTOとの非互換を起動前に確認する。
