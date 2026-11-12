# 入力・収集操作の運用設定

`joy_manager.yaml`は`kart_system::JoyManagerNode`（ノード名`kart_joy_manager`）の実運用設定。
全parameterは同YAMLに列挙し、各parameterの詳細は[kart_system README](../../../kart_system/README.md)を参照する。
このディレクトリ自身にノード・入出力はない。

| parameter | 既定値 | 意味 |
|---|---|---|
| `speed_scale` | `0.3` | MANUALの前進・後退倍率、固定AUTO値の設定上限。車速ではない |
| `auto_fixed_throttle` | `-1.0` | `-1`で通常操作、`0..speed_scale`でAUTO固定スロットル＋手動ステア。STOP中のD-pad上下で運用値を0.001ずつ調整 |
| `auto_fixed_throttle_r2_gate` | `false` | 起動時固定bool。collect/environmentの同定収録で、□保持中のR2を固定駆動／中立惰行の切替に使う。固定値が0以上の場合だけ有効 |

R2ゲート有効時はAUTO入口の中立待機後にR2を解放し、押し直して固定駆動を開始する。
押下0.10／解除0.05は操作コアの定数。R2解放中もAUTO・手動ステア・新鮮な中立指令を維持する。
□解除・×・入力喪失等は従来のSTOP条件。中立は物理停止の確認ではない。

通常の起動はbringupのYAMLを使用し、明示したCLI/launch引数だけで上書きする。
`--auto-fixed-throttle-r2-gate`／`--no-auto-fixed-throttle-r2-gate`、または
`auto_fixed_throttle_r2_gate:=true|false`を使用する。未指定・空文字はYAML保持。
drive/E2EではR2ゲートを`false`に固定し、明示`true`を拒否する。
同定収録では車両指令とmodeを保存する標準record profileを使う。
速度推定用に左右両方のInfraを有効にする。どちらかが無効なら起動前に拒否し、自動で有効化しない。
操作状態は既存`/teleop/fixed_config`へ記録する。ROS結合・実車動作は別途確認が必要。
