# EVS bias配置先

SilkyEvCam用の運用`.bias`ファイルをここへ配置します。初期ファイルは配布しません。
bringup.shのEVS選択後、このディレクトリを再帰探索し一覧表示します。
JSONはdriver読み込み対象外。カメラ個体・照明・調整日が分かるファイル名を推奨。
調整ツールはここへstartup/custom/autotuned.biasを保存します。
既存の同名ファイルは上書きされるため、運用版は別名で保管してください。
`.bias`はGit追跡可能です。ローカル限定の場合は.git/info/excludeへ追加してください。
