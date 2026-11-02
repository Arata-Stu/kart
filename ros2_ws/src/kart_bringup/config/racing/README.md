# Racing shadow

`predictive_spliner.yaml`は`kart_racing::PredictiveSplinerNode`（node名predictive_spliner）の全独自parameter。
全既定値・意味・topic契約は[kart_racing README](../../../kart_racing/README.md)を参照。
`racing_shadow.launch.py`で明示起動する。既存drive/collectには自動追加しない。
独自launch引数なし。地図、認識、車両、制御は起動しない。
RGB/Infra/Depth入力frontendは未実装。入力Frenet観測は外部が供給し、出力はshadow専用。
実運用に接続する前に時刻付き軌道adapter、衝突監視、制動経路と実機検証が必要。
