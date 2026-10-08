# 設定

Collector独自ROS parameterはなし。ROS共通`use_sim_time=true`のみ。
ジョブ入力はCLI `--job` のJSON。既定値は実運用の正本
`kart_bringup/config/mapping/workflow.json`。cuVSLAMは同ディレクトリの
`cuvslam.yaml`。CLIジョブは起動時固定で、動的parameter変更はない。
