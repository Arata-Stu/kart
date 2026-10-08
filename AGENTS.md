# kartで作業するエージェントへの指示

- 原則として日本語で回答する。
- 作業開始時に [ROSパッケージ設計・記述ルール](docs/policies/ros_packages.md) を読む。
- ROSパッケージの追加・変更では、実装と同時に当該packageのREADMEを更新する。
  ノード名、入出力topicの既定名・型、パラメータの既定値・意味を必ず記載する。
- C++・Pythonともに`ament_cmake_auto`を使う。Pythonでは`ament_cmake_python`を併用し、
  原則として`setup.py`／`ament_python`構成を新設しない。
- 各pkgの`config/`には原則全parameterを記載する。実運用は`kart_bringup/config/`のモジュール別設定を使う。
  launchで静的値を重複定義せず、明示的な実行時指定だけで上書きする。
- aptで導入する外部ROS pkgは、使用ノードの設定一式をbringup/configへ明示する。
  差分だけのYAMLにせず、出典・対応バージョン・kartでの変更点・未記載項目の理由をコメントに残す。
- 実行スクリプトのshebang、実行権限、CMakeのインストール先を確認する。
- 既存構成を確認してから必要最小限の変更を行い、可能な範囲で検証する。
  実装済み・単体テスト済み・ROS結合確認済み・実機確認済みを区別して報告する。
- 設計方針を変更したときは、会話だけに留めず`docs/policies/`にも反映する。
