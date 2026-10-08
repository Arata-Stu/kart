# ROSパッケージ設計・記述ルール

kartのROSパッケージを追加・変更するすべてのセッションで参照する。
対象は`ros2_ws/src/`配下の自作package。外部リポジトリにはそのプロジェクトの規約を優先する。
外部packageをkartから利用する際の設定管理には、本書4節のルールを適用する。
本書は継続的な設計ルール、[vehicle.md](../vehicle.md)は現在の車両構成・動作仕様を扱う。

## 1. C++・Pythonともにament_cmake_autoを使う

- `CMakeLists.txt`をビルド・インストール定義の入口にする。
- `find_package(ament_cmake_auto REQUIRED)`、`ament_auto_find_build_dependencies()`、
  `ament_auto_package()`を基本構成とする。
- package.xmlのbuild typeは`ament_cmake`。依存はpackage.xmlに明示し、必要なCMake依存解決も行う。
- Pythonのみのpackageも同じ方針とし、`ament_cmake_python`を併用する。
  原則として`setup.py`や`ament_python`の別系統を新設しない。
- C++ノードはComposable Nodeを基本とし、単体実行ファイルも提供する。
  Component化が適さない場合は、その理由と起動方法をREADMEに記載する。
- msg定義は`kart_interfaces`へ集約する。生成処理には`rosidl_default_generators`を使う。

既存の例: [kart_joy/CMakeLists.txt](../../ros2_ws/src/kart_joy/CMakeLists.txt)。

### Python packageの基本形

```text
example_package/
├── CMakeLists.txt
├── package.xml
├── README.md
├── example_tools/
│   ├── __init__.py
│   └── app.py
└── scripts/
    └── example_node
```

```cmake
cmake_minimum_required(VERSION 3.16)
project(example_package)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(example_tools)
install(PROGRAMS scripts/example_node DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_auto_package()
```

package.xmlには`ament_cmake_auto`・`ament_cmake_python`のbuildtool依存と、使用する`rclpy`・msg等の依存を記載する。
実行用wrapperは以下のように薄く保ち、本体をPython moduleへ置く。

```python
#!/usr/bin/env python3
from example_tools.app import main

if __name__ == "__main__":
    main()
```

ROSノードはROS環境のPython・依存を利用する。推論用uv環境を暗黙に混用しない。
実行環境を分ける必要がある場合は、interpreterと起動方法をREADMEへ明記する。

## 2. 実行可能ファイルと配置を確認する

`ros2 run`やlaunchの`Node`で起動するスクリプトには、次を揃える。

1. 適切なshebang。
2. ソースの実行権限（例: `chmod +x scripts/example_node`）。Git管理時も実行bitを保持する。
3. `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})`によるインストール。
4. ビルド後にworkspaceの`install/setup.bash`をsourceする。

`install(PROGRAMS)`は実行用のインストール規則だが、`--symlink-install`を使う場合もあるため、
ソースの実行権限まで確認する。実行権限だけを付けても、package登録・インストール先が誤っていれば起動できない。

C++の実行ファイルはCMakeで生成・インストールする。`.cpp`自体への実行権限は不要。
importされるPython moduleや、`ros2 launch`が読み込む`.launch.py`ファイルにも通常は実行権限は不要。
launch/configは`share/${PROJECT_NAME}`へインストールする。すべての`.py`へ一律にchmodしない。

## 3. 各packageにREADME.mdを置く

以下を必須とし、実装変更と同じ作業で更新する。READMEは`share/${PROJECT_NAME}`にもインストールする。

### 概要・ノード

- 責務と他packageとの分担。
- 既定ROSノード名、package名、実行ファイル名、Componentのplugin名。
- ノードがないmsg専用・launch専用packageは「自身のノード／入出力はなし」と明記する。

### 入出力topic

ノードごとに、最低限次の列を持つ表を用意する。

| 方向 | 既定topic名 | 型 | QoS | 内容 |
| --- | --- | --- | --- | --- |
| 入力 | `/joy` | `sensor_msgs/msg/Joy` | reliable / volatile / depth 1 | 正規化入力 |

- **入力／出力、topic既定名、完全な型名を必ず明記する。**
- 既定名はnamespaceなし・remapなしで解決した名前を記載する。
  相対名・private名の使用、namespace/remapで名前がどう変わるかも説明する。
- QoSのreliability・durability・depth、周期／イベント条件、単位・値域を記載する。
- 条件付きのpublisher/subscriber、外部ノードが必要な入力、観測専用出力を区別する。
- サービス・actionがあれば名前・型・方向・用途を別表で記載する。
- `/rosout`、`/parameter_events`などROS標準の自動生成インターフェースは、省略を明示したうえで除外してよい。

### パラメータ・launch引数

- **名前、既定値、意味を必ず記載する。** 可能なものは型、単位、範囲も記載する。
- 起動時固定か、動作中に変更可能かを区別する。
- ノード自体の既定値と、launch/configによる上書き後の値を混同しない。
- 必須入力、空文字、環境変数に依存するパスは、その条件を記載する。
- launch引数、ROSパラメータ、profile等の独自設定を別項目にする。
- パラメータがない場合も「独自パラメータなし」と明記する。

### 起動・検証

- ビルド・source・起動方法と必要な外部依存。
- 対応するテスト、確認済みの範囲、未確認のROS結合・実機項目。
- 起動例が実機出力を伴う場合はその条件を明示する。

## 4. 設定の配置と優先順位

- **全ての自作ROS packageにconfig/を置く。** 原則として独自ROSパラメータを全てYAMLに列挙し、
  READMEにも既定値・意味・範囲を記載する。原則の例外は理由と設定方法を明記する。
- 個別pkgのconfigはパラメータ一覧・単体起動用の基準値。実運用設定の正本ではない。
  msg専用package等はconfig/README.mdに「パラメータなし」と記載し、架空の設定を作らない。
- **実運用で読み込むのはkart_bringup/config/の設定。** input/、system/、vehicle/、recording/等の
  モジュール別ディレクトリ・ノード別ファイルへ分け、静的な初期値を明記する。
- bringupは個別pkgのconfigを暗黙のfallbackとして重ねない。新規parameter追加時は
  個別pkgのconfig、bringupの該当config、両READMEを同じ変更で更新する。
- **launchは起動構成の組立てと実行時に変えたい値の窓口。** YAMLのノードparameterを
  launch内の辞書へ重複定義しない。ノード／plugin名等の構成情報はlaunchに記述してよい。
- 運用上の優先順位は「bringupの静的YAML → 明示的なlaunch引数」。引数未指定ならYAMLを保持する。
  現在は空文字を未指定として扱い、root namespaceの明示は`namespace:=/`を使う。
- container名・機能のenabled・終了猶予もbringup/config/bringup.yamlに基準値を置く。
  record_dir等の明示指定は対応するノードparameterへ変換して最後に適用する。
- 起動前に設定ファイル・実際の採用値・明示上書きを表示する。未知の起動構成キーや不整合はエラーにする。
- 個別pkgのconfigとbringupのconfigは用途が違うので値の差は許容する。
  ノード内の既定値は直接実行時のfallbackだが、設定への記載漏れを放置する理由にはしない。
- profile等の独自ファイル、ROS共通parameter、起動後の操作要求は分類を明示する。
  YAMLの変更は原則として次回起動時に反映し、動的変更の可否はノード実装に従う。

現在の配置例:

```text
kart_joy/config/joy.yaml                       # pkgの基準値
kart_system/config/joy_manager.yaml            # pkgの基準値
kart_vehicle/config/bridge.yaml                # pkgの基準値
kart_bringup/config/
├── bringup.yaml                               # 起動構成と各ファイルの対応
├── input/{joy,joy_manager}.yaml                # 実運用値
├── system/{operation_mode_manager,command_mux}.yaml
├── vehicle/bridge.yaml
└── recording/bag_manager.yaml
```

### aptで導入する外部ROS package

Isaac ROS等のapt導入packageには、上流package側のconfig/やREADMEを整備する義務を適用しない。
代わりに、**使用するノードの設定YAML一式をkart_bringup/config/の対応モジュールへ置く。**

- 変更した値だけの差分YAMLではなく、使用ノードの設定可能な独自parameterを原則全て明記する。
  上流YAMLにないparameterも、対応バージョンの公式ドキュメント・実装で確認して補う。
  未使用ノードやpackage全体のサンプルを無条件にコピーする必要はない。
- aptのshare配下のYAMLを暗黙の下地にしない。外部launchが追加する固定値や別設定も確認し、
  bringupのYAMLが実際に適用されるよう接続する。外部launchの制約で従えない場合は、
  隠れる値・優先順位・理由をYAMLコメントとbringup READMEに明記する。
- YAML冒頭に、ROS package名、apt package名と確認したバージョン、上流の出典URL・ファイルパス、
  tag/commit（取得できる場合）、確認日をコメントで残す。未確認情報を推測で埋めない。
- kartで変更した値と理由は該当項目のコメントに記載する。起動時に設定できない項目や
  動的に生成される項目等を省略する場合も、項目名と理由を残す。
- bringup READMEに、設定ファイル、対象ノード、parameterの意味・値、出典、上流との差異を記載する。
  長い一覧はモジュール別READMEへ分離してリンクしてよい。
- apt更新時は出典との追加・削除・改名・既定値・型の差分を確認し、YAMLとREADMEを更新する。
  コピーした設定は上流更新へ自動追従しない。転載するコメント等のライセンス表記も保持する。

YAML冒頭の記載例（プレースホルダーは導入時に実値へ置き換える）:

```yaml
# ROS package: <package_name>
# APT package/version: <apt_package_name> / <verified_version>
# Upstream: <official_source_url_at_tag_or_commit>
# Source config: <upstream_relative_path or no upstream YAML>
# Verified: <YYYY-MM-DD>
# kart changes: <changed parameters and reasons; details beside each value>
# Omitted parameters: <names and reasons, or none>
```

## 5. 入力と車両操作の責務を分ける

現在のJoy構成は以下を基本とする。

- `kart_joy`: evdev接続監視、取得、正規化、配置変換。車両モードや走行指令を判断しない。
- `kart_joy_manager`: 入力の鮮度・接続・操作許可を一括管理し、車両指令・モード要求・トリム要求を生成する。
  連続入力とボタン操作は同一ノード内の別クラスにする。
- mode manager: モード状態を管理する。
- command mux: モードに応じた指令の選択と鮮度確認を行う。
- bridge: 基板通信、最終指令確認、出力補正を行う。
- `kart_bag_manager`: 独立Pythonプロセスで録画を管理する。Joy managerのL1/R1要求を受け、
  録画と走行モードを独立させる。録画の開始・停止を走行container内で待たない。

ノード・package分割は責務や独立した再利用の必要性で決める。クラス分割だけで足りるものを無条件に別ノードにしない。

### 車両モードとESC状態を区別する

- AUTO/MANUAL/STOP/PROPOの制御権と、ESC出力履歴のunknown/neutral/forward/brake/reverseを別に管理する。
- ESC指令の遷移はbridgeの出力直前に集約する。Joy・自律制御側で独立したESCモデルを重複実装しない。
- 指令履歴からのモデル、基板が報告したPWM、実測した車速・停止を混同しない。
  中立指令やブレーキ保持時間の満了を停止確認として扱わない。
- 起動・再接続・制御権喪失で履歴の信頼性を失った場合はunknownへ戻す。
  異常回復だけでは走行再開しない。
- 遷移表と異常時の出力規則をREADMEに記載し、保持・中立経由・再接続を含むテストを設ける。

## 6. 接続図の自動生成を見据える

目標は、各ノードのpublisher/subscriber情報とlaunch設定からtopic接続図を生成できる状態にすること。
**現在、自動生成器や機械可読manifestは未実装**。READMEを揃えることを第一段階とする。

将来の生成器は、次を扱う設計にする。

- ノードごとのpackage・実行ファイル／plugin・既定名。
- 各入出力の相対名／private名／絶対名、型、QoS、生成条件。
- launchのnamespace・remap・条件分岐。topic名は解決後の名前で比較する。
- 同じ名前でも型・QoSが互換でない場合を区別する。
- 未起動の自律制御ノード等、外部依存を図から消さず外部ノードとして表示する。
- 設計上の接続図と、実行中のROS graphから得た観測図を区別する。

READMEの自由文だけでは正確な自動生成は難しい。導入時にはpackageごとの`interfaces.yaml`等の
機械可読定義を検討し、そこからREADMEの表とMermaid等を生成する。
実装との整合チェックを併設し、README・manifest・実装の三重手編集で食い違いを増やさない。
ファイル形式とschemaは生成器導入時に決め、本書に追記する。

## 7. 変更時の確認

- packageのREADME、全体構成、launch、設定例の名前・型・既定値が一致しているか。
- 新規／変更スクリプトのshebang・実行bit・インストール先が適切か。
- 実装変更に必要な単体テスト・ビルド・ROS結合確認を実施したか。
- 実行できない確認は理由と未確認範囲を残したか。
- 方針を変更した場合は本書とルート`AGENTS.md`を必要に応じて更新したか。

この方針はrepo内で管理し、会話履歴やセッション固有の記憶だけに依存させない。

## 8. ホスト単位のハードウェア監視

Jetsonの監視は通常vehicle launchが1プロセスを所有する。
localization/control等の各moduleに重複配置しない。公式jtopはPython実行ファイルなので
containerへloadせず別processで起動する。notepcでは自動skipし、Jetson限定依存を無条件に要求しない。
測定DiagnosticArrayを専用topicへ分離して標準rosbag対象へ含める。
監視の起動と録画STARTは分離し、hostのfan/clock/power設定をlaunchで変更しない。
