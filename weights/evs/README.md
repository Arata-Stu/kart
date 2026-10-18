# 20ch EVS encoder重みの配置先

EventStateからexportした、時間補間なし20ch histogram encoderを配置する。

```text
weights/evs/
├── event_encoder.pt    # encoder bundle（state_dictとmetadataを含む）
└── event_encoder.json  # 同じexportで生成した対応metadata
```

Docker内のパスは`/workspaces/weights/evs/`。
元の学習checkpointではなく、EventStateの`tools/export_event_encoder.py`で
生成したbundleとJSONを、必ず同じ組で配置する。
別名を使う場合はCLIの`--weights`／`--model-json`を変更する。

対応契約は40ms・10bin・20ch・positive→negative・時間補間なし・sum pooling。
content解像度は160×120または320×240、固定統計は`histogram_version=2`。
配置するのはencoder重みであり、kartの操作教師による制御head学習は別途必要。

重みとJSONはGit管理対象外。自動取得・代替重みの生成は行わない。
[データセット作成・学習・推論の手順](../../ros2_ws/src/kart_e2e/README.md#evs-rawから制御policyまで2026-10-18)。
