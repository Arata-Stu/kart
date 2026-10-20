# RGB制御モデルの比較計画

目的は、十分な制御精度を保ちながらモデルサイズと準備時間を減らすこと。
以下は実験計画であり、精度や最良設定はまだ測定していない。
通常の単一headで比較し、Multi-head・gateの影響は別実験にする。
ONNX export・TensorRT buildはこの比較に含めない。

サイズ優先の一括実行は`bash scripts/e2e_model_compare.sh --train <dataset> --preset sizes --hours 6`を使用する。
既定のsizesはViT-S/B/Tinyのfreezeと全体fine-tuningの6条件。
Tinyは50 epoch蒸留した同じencoderから両条件を開始する（`--size-distill-epochs`で変更可能）。
scratchや蒸留epochの差まで含める従来の11条件は`--preset full`で実行する。
sizesは実用的なモデル候補の比較であり、Tinyの蒸留と公式S/Bの初期重みの違いも含む。
`size_comparison.md/csv`にparameter数、checkpointサイズ、MSE、同じ更新範囲のBに対する比率を保存する。
各方式の2 epoch試走から時間を見積もり、収まる条件を100 epochのまま実行する。
時間不足の条件を自動で短縮せず、未実行として記録する。
全条件の6時間内完了を保証するものではない。
詳細は[scripts README](../scripts/README.md#モデルサイズ事前学習蒸留の時間予算付き比較)を参照。

## 共通条件

- 同じRGB dataset、入力サイズ・前処理、操作ラベル、予測対象を使用する。
- 制御学習はSmooth L1（β=0.1）、100 epoch、Plateau ON、同じbatchとseed。
  初回はseed=42、候補を絞った後に42/43/44で再確認する。
- 凍結head学習はLR=1e-4、事前学習済み全体fine-tuningは1e-5、scratchは1e-4を出発点にする。
  これは各方式を実用条件で比較する設定であり、初期重みだけの因果比較ではない。
  事前学習の効果をより厳密に調べる場合は、同じarchitectureのpretrained全体学習とscratchに
  同じLR候補（1e-5/1e-4）を割り当て、調整予算も揃える。
- RAM preloadはON。凍結時のみ特徴量キャッシュを利用する。
  全体fine-tuningとscratchではキャッシュを無効にする。
- train=validationでも実行可能だが、そのMSEは学習データへの適合度。
  別走行bagでの誤差と実走行を確認するまで、コース追従性能の優劣とは断定しない。

## 第1段階: サイズ・初期重み・更新範囲（8条件）

| architecture | 初期重み | 更新対象 | 比較すること |
|---|---|---|---|
| ViT-B | 公式pretrained | headのみ | 大きな固定特徴の基準 |
| ViT-B | 公式pretrained | backbone＋head | 全体更新の効果 |
| ViT-B | scratch | backbone＋head | 同じBで事前学習の恩恵 |
| ViT-S | 公式pretrained | headのみ | Bからの縮小の影響 |
| ViT-S | 公式pretrained | backbone＋head | 小型でも全体更新で補えるか |
| ViT-S | scratch | backbone＋head | 同じSで事前学習の恩恵 |
| ViT-Tiny | scratch | backbone＋head | 蒸留なしのTiny基準 |
| simple_cnn | scratch | backbone＋head | 単純な軽量モデルの基準 |

Tinyはこのproject独自のDINOv3互換構成で、公式Tiny pretrained重みを使用する経路はない。
scratchのランダムencoderを凍結する条件は、現在のscratch学習では扱わない。
100 epochは共通の学習予算であり、scratchが収束する保証ではない。
100 epoch付近まで改善する条件は、その条件だけ予算を延長して再確認する。

## 第2段階: Tinyの特徴蒸留予算（まず3条件）

| ViT-S→Tinyの特徴蒸留 | その後の制御学習 |
|---|---|
| 10 epoch | encoder凍結・新しいheadを100 epoch |
| 50 epoch | encoder凍結・新しいheadを100 epoch |
| 100 epoch | encoder凍結・新しいheadを100 epoch |

教師は同一ViT-S重みで固定する。公式pretrained教師と制御fine-tuning済み教師を混ぜない。
画像・前処理・蒸留LR=1e-4・MSE＋cosine重み・batch・seedも同一にする。
各蒸留は同じseedのランダムTinyから独立に開始する。10 epochモデルから50 epochを追加する比較ではない。
第1段階のTiny scratchと合わせて、蒸留なし／10／50／100の4条件を比較できる。

現実装は各蒸留予算内の検証alignment loss最良を`encoder.pt`に保存する。
そのため「100 epoch予算」でも採用encoderは途中epochの場合がある。
`encoder.pt`のmetrics.epochを記録し、予算と採用epochを区別する。
これは最終epoch固定の比較ではなく、各予算で得られた最良encoderの比較。
蒸留lossの低下だけでは制御精度の改善を判断せず、後段の制御MSE・MAEで比較する。

凍結で有望な蒸留予算を絞った後、同じencoderから全体fine-tuning（LR=1e-5）も実行する。
凍結と全体更新を比べる際は、片方の制御学習の続きから開始せず、同じ蒸留encoderから開始する。

既存CLIの実行例（Notebookコンテナ、`/workspaces`で実行）:

```bash
export PYTHONPATH=/workspaces/ros2_ws/src/kart_e2e${PYTHONPATH:+:$PYTHONPATH}
dataset=/workspaces/e2e/datasets/日付/時刻/dataset名
comparison=/workspaces/e2e/comparisons/$(date +%F/%H-%M-%S)
for budget in 10 50 100; do
  /usr/bin/python3 -m kart_e2e.distill \
    --train "$dataset" --validation "$dataset" \
    --repo /workspaces/python_ws/dinov3 \
    --teacher-weights /workspaces/weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth \
    --output "$comparison/distill-$budget" --epochs "$budget" \
    --batch-size 32 --learning-rate 1e-4 --seed 42 || break
  /usr/bin/python3 -m kart_e2e.train \
    --train "$dataset" --validation "$dataset" \
    --repo /workspaces/python_ws/dinov3 --architecture dinov3_vitt16 \
    --weights "$comparison/distill-$budget/encoder.pt" \
    --output "$comparison/head-$budget" --mode steer_only \
    --epochs 100 --loss smooth_l1 --lr-scheduler plateau \
    --batch-size 32 --learning-rate 1e-4 --seed 42 || break
done
```

新しい制御headでの学習になる。既定でencoder凍結、キャッシュ・RAM preload ON。
GUIでも通常モデル側の蒸留epochと第2段階のencoder選択で同じ操作ができる。

## 記録する比較指標

各条件で制御のbest MSE・steering MAE・採用epoch、25/50/100 epoch時点までの最良MSE、
制御学習の総時間、初回キャッシュ時間、parameter数、GPUメモリを記録する。
蒸留条件は蒸留時間と後段学習時間を足した総準備時間も比較する。
凍結はキャッシュ済み時間だけで比較せず、初回準備込みと再学習を分ける。
steer＋throttle学習では各出力のMAEも分けて記録する。

同じモデルでもseed間で差があるため、誤差が近ければ3 seedと別走行で再比較する。
許容精度を満たす候補の中から小型・短時間を優先する。
parameter数や学習速度だけでJetson上の推論速度を断定せず、推論時間は別段階で実測する。
