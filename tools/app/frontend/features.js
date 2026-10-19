import { $, api, action } from './api.js';
let generation = 0;
export async function showFeatures(id) {
  const token = ++generation;
  const report = await api('e2e/features?id=' + encodeURIComponent(id));
  if (token !== generation) return;
  const slider = $('e2e-features-slider');
  slider.max = report.frames.length - 1;
  slider.value = 0;
  $('e2e-features-source').textContent = `${id} · ${report.mode} · PCA上位3成分 ${(100 * report.pca_explained_variance.reduce((a,b)=>a+b,0)).toFixed(1)}%`;
  function draw() {
    const i = Number(slider.value), frame = report.frames[i];
    $('e2e-features-info').textContent = `${i+1} / ${report.frames.length} · sample ${frame.sample_index} · 開始から ${((frame.stamp_ns-report.frames[0].stamp_ns)/1e9).toFixed(3)} s · ステア 教師 ${frame.steering_label.toFixed(3)} / 予測 ${frame.steering_prediction.toFixed(3)}`;
    const root = $('e2e-features-images');
    root.replaceChildren();
    for (const [key,label] of [['rgb','RGB（モデル入力）'],['pca','パッチ特徴 PCA'],['saliency','ステア入力感度']]) {
      const figure = document.createElement('figure');
      const image = document.createElement('img');
      image.alt = label;
      image.src = '/e2e-feature-image?id=' + encodeURIComponent(id) + '&file=' + encodeURIComponent(frame[key]);
      const caption = document.createElement('figcaption'); caption.textContent = label;
      figure.append(image,caption); root.append(figure);
    }
  }
  slider.oninput = draw; draw();
  if (!$('e2e-features-dialog').open) $('e2e-features-dialog').showModal();
}
action('e2e-features-close', () => {generation++; $('e2e-features-dialog').close();});
