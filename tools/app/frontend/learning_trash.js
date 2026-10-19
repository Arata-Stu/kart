import { $, api, action, toast } from './api.js';
const labels = {encoders:"蒸留済みTiny encoder",features:'特徴量の確認結果',datasets:'データセット',runs:'学習実験',models:'ONNXモデル',evaluations:'評価結果',engines:'TensorRT計測'};

export function trashButton(kind, row, refresh) {
  const button = document.createElement('button');
  button.className = 'quiet artifact-trash';
  button.textContent = 'ごみ箱へ';
  button.setAttribute('aria-label', `${labels[kind]} ${row.id} をごみ箱へ`);
  button.addEventListener('click', async () => {
    button.disabled = true;
    try {
      await api('e2e/trash', {kind, id:row.id});
      if (kind === 'evaluations') $('e2e-eval-result').replaceChildren();
      await refresh();
      toast('ごみ箱へ移動しました。整理・ごみ箱から復元できます');
    } catch (error) {toast(error.message, true); button.disabled = false;}
  });
  return button;
}

export function installTrash(refresh) {
  async function render() {
    const kind = $('e2e-manage-kind').value;
    const rows = kind === 'trash' ? await api('e2e/trash') : (await api('e2e'))[kind];
    const list = $('e2e-manage-list');
    list.replaceChildren();
    if (!rows.length) {const p=document.createElement('p');p.textContent='対象の成果物はありません';list.append(p);}
    for (const row of rows) {
      const entry = document.createElement('div');
      entry.className = 'artifact-row';
      const detail = document.createElement('div');
      detail.className = 'artifact-info';
      const title = document.createElement('strong');
      title.textContent = row.name || row.id.split('/').at(-1);
      const path = document.createElement('small');
      path.textContent = `${labels[row.kind || kind]} · ${row.id}`;
      detail.append(title,path);
      entry.append(detail);
      if (kind === 'trash') {
        const restore = document.createElement('button');
        restore.textContent = '復元';
        restore.setAttribute('aria-label', `${row.name} を復元`);
        restore.addEventListener('click', async () => {
          restore.disabled=true;
          try {await api('e2e/restore',{token:row.token});await refresh();await render();toast('元の保存先へ復元しました');}
          catch(error) {toast(error.message,true);restore.disabled=false;}
        });
        entry.append(restore);
      } else entry.append(trashButton(kind,row,async()=>{await refresh();await render();}));
      list.append(entry);
    }
  }
  action('e2e-manage-open', async()=>{await render();$('e2e-manage-dialog').showModal();});
  action('e2e-manage-close', ()=>$('e2e-manage-dialog').close());
  action('e2e-manage-refresh', render);
  $('e2e-manage-kind').addEventListener('change', ()=>render().catch(error=>toast(error.message,true)));
}
