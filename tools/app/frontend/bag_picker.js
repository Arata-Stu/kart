import { $, action } from './api.js';

// Keep full relative paths as identities, including bags with repeated leaf names.
export function installBagPicker() {
  let records = [], target = null;
  const expanded = new Set(['']);
  const dialog = $('e2e-bag-dialog');
  function caption(id) {
    $(id + '-choose').textContent = $(id).value || 'bagを階層から選択';
  }
  function render() {
    const root = {name:'record', id:'', children:new Map()};
    for (const record of records) {
      let node = root;
      const parts = record.id.split('/');
      parts.forEach((part,i) => {
        if (!node.children.has(part)) node.children.set(part, {
          name:part, id:parts.slice(0,i+1).join('/'), children:new Map(),
        });
        node = node.children.get(part);
      });
      node.bag = true;
    }
    const list = $('e2e-bag-tree'); list.replaceChildren();
    function branch(node,parent) {
      const li = document.createElement('li'), button = document.createElement('button');
      button.type = 'button'; button.className = 'remote-tree-row';
      const open = expanded.has(node.id);
      button.textContent = (node.bag ? '▣ ' : open ? '▾ ' : '▸ ') + node.name;
      button.title = 'record/' + node.id;
      if (node.bag) {
        button.setAttribute('aria-pressed', String($(target).value === node.id));
        button.classList.toggle('selected', $(target).value === node.id);
      } else button.setAttribute('aria-expanded', String(open));
      button.addEventListener('click', () => {
        if (node.bag) {
          $(target).value = node.id;
          caption(target);
          $(target).dispatchEvent(new Event('change', {bubbles:true}));
          dialog.close();
        } else {
          if (open) expanded.delete(node.id); else expanded.add(node.id);
          render();
        }
      });
      li.append(button); parent.append(li);
      if (!node.bag && open) {
        const children = document.createElement('ul'); li.append(children);
        for (const child of [...node.children.values()].sort((a,b) =>
          Number(Boolean(a.bag))-Number(Boolean(b.bag)) || b.name.localeCompare(a.name, 'ja', {numeric:true}))) branch(child,children);
      }
    }
    if (!records.length) {list.textContent='Notebookにbagがありません。「転送」から先に受信してください。';return;}
    branch(root,list);
  }
  for (const id of ['e2e-bag','e2e-eval-bag']) action(id+'-choose', () => {
    target = id;
    const parts = $(id).value.split('/');
    for (let i=0;i<parts.length;i++) expanded.add(parts.slice(0,i).join('/'));
    render(); dialog.showModal();
  });
  action('e2e-bag-close', () => dialog.close());
  return function refresh(rows) {
    records = rows;
    for (const id of ['e2e-bag','e2e-eval-bag']) {
      if (!rows.some(row=>row.id===$(id).value)) $(id).value='';
      caption(id);
    }
    if (dialog.open) render();
  };
}
