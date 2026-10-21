import { $, api, action, emit, on, item, toast, state } from './api.js';

let selected = null, pending = null, submitting = false, catalog = [];
export const reportURL = (id, file='report.html') => '/audit-file?' + new URLSearchParams({id, file});

export function renderAuditSummary(row) {
  const final = row.final || {};
  const metric = final.count ? `初回遅延 P95 ${final.p95.toFixed(1)} ms · 最大 ${final.max.toFixed(1)} ms · ${final.count}画像` :
    '条件確認済みの遅延サンプルなし · 記録不足／除外理由を確認';
  $('audit-summary').textContent = metric;
  $('audit-title').textContent = row.bag;
  $('audit-meta').textContent = `${row.source_topic} · スパイク閾値 ${row.age_spike_ms} ms`;
}

function showReport(row) {
  selected = row.id;
  renderAuditSummary(row);
  $('audit-report').src = reportURL(row.id) + '#timeseries';
  $('audit-report').hidden = false;
  $('audit-empty').hidden = true;
  for (const [id, file] of [['audit-open','report.html'], ['audit-csv','events.csv'], ['audit-json','summary.json']]) {
    $(id).href = reportURL(row.id, file);
    $(id).hidden = false;
  }
  renderHistory();
}

function renderHistory() {
  const list = $('audit-list');
  list.replaceChildren();
  if (!catalog.length) {
    const p = document.createElement('p'); p.className = 'empty';
    p.textContent = 'bagを選んで解析すると、結果がここに保存されます。'; list.append(p);
  }
  for (const row of catalog) {
    const date = new Date(row.created * 1000).toLocaleString('ja-JP');
    list.append(item(row.bag, `${date} · 条件確認 ${row.final?.count || 0} / 観測 ${row.observed?.count || 0}画像`, () => showReport(row), row.id === selected));
  }
}

function busy() {
  const active = state.jobs.find(j => j.id === pending && ['running','orphaned'].includes(j.status));
  $('audit-run').disabled = submitting || !$('audit-bag').value;
  $('audit-stop').hidden = !active;
  $('audit-stop').disabled = !!active?.cancel_requested;
  if (active) $('audit-status').textContent = active.cancel_requested ? '停止処理中…' : '解析中 · 下部の処理一覧からログを確認できます';
}

export async function refreshAudit() {
  const [records, result] = await Promise.all([api('records'), api('audits')]);
  const select = $('audit-bag'), previous = select.value;
  select.replaceChildren();
  const empty = document.createElement('option'); empty.value = ''; empty.textContent = 'rosbagを選択'; select.append(empty);
  for (const record of records) {
    const option = document.createElement('option'); option.value = record.id; option.textContent = record.id; select.append(option);
  }
  select.value = records.some(r => r.id === previous) ? previous : '';
  catalog = result.results;
  if (!pending && result.running?.length) pending = result.running[0].id;
  $('audit-record-root').textContent = state.config?.records || '';
  renderHistory(); busy();
  if (!selected && catalog.length) showReport(catalog[0]);
  if (!records.length) $('audit-status').textContent = 'Notebookのrecordにbagがありません。「転送」でJetsonから取得してください。';
}

action('audit-refresh', refreshAudit);
$('audit-bag').addEventListener('change', busy);
$('audit-source').addEventListener('change', () => {
  $('audit-custom-label').hidden = $('audit-source').value !== 'custom';
});
$('audit-form').addEventListener('submit', async event => {
  event.preventDefault();
  if (submitting) return;
  submitting = true;
  const button = $('audit-run'); button.disabled = true;
  try {
    const source_topic = $('audit-source').value === 'custom' ? $('audit-custom').value.trim() : $('audit-source').value;
    const job = await api('audit/run', {bag:$('audit-bag').value, source_topic, age_spike_ms:Number($('audit-spike').value)});
    pending = job.id; emit('job-started', job); toast('時刻解析を開始しました');
  } catch (error) { toast(error.message, true); }
  finally { submitting = false; busy(); }
});
action('audit-stop', async () => {
  if (pending) await api('stop', {id:pending});
});
on('status', busy);
on('job-finished', async event => {
  const job = event.detail;
  if (job.id !== pending && !job.result?.audit) return;
  if (job.id === pending) {
    pending = null;
    $('audit-status').textContent = job.status === 'succeeded' ? '解析完了' : `${job.status === 'cancelled' ? '中止' : '解析失敗'} · ${job.message}`;
  }
  await refreshAudit();
  const result = catalog.find(row => row.id === job.result?.audit);
  if (result) showReport(result);
});
