import { $ } from './api.js';
const phases={inactive:'待機',driving:'通常走行監視',suspect:'接触・スタックの疑い',
  braking:'制動中',reverse_neutral:'後退前の中立',reversing:'後退中',post_reverse_braking:'後退後の制動・停車確認',resume_neutral:'再開前の中立',fault:'復帰中断・異常'};
const availability={absent:'未起動',waiting:'受信待ち・無効',stale:'受信停止',conflict:'配信元の競合'};
export function renderVehicle(status) {
  const bridge=status?.vehicle?.bridge, recovery=status?.vehicle?.recovery;
  const b=bridge?.availability==='fresh'?bridge:null, r=recovery?.availability==='fresh'?recovery:null;
  const mode={1:'AUTO',2:'MANUAL',3:'STOP',4:'PROPO'}[status?.mode];
  $('vehicle-mode').textContent=mode || 'モード不明';
  $('vehicle-mode').dataset.tone=mode==='STOP'?'neutral':mode?'ok':'unknown';
  $('vehicle-host').textContent=b ? `HOST: ${b.values.host_active==='true'?'有効':'無効'} · ESC指令 ${b.values.esc_command_state || 'ESC不明'}`
    : `HOST: ${availability[bridge?.availability] || '未接続'}`;
  $('vehicle-recovery').textContent=r ? `復帰: ${phases[r.state] || '不明'} · 試行 ${r.attempts}回${r.reason?' · '+r.reason:''}`
    : `復帰: ${availability[recovery?.availability] || '未接続'}`;
  $('vehicle-recovery').dataset.tone=r?.state==='fault'?'error':r && !['inactive','driving'].includes(r.state)?'warn':'neutral';
  $('vehicle-health').textContent=b ? `${b.level>=2?'車両異常':'車両通信'}: ${b.message}${b.values.fault_bits && b.values.fault_bits!=='0'?' · fault_bits='+b.values.fault_bits:''}` : '車両の通信状態は未確認';
  $('vehicle-health').dataset.tone=b?.level>=2?'error':'neutral';
  $('vehicle-calibration').textContent=b ? `steering_offset ${b.values.steering_offset ?? '不明'} / trim ${b.values.steering_trim ?? '不明'}（別の補正値）` : '';
}
