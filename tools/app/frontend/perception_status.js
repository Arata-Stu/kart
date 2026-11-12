import { $ } from './api.js';

const channels=[['detections','検出'],['tracks','追跡']];
function ageText(age) {
  return Number.isFinite(age) && age>=0 ? `Jetson受信後 ${age.toFixed(2)} 秒` : '';
}
function objectText(object) {
  const id=object.id ? `ID ${object.id} · ` : '';
  const score=Number.isFinite(object.score) ? object.score.toFixed(2) : '不明';
  return `${id}class ID ${object.class_id ?? '不明'} · score ${score}`;
}

export function renderPerception(perception, connected, elapsedS=0) {
  const elapsed=Number.isFinite(elapsedS) && elapsedS>=0 ? elapsedS : 0;
  for(const [kind,label] of channels) {
    const summary=$(`perception-${kind}-status`), source=$(`perception-${kind}-source`), list=$(`perception-${kind}-objects`);
    const channel=connected ? perception?.[kind] : null;
    source.textContent=''; list.textContent=''; list.hidden=true;
    summary.dataset.tone='unknown';
    if(!connected) {summary.textContent=`${label}: 未接続・受信停止`;continue;}
    if(!channel) {summary.textContent=`${label}: 状態未取得`;continue;}
    if(!channel.available) {summary.textContent=`${label}: 配信なし`;continue;}
    const age=ageText(Number.isFinite(channel.age_s) ? channel.age_s+elapsed : null);
    if(age) source.textContent=[age,`frame: ${channel.frame_id || '不明'}`,`stamp: ${channel.stamp_ns ?? '不明'} ns`].join(' · ');
    const remaining=Number.isFinite(channel.fresh_for_s) ? channel.fresh_for_s
      : Number.isFinite(channel.age_s) ? Math.max(0,2-channel.age_s) : 0;
    if(!channel.fresh || elapsed>=remaining) {
      summary.textContent=`${label}: ${age ? '受信停止' : '受信待ち'}`;
      continue;
    }
    if(!Number.isInteger(channel.count) || channel.count<0) {
      summary.textContent=`${label}: 件数未確認`;
      continue;
    }
    const objects=Array.isArray(channel.objects) ? channel.objects.slice(0,10) : [];
    const partial=channel.truncated || channel.objects?.length>10;
    summary.textContent=`${label}: ${channel.count}件${partial ? `（${objects.length}件を表示）` : ''}`;
    summary.dataset.tone='ok';
    if(channel.count>0 && objects.length) {
      list.textContent=objects.map(objectText).join('\n');
      list.hidden=false;
    }
  }
}
