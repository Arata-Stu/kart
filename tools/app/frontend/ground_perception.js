import { $ } from './api.js';

export const groundKinds={rc_car:{name:'RCカー',label:'RC',color:'#39dcb0'},
  obstacle:{name:'障害物',label:'障',color:'#d16ba5'}};
const elapsedTime=value=>Number.isFinite(value) && value>=0 ? value : 0;
const validAge=value=>Number.isFinite(value) && value>=0;
function remaining(channel, lifetime) {
  const receipt=validAge(channel?.age_s) ? Math.max(0,lifetime-channel.age_s) : 0;
  return Number.isFinite(channel?.fresh_for_s) ? Math.min(receipt,Math.max(0,channel.fresh_for_s)) : receipt;
}

// The bridge already transforms these points to map. Never use the current car
// pose or infer a kind from a numeric model class ID here.
export function groundPerceptionView(channel, connected, elapsedS=0) {
  const elapsed=elapsedTime(elapsedS), objects=[];
  const result={objects,rc:0,obstacles:0,source:'',diagnostic:'',tone:'unknown',message:''};
  if(!connected) return {...result,message:'地図上の接地点: 未接続・受信停止'};
  if(!channel) return {...result,message:'地図上の接地点: 未対応・状態未取得'};
  if(validAge(channel.age_s)) result.source=`Jetson受信後 ${(channel.age_s+elapsed).toFixed(2)} 秒 · frame: ${channel.frame_id || '不明'} · stamp: ${channel.stamp_ns ?? '不明'} ns`;
  const diagnostic=channel.diagnostic;
  const diagnosticFresh=diagnostic?.available && diagnostic.fresh && elapsed<remaining(diagnostic,2.5);
  result.diagnostic=!diagnostic ? '投影診断: 状態未取得' : !diagnostic.available ? '投影診断: 配信なし'
    : !diagnosticFresh ? '投影診断: 未受信・受信停止'
    : `投影診断: ${diagnostic.message || diagnostic.reason || '状態不明'}${diagnostic.reason ? ` (${diagnostic.reason})` : ''}`;
  if(!channel.available) return {...result,message:'地図上の接地点: 配信なし'};
  if(diagnosticFresh && Number.isFinite(diagnostic.level) && diagnostic.level>0)
    return {...result,message:'地図上の接地点: 投影できません（診断を確認）'};
  if(!channel.fresh || elapsed>=remaining(channel,1))
    return {...result,message:`地図上の接地点: ${validAge(channel.age_s) ? '受信停止・表示失効' : '受信待ち'}`};
  if(channel.frame_id!=='map') return {...result,message:'地図上の接地点: frame不一致（map以外は表示しません）'};
  if(!Number.isInteger(channel.count) || channel.count<0 || !Array.isArray(channel.objects)
      || channel.count<channel.objects.length)
    return {...result,message:'地図上の接地点: 件数・点データが不正です'};
  let rejected=0;
  for(const object of channel.objects.slice(0,100)) {
    if(!object || !Object.hasOwn(groundKinds,object.kind)
        || ![object.x,object.y,object.z,object.score].every(Number.isFinite)
        || object.score<0 || object.score>1) {rejected++;continue;}
    objects.push(object);
    if(object.kind==='rc_car')result.rc++; else result.obstacles++;
  }
  const partial=channel.truncated || channel.objects.length>100 || channel.count>objects.length;
  result.message=`地図上の接地点: RCカー ${result.rc}件 · 障害物 ${result.obstacles}件${partial ? `（表示 ${objects.length} / 受信 ${channel.count}件）` : ''}${rejected ? ' · 不正な点を除外' : ''}`;
  result.tone='ok';
  return result;
}

export function renderGroundPerception(channel, connected, elapsedS=0) {
  const view=groundPerceptionView(channel,connected,elapsedS);
  $('ground-perception-status').textContent=view.message;
  $('ground-perception-status').dataset.tone=view.tone;
  $('ground-perception-source').textContent=view.source;
  $('ground-perception-diagnostic').textContent=view.diagnostic;
  return view;
}

export function paintGroundPerception(ctx, project, channel, connected, elapsedS=0) {
  const view=groundPerceptionView(channel,connected,elapsedS);
  let painted=0;
  for(const object of view.objects) {
    const point=project([object.x,object.y]);
    if(!Array.isArray(point) || point.length!==2 || !point.every(Number.isFinite))continue;
    const [x,y]=point, kind=groundKinds[object.kind];
    ctx.save();ctx.beginPath();
    if(object.kind==='rc_car')ctx.arc(x,y,6,0,Math.PI*2);
    else {ctx.moveTo(x,y-7);ctx.lineTo(x+7,y);ctx.lineTo(x,y+7);ctx.lineTo(x-7,y);ctx.closePath();}
    ctx.fillStyle=kind.color;ctx.fill();ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();
    ctx.font='bold 11px sans-serif';ctx.lineWidth=3;ctx.strokeStyle='#fff';
    ctx.strokeText(kind.label,x+10,y-7);ctx.fillStyle='#263a42';ctx.fillText(kind.label,x+10,y-7);
    ctx.restore();painted++;
  }
  return painted;
}
