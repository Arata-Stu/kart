export const passingColors={both:'#8b66ba',left:'#248d6d',right:'#bf7b28'};
export const passingSides={both:'左右',left:'左のみ',right:'右のみ'};

export function zoneIndices(start,end,count,closed) {
  if(!Number.isInteger(start) || !Number.isInteger(end) || !Number.isInteger(count)
      || start<0 || end<0 || start>=count || end>=count || start===end || (!closed && start>end))
    throw Error('開始・終了WPは異なる点を指定してください。開いたラインでは開始より後のWPを終了にします。');
  const result=[start];
  while(result.at(-1)!==end)result.push((result.at(-1)+1)%count);
  return result;
}

export function zonePath(zone,line,closed) {
  if(!zone || typeof zone!=='object' || !line || typeof line.geometry_revision!=='string' || zone.geometry_revision!==line.geometry_revision
      || !Object.hasOwn(passingSides,zone.side))return null;
  try {
    const points=zoneIndices(zone.start_index,zone.end_index,line.points.length,closed).map(i=>line.points[i]);
    return points.every(p=>Array.isArray(p) && p.length===2 && p.every(Number.isFinite)) ? points : null;
  } catch {return null;}
}

export function newZone(line,closed,start,end,side) {
  if(!line || !/^[a-f0-9]{64}$/.test(line.geometry_revision || ''))throw Error('生成済みラインを再読み込みしてください');
  if(!Object.hasOwn(passingSides,side))throw Error('許可する側を選択してください');
  const existing=line.passing_zones || [];
  if(!Array.isArray(existing) || existing.some(zone=>!zone || typeof zone.id!=='string'))throw Error('失効した区間を削除してから設定し直してください');
  if(existing.length>=64)throw Error('追越し許可は各ライン64区間までです');
  zoneIndices(start,end,line.points.length,closed);
  let serial=1;
  while(existing.some(zone=>zone.id.toLowerCase()===`zone_${String(serial).padStart(3,'0')}`))serial++;
  return {id:`zone_${String(serial).padStart(3,'0')}`,start_index:start,end_index:end,side,geometry_revision:line.geometry_revision};
}

export function nearestWaypoint(points,point,tolerance) {
  let selected=-1,best=tolerance;
  points.forEach((p,i)=>{const distance=Math.hypot(p[0]-point[0],p[1]-point[1]);if(distance<=best){selected=i;best=distance;}});
  return selected;
}
