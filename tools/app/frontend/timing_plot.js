// Each series has independent validity; negative ROS input ages are meaningful.
export function timingPlot(samples, now, keys) {
  const rows=samples.filter(p=>Number.isFinite(p.t)&&p.t>=now-60&&p.t<=now);
  const values=rows.flatMap(p=>keys.map(k=>p[k]).filter(Number.isFinite));
  const min=Math.min(0,...values), max=Math.max(1,...values)*1.1;
  const y=v=>140-(v-min)/(max-min)*120;
  const paths=Object.fromEntries(keys.map(key=>{
    let previous=null;
    return [key,rows.map(p=>{
      if(!Number.isFinite(p[key])){previous=null;return '';}
      const move=previous===null||p.t-previous>.5;previous=p.t;
      return `${move?'M':'L'}${(48+(p.t-now+60)/60*700).toFixed(2)},${y(p[key]).toFixed(2)}`;
    }).join(' ')];
  }));
  return {min,max,y,paths,last:rows.at(-1)};
}
