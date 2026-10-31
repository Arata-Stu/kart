// Pure geometry: gaps and invalid diagnostics are never connected.
export function speedPlot(samples, now) {
  const rows=samples.filter(p=>Number.isFinite(p.t) && p.t>=now-60 && p.t<=now);
  const valid=p=>Number.isFinite(p.target) && Number.isFinite(p.measured);
  const values=rows.filter(valid).flatMap(p=>[p.target,p.measured]);
  const low=Math.min(0,...values), high=Math.max(1,...values);
  const min=Math.floor(low*2)/2, max=Math.ceil((high+.1)*2)/2;
  const x=t=>48+(t-now+60)/60*700, y=v=>140-(v-min)/(max-min)*120;
  function path(key) {
    let previous=null;
    return rows.map(p=>{
      if(!valid(p)){previous=null;return '';}
      const move=previous===null || p.t-previous>.5;
      previous=p.t;
      return `${move?'M':'L'}${x(p.t).toFixed(2)},${y(p[key]).toFixed(2)}`;
    }).join(' ');
  }
  return {min,max,y,target:path('target'),measured:path('measured'),last:rows.at(-1)};
}
