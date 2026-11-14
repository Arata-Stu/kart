const DISTANCE_KEYS=['lookahead_m','target_distance_m'];
const SPEED_KEYS=['measured_speed_mps','target_speed_mps'];

export function lookaheadSupported(row) {
  return row?.controller==='mpc' || (row?.controller==='pure_pursuit'
    && ['fixed','speed_interpolated'].includes(row.lookahead_mode));
}

export function lookaheadValue(row,key) {
  if(!lookaheadSupported(row) || !Number.isFinite(row[key]))return null;
  return DISTANCE_KEYS.includes(key) && row[key]<0 ? null : row[key];
}

// Only diagnostics are plotted. Nulls stay missing and neither chart resamples.
export function lookaheadPlot(samples,now) {
  const unique=new Map();
  if(Number.isFinite(now) && Array.isArray(samples))for(const row of samples) {
    if(row && Number.isFinite(row.t) && row.t>=now-60 && row.t<=now)unique.set(row.t,row);
  }
  const rows=[...unique.values()].sort((a,b)=>a.t-b.t).slice(-650);
  const x=t=>48+(t-now+60)/60*700;
  function chart(keys) {
    const values=rows.flatMap(row=>keys.map(key=>lookaheadValue(row,key)).filter(Number.isFinite));
    const low=Math.min(0,...values),high=Math.max(1,...values);
    const min=Math.floor(low*2)/2,max=Math.ceil((high+Math.max(.1,(high-low)*.1))*2)/2;
    const y=value=>140-(value-min)/(max-min)*120;
    const paths=Object.fromEntries(keys.map(key=>{
      let previous=null;
      return [key,rows.map(row=>{
        const value=lookaheadValue(row,key);
        if(value===null){previous=null;return '';}
        const move=!previous || row.t-previous.t>=.5-1e-9
          || row.controller!==previous.controller || row.lookahead_mode!==previous.lookahead_mode;
        previous=row;
        return `${move?'M':'L'}${x(row.t).toFixed(2)},${y(value).toFixed(2)}`;
      }).join(' ')];
    }));
    return {min,max,y,paths};
  }
  return {rows,last:rows.at(-1),x,distance:chart(DISTANCE_KEYS),speed:chart(SPEED_KEYS)};
}
