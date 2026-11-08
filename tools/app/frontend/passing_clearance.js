// Local cross-section in metres. Positive lateral coordinates are to the left.
export function passingSection(origin, tangent, lane, width, offset, margin) {
  const [tx,ty]=tangent, normal=[-ty,tx];
  const cuts=[];
  let onEdge=false;
  function inside(poly) {
    let yes=false;
    for(let i=0,j=poly.length-1;i<poly.length;j=i++) {
      const a=poly[j], b=poly[i];
      const local=p=>[(p[0]-origin[0])*tx+(p[1]-origin[1])*ty,
                      (p[0]-origin[0])*normal[0]+(p[1]-origin[1])*normal[1]];
      const [u,v]=local(a), [w,z]=local(b);
      if(Math.abs(u)<1e-8 && Math.abs(w)<1e-8) {
        cuts.push(v,z);
        if(Math.min(v,z)<=0 && Math.max(v,z)>=0)onEdge=true;
      } else if((u<=0 && w>=0)||(w<=0 && u>=0)) {
        const hit=v+(z-v)*u/(u-w);
        cuts.push(hit);
        if(Math.abs(hit)<1e-8)onEdge=true;
      }
      if((a[1]>origin[1])!==(b[1]>origin[1]) &&
         origin[0]<(b[0]-a[0])*(origin[1]-a[1])/(b[1]-a[1])+a[0])yes=!yes;
    }
    return yes;
  }
  const left=inside(lane.left),right=inside(lane.right);
  const blocked=(lane.obstacles||[]).map(o=>inside(o.polygon)).some(Boolean);
  const side=Math.sign(offset);
  const gaps=cuts.map(v=>side*v).filter(v=>v>=0);
  const available=gaps.length?Math.min(...gaps):null;
  const needed=Math.abs(offset)+width/2+margin;
  const between=Math.abs(offset)-width;
  return {normal,available,needed,between,margin,
          usable:!onEdge && left!==right && !blocked && available!==null,
          fits:!onEdge && left!==right && !blocked && available!==null &&
               available+1e-9>=needed && between+1e-9>=margin &&
               cuts.some(v=>side*v<0) && cuts.filter(v=>side*v<0).every(v=>Math.abs(v)+1e-9>=width/2+margin)};
}

export function drawPassingSection(ctx,project,origin,tangent,section,vehicle,offset) {
  const {normal,available,needed,margin}=section;
  const pos=(s,d)=>project([origin[0]+tangent[0]*s+normal[0]*d,origin[1]+tangent[1]*s+normal[1]*d]);
  function body(d,pad,color,dashed) {
    const half=vehicle.width/2+pad;
    const a=-vehicle.rear-pad,b=vehicle.length-vehicle.rear+pad;
    const corners=[[a,d-half],[b,d-half],[b,d+half],[a,d+half]];
    ctx.beginPath();corners.forEach(([s,v],i)=>i?ctx.lineTo(...pos(s,v)):ctx.moveTo(...pos(s,v)));
    ctx.closePath();ctx.strokeStyle=color;ctx.setLineDash(dashed?[4,3]:[]);
    if(!dashed){ctx.fillStyle=color+'aa';ctx.fill();}ctx.stroke();
  }
  ctx.save();ctx.lineWidth=2;
  body(0,0,'#b96108',false);body(offset,0,'#234c7a',false);
  body(offset,margin,section.fits?'#237851':'#b42335',true);
  ctx.setLineDash([]);ctx.font='bold 12px sans-serif';
  for(const [s,d,label,color] of [[-vehicle.rear-.05,0,'相手','#854300'],
      [vehicle.length-vehicle.rear+.05,offset,'自車','#234c7a']]){
    const [x,y]=pos(s,d);ctx.fillStyle=color;ctx.fillText(label,x+4,y-3);
  }
  function dimension(s,to,color){
    ctx.strokeStyle=color;ctx.beginPath();ctx.moveTo(...pos(s,0));ctx.lineTo(...pos(s,to));
    for(const d of [0,to]){ctx.moveTo(...pos(s-.03,d));ctx.lineTo(...pos(s+.03,d));}ctx.stroke();
  }
  dimension(-vehicle.rear-.10,Math.sign(offset)*needed,'#6f328f');
  if(available!==null)dimension(-vehicle.rear-.20,Math.sign(offset)*available,'#147781');
  ctx.restore();
}
