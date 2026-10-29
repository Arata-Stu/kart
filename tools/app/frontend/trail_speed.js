import { paintSpeedLine } from './line_speed.js';

// Odometry speed magnitude, not target speed or signed forward/reverse motion.
export function paintTrailSpeed(ctx, trail, project, max=3) {
  for(let i=1;i<trail.length;i++) {
    const a=trail[i-1],b=trail[i];
    if(!a||!b) continue;
    if(Number.isFinite(a[2])&&Number.isFinite(b[2])&&a[2]>=0&&b[2]>=0) {
      paintSpeedLine(ctx,[a,b],false,[a[2],b[2]],max,project);
    } else {
      const p=project(a),q=project(b);
      ctx.save();ctx.setLineDash([]);ctx.strokeStyle='#88919b';ctx.lineWidth=3;
      ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(...q);ctx.stroke();ctx.restore();
    }
  }
}
