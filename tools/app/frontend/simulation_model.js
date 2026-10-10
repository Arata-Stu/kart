// Offline kinematic preview. No ROS, motor, localization, or tire model.
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
export function createSimulation(line, closed, options = {}) {
  const cfg = {wheelbase: .257, lookahead: .8, maxSteer: .45, speed: 1, ...options};
  for (const key of Object.keys(cfg)) if (!Number.isFinite(cfg[key]) || cfg[key] <= 0) throw Error("試走設定は正の数にしてください");
  const points = line.points;
  if (!points || points.length < (closed ? 3 : 2) || points.some(p => !p.every(Number.isFinite))) throw Error("生成済みラインが必要です");
  const segments = []; let total = 0;
  for (let i = 0; i < points.length - (closed ? 0 : 1); i++) {
    const a = points[i], b = points[(i + 1) % points.length], length = Math.hypot(b[0]-a[0], b[1]-a[1]);
    if (length < 1e-8) continue;
    segments.push({a, b, length, start: total, i}); total += length;
  }
  if (!segments.length) throw Error("ライン長が不足しています");
  function sample(s) {
    s = closed ? ((s % total) + total) % total : clamp(s, 0, total);
    const seg = segments.find(e => s <= e.start + e.length) || segments.at(-1);
    const t = clamp((s-seg.start)/seg.length, 0, 1);
    const v0 = line.profile?.[seg.i]?.[5], v1 = line.profile?.[(seg.i+1)%points.length]?.[5];
    return {x: seg.a[0]+t*(seg.b[0]-seg.a[0]), y: seg.a[1]+t*(seg.b[1]-seg.a[1]), speed: Number.isFinite(v0) && Number.isFinite(v1) ? v0+t*(v1-v0) : cfg.speed};
  }
  const first = segments[0], car = {x:first.a[0], y:first.a[1], yaw:Math.atan2(first.b[1]-first.a[1],first.b[0]-first.a[0]), speed:0, steering:0, station:0, error:0, maxError:0, time:0, done:false, reason:""};
  function step(dt = .02) {
    if (car.done) return car;
    if (!(dt > 0 && dt <= .1)) throw Error("時間刻みが不正です");
    // Restrict search to forward progress; crossings must not switch branches.
    const low = Math.max(0, car.station-.05), high = Math.min(total, car.station + Math.max(.5, car.speed*dt*3));
    let best = null;
    for (const seg of segments) {
      if (seg.start+seg.length < low || seg.start > high) continue;
      const dx=seg.b[0]-seg.a[0], dy=seg.b[1]-seg.a[1];
      const t=clamp(((car.x-seg.a[0])*dx+(car.y-seg.a[1])*dy)/seg.length**2, Math.max(0,(low-seg.start)/seg.length), Math.min(1,(high-seg.start)/seg.length));
      const error=Math.hypot(car.x-seg.a[0]-t*dx,car.y-seg.a[1]-t*dy);
      if (!best || error < best.error) best={error, s:seg.start+t*seg.length};
    }
    car.station=Math.max(car.station,best.s); car.error=best.error; car.maxError=Math.max(car.maxError,car.error);
    if (total-car.station < .04 && Math.hypot(car.x-points[closed?0:points.length-1][0],car.y-points[closed?0:points.length-1][1]) < .15) {
      car.done=true; car.speed=0; car.reason=closed?"1周完了":"終点到達"; return car;
    }
    if (car.error > 2 || car.time >= 300) {car.done=true; car.speed=0; car.reason=car.error>2?"追従逸脱（2 m超）":"時間上限（300秒）"; return car;}
    const target=sample(car.station+cfg.lookahead), dx=target.x-car.x, dy=target.y-car.y;
    car.steering=clamp(Math.atan2(2*cfg.wheelbase*(-Math.sin(car.yaw)*dx+Math.cos(car.yaw)*dy),Math.max(dx*dx+dy*dy,1e-8)),-cfg.maxSteer,cfg.maxSteer);
    // Short preview avoids an open path's zero-speed first sample deadlock.
    const reference=sample(Math.min(total,Math.max(car.station,.05))).speed;
    let desired=Math.min(cfg.speed,reference);
    if (!closed) desired=Math.min(desired,Math.sqrt(2*2.5*Math.max(0,total-car.station)));
    car.speed += clamp(desired-car.speed,-2.5*dt,1.5*dt);
    const turn=car.speed/cfg.wheelbase*Math.tan(car.steering)*dt;
    car.x+=car.speed*Math.cos(car.yaw+turn/2)*dt; car.y+=car.speed*Math.sin(car.yaw+turn/2)*dt; car.yaw+=turn; car.time+=dt;
    return car;
  }
  return {car, step, total, cfg};
}
