export function replaySimulation(result, cfg) {
  if(!result.frames?.length)throw Error('試走結果がありません');
  let time=0;
  const car={};
  const reasons={simulated_parked:'モデル上の駐車完了（実車未確認）',planned_goal:'計画上のgoal到達（実車の駐車完了判定ではありません）',completed:'試走完了',stopped_target:'停止目標が継続',time_limit:'時間上限'};
  function update(){
    const i=Math.min(result.frames.length-1,Math.floor(time*30+1e-8));
    const [t,x,y,yaw,speed,steering,error,maxError,tx,ty,solverMs,targetSpeed,lookahead]=result.frames[i];
    Object.assign(car,{time:t,x,y,yaw,speed,steering,error,maxError,target:{x:tx,y:ty},solverMs,targetSpeed,lookahead,
      done:time>=result.frames.at(-1)[0],reason:time>=result.frames.at(-1)[0]?(reasons[result.reason]||result.reason):''});
    return car;
  }
  update();return {car,cfg,step(dt=.02){time+=dt;return update();}};
}
