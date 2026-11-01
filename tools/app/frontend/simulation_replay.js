export function replaySimulation(result, cfg) {
  if(!result.frames?.length)throw Error('試走結果がありません');
  let time=0;
  const car={};
  const reasons={mpc_not_converged:'MPC未収束（反復上限）',mpc_timeout:'MPC計算時間超過',mpc_heading_error:'MPC方向差超過',mpc_off_path:'MPCライン逸脱',invalid_geometry_or_off_path:'先読み点・形状不正またはライン逸脱',vehicle_clearance:'車体が走行可能領域外・障害物に接近',off_route:'計画経路から逸脱',wrong_heading:'車体の向きが計画経路と不一致',stalled_before_goal:'goal手前で進捗なし',controller_invalid:'制御計算が成立しません',localization_jump:'位置の変化が許容値を超過',left_goal_after_arrival:'到達後に駐車枠から逸脱',motion_after_parking:'駐車完了後に移動',invalid_measurement:'位置・速度の値が不正',clock_jump:'時刻の変化が不正',simulated_parked:'モデル上の駐車完了（実車未確認）',planned_goal:'計画上のgoal到達（実車の駐車完了判定ではありません）',completed:'試走完了',stopped_target:'停止目標が継続',time_limit:'時間上限'};
  function update(){
    const i=Math.min(result.frames.length-1,Math.floor(time*30+1e-8));
    const [t,x,y,yaw,speed,steering,error,maxError,tx,ty,solverMs,targetSpeed,lookahead]=result.frames[i];
    Object.assign(car,{time:t,x,y,yaw,speed,steering,error,maxError,target:{x:tx,y:ty},solverMs,targetSpeed,lookahead,
      phase:result.states?.[i],lap:result.lap_frames?.[i],laps:result.laps,solverFailure:time>=result.frames.at(-1)[0]?result.solver_failure:null,
      done:time>=result.frames.at(-1)[0],reason:time>=result.frames.at(-1)[0]?(reasons[result.reason]||result.reason):''});
    return car;
  }
  update();return {car,cfg,diagnostic:result.diagnostic,step(dt=.02){time+=dt;return update();}};
}
