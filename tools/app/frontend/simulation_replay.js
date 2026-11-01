export function replaySimulation(result, cfg) {
  if(!result.frames?.length)throw Error('試走結果がありません');
  let time=0;
  const car={};
  const reasons={mpc_not_converged:'MPC未収束（反復上限）',mpc_timeout:'MPC計算時間超過',mpc_heading_error:'MPC方向差超過',mpc_off_path:'MPCライン逸脱',invalid_geometry_or_off_path:'先読み点・形状不正またはライン逸脱',vehicle_clearance:'車体が走行可能領域外・障害物に接近',off_route:'計画経路から逸脱',wrong_heading:'車体の向きが計画経路と不一致',stalled_before_goal:'goal手前で進捗なし',controller_invalid:'制御計算が成立しません',localization_jump:'位置の変化が許容値を超過',left_goal_after_arrival:'到達後に駐車枠から逸脱',motion_after_parking:'駐車完了後に移動',invalid_measurement:'位置・速度の値が不正',clock_jump:'時刻の変化が不正',simulated_parked:'モデル上の駐車完了（実車未確認）',planned_goal:'計画上のgoal到達（実車の駐車完了判定ではありません）',completed:'試走完了',stopped_target:'停止目標が継続',time_limit:'時間上限'};
  function update(){
    let low=0,high=result.frames.length;
    while(low<high){const mid=(low+high)>>1;if(result.frames[mid][0]<=time+1e-8)low=mid+1;else high=mid;}
    const i=Math.max(0,low-1);
    const finished=time+1e-8>=result.frames.at(-1)[0];
    const [t,x,y,yaw,speed,steering,error,maxError,tx,ty,solverMs,targetSpeed,lookahead]=result.frames[i];
    Object.assign(car,{time:t,x,y,yaw,speed,steering,error,maxError,target:{x:tx,y:ty},solverMs,targetSpeed,lookahead,
      phase:result.states?.[i],lap:result.lap_frames?.[i],laps:result.laps,solverFailure:finished?result.solver_failure:null,
      opponent:result.opponents?.[i],h2h:result.h2h?.[i],
      done:finished,reason:finished?(reasons[result.reason]||result.reason):''});
    return car;
  }
  Object.assign(reasons,{overtake_completed:'モデル上の追い抜き・合流完了',merge_unavailable:'合流車間不足・試走中断',merge_deadline:'合流期限超過・試走中断',boundary_violation:'境界・障害物との余裕不足',vehicle_conflict:'相手との接触または余裕不足',compute_timeout:'試走計算時間超過'});
  update();return {car,cfg,duration:result.frames.at(-1)[0],h2hResult:result.h2h?result:null,diagnostic:result.diagnostic,seek(t){time=Math.max(0,Math.min(result.frames.at(-1)[0],t));return update();},step(dt=.02){time+=dt;return update();}};
}
