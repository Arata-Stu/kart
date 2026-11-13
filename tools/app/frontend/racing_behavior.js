import { $ } from './api.js';

const modes={off:'off（無効）',shadow:'shadow（観察のみ）',active:'active（参照経路へ反映）'};
const states={CLEAR:'対象なし',FOLLOW:'追従',OBSERVE:'観察',PASS_MOVING:'移動対象を追越し',
  AVOID_SLOW:'低速対象を回避',AVOID_OBSTACLE:'障害物を回避',MERGE:'参照経路へ復帰',
  COAST_PASS:'欠測中に並走継続',HOLD_SIDE:'合流待ち',BLOCKED:'両側不成立',FAULT:'異常'};
const sides={left:'左',right:'右'};
const reasonLabels={adopted_swept_candidate:'占有範囲を検査した候補経路を採用',swept_assumed_geometry_clear:'仮定した占有範囲で通過可能',
  no_observed_blocking_object:'観測した進路上の対象なし',holding_adopted_route:'採用済み経路を維持',
  target_not_confirmed:'対象の動きが未確認',moving_observation_or_speed_advantage:'移動対象を観察中、または速度差不足',
  passing_opportunity_too_short:'追越しに使える距離が不足するため追従',no_swept_candidate:'左右とも通過可能な候補なし',
  start_alignment_required:'基準経路への位置・向き合わせが必要',boundary_or_registered_obstacle:'境界または登録障害物との余裕不足',
  object_collision:'対象の占有範囲との余裕不足',curvature_limit:'曲率上限を超過',route_too_long:'候補に必要な経路長が不足',
  prediction_horizon:'予測時間の範囲を超過',insufficient_speed_advantage:'追越しに必要な速度差が不足',
  moving_merge_clearance:'移動対象への復帰余裕が不足',zero_reference_speed:'参照経路の速度が0',
  pose_unavailable:'自己位置が未受信、または古い',baseline_or_map_unavailable:'基準経路または地図が未受信、または古い',
  perception_timeout:'認識入力の受信停止',confirmed_rc_tracks_unavailable:'確定RC追跡が未受信、または古い',
  reference_publisher_conflict:'参照経路の配信元が競合',clock_reversed:'ROS時刻が逆転',
  external_input:'入力異常',planner_error:'経路計画の処理エラー',follow_route_invalid:'追従経路が不成立',
  adopted_route_invalid:'採用済み経路が不成立',parameters_changed:'調整値が変更されたためリセットが必要',
  passing_not_authorized:'追越し許可区間外',opponent_model_learning:'相手車モデルの学習中のため追従',
  permission_interval_too_short:'許可区間内に合流まで収まらない',side_not_authorized:'この側への追越しは未許可',
  no_rc_model:'RCカーの学習モデルなし',native_library_unavailable:'学習用の実行ライブラリが未準備',
  committed_prediction:'採用した並走側を維持し、対象位置を予測',holding_committed_side:'採用した並走側を維持',
  merge_clearance_pending:'車体端の合流余裕を確認するまで並走',merge_clearance_verified:'車体端の合流余裕を確認',
  parallel_corridor_exhausted:'許可された並走区間の残りが不足',committed_permission_expired:'採用した追越し許可区間の期限切れ',
  straight_section_required:'追越しに必要な直線区間が不足',
  prediction_uncertainty_exceeded:'対象位置の予測不確かさが上限を超過',
  reference_geometry_or_revision_changed:'基準経路が変更されたためリセットが必要'};
const text=(value,fallback='—',limit=512)=>typeof value==='string' ? value.slice(0,limit) : fallback;
function reasonText(value) {
  const raw=text(value), [key,detail]=raw.split(':');
  const label=reasonLabels[key];
  return label ? `${label}${reasonLabels[detail] ? `: ${reasonLabels[detail]}` : ''}（${raw}）` : raw;
}
const number=(value,digits=2)=>Number.isFinite(value) ? value.toFixed(digits) : '不明';
const finitePoint=point=>Array.isArray(point) && point.length===2 && point.every(Number.isFinite);
function path(value) {
  return Array.isArray(value) && value.length<=300 && value.every(finitePoint) ? value : null;
}
function target(value) {
  return value && ['rc_car','obstacle'].includes(value.kind) && typeof value.id==='string'
    && [value.x,value.y].every(Number.isFinite) ? value : null;
}

export function behaviorView(channel, connected, elapsedS=0) {
  const elapsed=Number.isFinite(elapsedS) && elapsedS>=0 ? elapsedS : 0;
  const view={message:'追従・追越し: 未接続・受信停止',reason:'',source:'',target:'',candidatesText:'',geometryText:'',
    context:'',path:[],candidates:[],object:null,geometry:null,mode:null,tone:'unknown'};
  if(!connected)return view;
  if(!channel)return {...view,message:'追従・追越し: 未対応・状態未取得'};
  if(!channel.available)return {...view,message:'追従・追越し: 配信なし'};
  const age=Number.isFinite(channel.age_s) && channel.age_s>=0 ? channel.age_s : null;
  const receipt=age===null ? 0 : Math.max(0,.5-age);
  const remaining=Number.isFinite(channel.fresh_for_s) ? Math.min(receipt,Math.max(0,channel.fresh_for_s)) : receipt;
  if(age!==null)view.source=`Jetson受信後 ${(age+elapsed).toFixed(2)} 秒`;
  if(!channel.fresh || age===null || elapsed>=remaining)
    return {...view,message:`追従・追越し: ${age===null ? '受信待ち' : '受信停止・表示失効'}`};
  const status=channel.status;
  if(!status || !Object.hasOwn(modes,status.mode) || !Object.hasOwn(states,status.state)
      || typeof status.valid!=='boolean' || !Number.isFinite(status.target_speed) || status.target_speed<0)
    return {...view,message:'追従・追越し: 状態データが不正です'};
  const frame=status.frame_id ?? channel.frame_id;
  view.source+=` · frame: ${text(frame,'不明')} · stamp: ${text(status.stamp_ns ?? channel.stamp_ns,'不明',30)} ns`;
  if(frame!=='map')return {...view,message:'追従・追越し: frame不一致（map以外は表示しません）'};
  view.message=`追従・追越し: ${modes[status.mode]} · ${status.state}（${states[status.state]}）`;
  view.mode=status.mode;view.tone=status.state==='FAULT' || status.state==='BLOCKED' ? 'unknown' : 'ok';
  const context=[];
  const planning={simple:'単純・幾何ルール',learned:'学習モデル',follow:'追従のみ'}[status.planning_mode];
  if(planning)context.push(`計画方式: ${planning}`);
  if(status.planning_mode==='simple')context.push('この簡易方式は周回学習不要');
  const learning=status.learning;
  if(learning && typeof learning==='object')context.push(`学習${status.planning_mode==='simple' ? '（参考）' : ''}: ${learning.available ? learning.ready ? '利用可能' : '準備不足' : '利用不可'}${Number.isFinite(learning.coverage) ? ` · coverage ${number(learning.coverage)}` : ''}${learning.reason ? ` · ${reasonText(learning.reason)}` : ''}`);
  const permission=status.permission;
  if(permission && typeof permission==='object')context.push(`許可区間: ${text(permission.zone_id,'なし')} · ${{...sides,both:'左右'}[permission.side] || '許可なし'}${Number.isFinite(permission.remaining_m) ? ` · 残り ${number(permission.remaining_m)} m` : ''}`);
  else if(Object.hasOwn(status,'permission'))context.push('許可区間: なし');
  const observation=status.target_observation;
  if(observation && ['observed','predicted'].includes(observation.mode))context.push(`対象の位置: ${observation.mode==='observed' ? '観測' : '予測で保持'} · 観測後 ${number(observation.age_s)} 秒 · 位置の不確かさ ${number(observation.uncertainty_m)} m`);
  view.context=context.join('\n');
  view.reason=`理由: ${reasonText(status.reason)} · 目標速度 ${number(status.target_speed)} m/s · 選択側 ${sides[status.side] || 'なし'}${status.valid ? '' : ' · 有効な経路なし'}`;
  const object=target(status.target);
  view.target=object ? `対象: ${object.kind==='rc_car' ? 'RCカー' : '障害物'} · ID ${text(object.id)} · 推定速度 ${number(object.speed)} m/s · map (${number(object.x)}, ${number(object.y)}) m` : '対象: なし・未確定';
  const rawCandidates=Array.isArray(status.candidates) ? status.candidates.slice(0,2) : [];
  for(const side of ['left','right']) {
    const candidate=rawCandidates.find(row=>row && row.side===side);
    if(!candidate) {view.candidates.push({side,feasible:false,reason:'未算出',path:[]});continue;}
    const points=path(candidate.path), feasible=candidate.feasible ?? candidate.valid;
    view.candidates.push({side,feasible:feasible===true && points!==null,reason:points===null ? '候補座標が不正です' : reasonText(candidate.reason),
      path:points || [],min_clearance_m:candidate.min_clearance_m,
      merge_start:finitePoint(candidate.merge_start) ? candidate.merge_start : null,
      merge_end:finitePoint(candidate.merge_end) ? candidate.merge_end : null,
      merge_distance_m:Number.isFinite(candidate.merge_end_s) && Number.isFinite(candidate.merge_start_s)
        ? Math.max(0,candidate.merge_end_s-candidate.merge_start_s) : null});
  }
  view.candidatesText=view.candidates.map(candidate=>`${sides[candidate.side]}: ${candidate.feasible ? '成立' : '不成立'} · ${candidate.reason}${Number.isFinite(candidate.min_clearance_m) ? ` · 最小余裕 ${number(candidate.min_clearance_m)} m` : ''}${Number.isFinite(candidate.merge_distance_m) ? ` · 復帰区間 ${number(candidate.merge_distance_m)} m` : ''}`).join('\n');
  const geometry=status.geometry;
  if(geometry && ['vehicle_length_m','vehicle_width_m'].every(key=>Number.isFinite(geometry[key]) && geometry[key]>0 && geometry[key]<=10)
      && ['target_length_m','target_width_m'].every(key=>Number.isFinite(geometry[key]) && (object ? geometry[key]>0 : geometry[key]>=0) && geometry[key]<=10)
      && Number.isFinite(geometry.merge_clearance_m) && geometry.merge_clearance_m>=0) {
    view.geometry=geometry;
    const dimensions=object ? `${number(geometry.target_length_m)} × ${number(geometry.target_width_m)} m` : 'なし';
    const rear=geometry.rear_axle_to_rear_m;
    const rearText=Number.isFinite(rear) && rear>=0 && rear<=geometry.vehicle_length_m
      ? `後車軸から後端 ${number(rear,4)} m` : '自車の後端位置未取得（自車矩形なし）';
    view.geometryText=`寸法仮定: 自車 ${number(geometry.vehicle_length_m)} × ${number(geometry.vehicle_width_m)} m · 対象 ${dimensions} · 復帰余裕 ${number(geometry.merge_clearance_m)} m · ${rearText}`;
  } else view.geometryText='寸法・占有範囲の仮定: 未取得';
  const selected=path(status.path);
  if(status.valid && selected && status.mode!=='off' && status.state!=='FAULT')view.path=selected;
  if(status.mode==='off' || status.state==='FAULT')view.candidates=[];
  else view.object=object;
  if(status.state==='FAULT')view.candidatesText='左右候補: 異常のため非表示';
  return view;
}

export function renderBehavior(channel, connected, elapsedS=0) {
  const view=behaviorView(channel,connected,elapsedS);
  for(const [id,value] of [['status',view.message],['reason',view.reason],['source',view.source],
    ['target',view.target],['candidates',view.candidatesText],['geometry',view.geometryText],['context',view.context]])
    $(`behavior-${id}`).textContent=value;
  $('behavior-status').dataset.tone=view.tone;
  return view;
}

export function paintBehavior(ctx, project, line, channel, connected, elapsedS=0) {
  const view=behaviorView(channel,connected,elapsedS);
  let painted=false;
  for(const candidate of view.candidates) {
    if(candidate.path.length<2)continue;
    const color=candidate.side==='left' ? '#45a77b' : '#de9340';
    line(candidate.path,color,false,!candidate.feasible,2);painted=true;
    if(candidate.merge_start && candidate.merge_end) {
      line([candidate.merge_start,candidate.merge_end],color,false,true,3);
      for(const point of [candidate.merge_start,candidate.merge_end]) {
        const p=project(point);if(!finitePoint(p))continue;
        ctx.save();ctx.beginPath();ctx.arc(p[0],p[1],4,0,Math.PI*2);ctx.fillStyle=color;ctx.fill();ctx.restore();
      }
    }
  }
  if(view.path.length>=2) {line(view.path,'#168ddd',false,view.mode==='shadow',4);painted=true;}
  // Occupancy is an explicit rectangular assumption, aligned to the selected
  // route, not a measured object footprint or the car's current pose.
  if(view.geometry && view.path.length>=2) {
    const [first,next]=view.path, yaw=Math.atan2(next[1]-first[1],next[0]-first[0]);
    const rectangles=[], rear=view.geometry.rear_axle_to_rear_m;
    if(Number.isFinite(rear) && rear>=0 && rear<=view.geometry.vehicle_length_m) {
      const forward=view.geometry.vehicle_length_m/2-rear;
      rectangles.push([[first[0]+forward*Math.cos(yaw),first[1]+forward*Math.sin(yaw)],
        view.geometry.vehicle_length_m,view.geometry.vehicle_width_m,'#168ddd']);
    }
    if(view.object)rectangles.push([[view.object.x,view.object.y],view.geometry.target_length_m,
      view.geometry.target_width_m,view.object.kind==='rc_car' ? '#39dcb0' : '#d16ba5']);
    for(const [center,length,width,color] of rectangles) {
      const rectangle=[[-length/2,-width/2],[length/2,-width/2],[length/2,width/2],[-length/2,width/2]]
        .map(([x,y])=>[center[0]+x*Math.cos(yaw)-y*Math.sin(yaw),center[1]+x*Math.sin(yaw)+y*Math.cos(yaw)]);
      line(rectangle,color,true,true,2);painted=true;
    }
  }
  return painted;
}
