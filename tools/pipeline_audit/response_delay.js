/* Apparent steering response delay, recorder-time basis. No clock correction.
 * y(t) = bias + gain * steering(t - tau). Gain sign is fitted, not forced.
 * Pure function shared by standalone reports and portable Node tests.
 */
function estimateResponseDelay(command, sensor, options={}) {
  const settings={start:options.start, end:options.end, maxLagMs:options.maxLagMs??500,
    stepMs:5, trainFraction:.7, minSamples:80, minDurationS:1,
    minInputStd:.03, minOutputStd:.02, maxSamples:100000};
  const fail=reason=>({schema:'kart.response_delay.v1',status:'unavailable',reason,settings});
  if(!Number.isFinite(settings.start)||!Number.isFinite(settings.end)||settings.end<=settings.start||
     !Number.isFinite(settings.maxLagMs)||settings.maxLagMs<10||settings.maxLagMs>2000)
    return fail('区間と探索上限（10..2000 ms）を確認してください');
  const valid=points=>Array.isArray(points)&&points.length>1&&points.every((p,i)=>
    Array.isArray(p)&&p.length===2&&p.every(Number.isFinite)&&(i===0||p[0]>points[i-1][0]));
  if(!valid(command)||!valid(sensor))return fail('指令／IMUの時刻が欠損・重複・逆行、または値が不正です');
  const lagCount=Math.floor(settings.maxLagMs/settings.stepMs);
  // Fixed grid, never claim sub-grid timing precision.
  const maxLag=lagCount*settings.stepMs/1000;
  const intervals=command.slice(1).map((p,i)=>p[0]-command[i][0]).sort((a,b)=>a-b);
  const gapLimit=Math.max(.05,intervals[Math.floor(intervals.length/2)]*3);
  settings.commandGapLimitS=gapLimit;
  const segments=[];let begin=command[0][0];
  for(let i=1;i<command.length;i++)if(command[i][0]-command[i-1][0]>gapLimit){
    segments.push([begin,command[i-1][0]]);begin=command[i][0];
  }
  segments.push([begin,command.at(-1)[0]]);
  // Identical response samples at every candidate lag. No endpoint extrapolation,
  // no interpolation across command gaps, no lag-dependent sample count advantage.
  const selected=[];let segment=0,candidates=0;
  for(const p of sensor){
    if(p[0]<settings.start||p[0]>settings.end)continue;
    candidates++;
    while(segment<segments.length&&segments[segment][1]<p[0])segment++;
    if(segment<segments.length&&p[0]-maxLag>=segments[segment][0])selected.push(p);
  }
  if(selected.length<settings.minSamples||selected.at(-1)[0]-selected[0][0]<settings.minDurationS)
    return fail('連続した比較データが不足しています。1秒以上で操舵が変化する区間を選んでください');
  if(selected.length>settings.maxSamples)return fail('比較区間が長すぎます。IMU 100000点以内の区間に絞ってください');
  const split=Math.floor(selected.length*settings.trainFraction);
  const variance=values=>{const mean=values.reduce((a,b)=>a+b,0)/values.length;return values.reduce((a,b)=>a+(b-mean)**2,0)/values.length;};
  if(Math.sqrt(variance(selected.slice(0,split).map(p=>p[1])))<settings.minOutputStd)
    return fail('角速度の変化が小さく、遅れを推定できません');
  function inputsAt(tau){
    let index=0;const values=[];
    for(const p of selected){const query=p[0]-tau;
      while(index+1<command.length&&command[index+1][0]<=query+1e-12)index++;
      values.push(command[index][1]);
    }
    return values;
  }
  function regress(x){
    let sx=0,sy=0,sxx=0,syy=0,sxy=0;
    for(let i=0;i<split;i++){const y=selected[i][1];sx+=x[i];sy+=y;sxx+=x[i]*x[i];syy+=y*y;sxy+=x[i]*y;}
    const xx=Math.max(0,sxx-sx*sx/split),yy=Math.max(0,syy-sy*sy/split),xy=sxy-sx*sy/split;
    if(xx/split<settings.minInputStd**2||yy<=0)return null;
    const gain=xy/xx,bias=(sy-gain*sx)/split,mse=Math.max(0,yy-xy*xy/xx)/split;
    return {gain,bias,mse,r2:1-mse/(yy/split),correlation:xy/Math.sqrt(xx*yy),variance:yy/split};
  }
  let best=null;const curve=[];
  for(let i=0;i<=lagCount;i++){
    const tau=i*settings.stepMs/1000,x=inputsAt(tau),fit=regress(x);
    curve.push({tau_ms:i*settings.stepMs,rmse:fit?Math.sqrt(fit.mse):null});
    if(fit&&(!best||fit.mse<best.mse))best={...fit,tau_ms:i*settings.stepMs,tau,x};
  }
  if(!best)return fail('操舵指令の変化が小さく、遅れとゲインを分離できません');
  const test=selected.slice(split),testVariance=variance(test.map(p=>p[1]));
  const validationMse=test.reduce((sum,p,i)=>sum+(p[1]-best.bias-best.gain*best.x[split+i])**2,0)/test.length;
  const validationR2=testVariance>=settings.minOutputStd**2?1-validationMse/testVariance:null;
  const near=curve.filter(p=>p.rmse!==null&&p.rmse**2<=best.mse+.02*best.variance);
  const nearRange=[near[0].tau_ms,near.at(-1).tau_ms];
  const warnings=[];
  if(best.tau_ms===0||best.tau_ms===lagCount*settings.stepMs)warnings.push('探索端が最良です。遅れを範囲内で特定できていない可能性があります');
  if(best.r2<.5)warnings.push('学習区間の当てはまりが弱く、単純な遅延モデルで説明できません');
  if(validationR2===null)warnings.push('検証区間の角速度変化が不足しています');
  else if(validationR2<.3)warnings.push('後半の検証区間で再現できません。速度変化や車体応答を確認してください');
  if(nearRange[1]-nearRange[0]>Math.max(50,settings.maxLagMs*.3))warnings.push('誤差曲線の谷が広い／複数候補があり、推定が不安定です');
  const predictions=selected.map((p,i)=>[p[0],best.bias+best.gain*best.x[i]]);
  return {schema:'kart.response_delay.v1',status:warnings.length?'unstable':'candidate',
    model:'angular(t) = bias + gain * steering(t - tau)',time_basis:'bag recorder time',
    tau_ms:best.tau_ms,gain:best.gain,K:-best.gain,bias:best.bias,
    train_r2:best.r2,train_rmse:Math.sqrt(best.mse),train_correlation:best.correlation,
    validation_r2:validationR2,validation_rmse:Math.sqrt(validationMse),
    requested_range_s:[settings.start,settings.end],effective_range_s:[selected[0][0],selected.at(-1)[0]],
    validation_start_s:test[0][0],train_samples:split,validation_samples:test.length,
    excluded_coverage_samples:candidates-selected.length,
    near_minimum_range_ms:nearRange,near_minimum_is_confidence_interval:false,
    warnings,settings,curve,predictions,
    assumptions:['Speed is approximately constant; fixed throttle does not verify speed.',
      'Raw IMU sensor axis and normalized steering request, not calibrated vehicle yaw or actual steering angle.',
      'Delay includes dynamics, command holding and recorder skew, not isolated transport latency.',
      'No timestamp correction or source-data shift is applied. Negative gain is allowed.']};
}
if(typeof module!=='undefined'&&module.exports)module.exports={estimateResponseDelay};
