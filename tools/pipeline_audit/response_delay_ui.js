/* Bind the estimator to a standalone report. No backend or timestamp mutation. */
function bindResponseDelayUI(chart, figure, payload) {
  const button=document.getElementById('response-fit');
  if(!button||!payload)return;
  const result=document.getElementById('response-fit-result');
  const save=document.getElementById('response-fit-save');
  const details=document.getElementById('response-fit-json');
  const cost=document.getElementById('response-fit-cost');
  const gyro=document.getElementById('response-gyro');
  const start=document.getElementById('start'),end=document.getElementById('end');
  const upper=document.getElementById('response-fit-max');
  let saved=null;
  function invalidate(){
    saved=null;save.disabled=true;details.value='';cost.hidden=true;
    result.textContent='区間と角速度軸を選び「応答遅れを推定」を押してください';
    Plotly.restyle(chart,{visible:false},[payload.model_index]);
  }
  for(const element of [start,end,upper])element.addEventListener('input',invalidate);
  gyro.addEventListener('change',invalidate);
  if(typeof chart.on==='function')chart.on('plotly_relayout',event=>{
    let range=null;
    for(const key of Object.keys(event)){
      if(/^xaxis\d*\.range$/.test(key)&&Array.isArray(event[key]))range=event[key];
      if(/^xaxis\d*\.range\[0\]$/.test(key)){
        const other=key.replace('[0]','[1]');
        if(Number.isFinite(event[other]))range=[event[key],event[other]];
      }
      if(/^xaxis\d*\.autorange$/.test(key)&&event[key]===true)range=[0,payload.duration_s];
    }
    if(range&&range.every(Number.isFinite)&&range[0]<range[1]){
      const changed=Number(start.value)!==range[0]||Number(end.value)!==range[1];
      start.value=range[0];end.value=range[1];if(changed)invalidate();
    }
  });
  button.onclick=()=>{
    invalidate();
    const sensor=payload.gyro[gyro.value];
    if(!sensor){result.textContent='推定対象の角速度を選択してください';return;}
    const estimate=estimateResponseDelay(payload.command.points,sensor.points,
      {start:Number(start.value),end:Number(end.value),maxLagMs:Number(upper.value)});
    if(estimate.status==='unavailable'){result.textContent='推定不可: '+estimate.reason;return;}
    const {predictions,...metrics}=estimate;
    saved={...metrics,command:payload.command.name,sensor:sensor.name,bag:payload.bag,
           origin_bag_ns:payload.origin_bag_ns,provenance:payload.provenance};
    const fmt=value=>value===null?'—':value.toFixed(3);
    result.textContent=(estimate.status==='candidate'?'モデル上の候補':'不安定・参考値')+
      ': τ ≈ '+estimate.tau_ms+' ms（探索刻み5 ms） · G = '+fmt(estimate.gain)+
      ' · b = '+fmt(estimate.bias)+' rad/s · R²(適合) = '+fmt(estimate.train_r2)+
      ' · R²(後半検証) = '+fmt(estimate.validation_r2)+
      ' · 最小誤差近傍 '+estimate.near_minimum_range_ms.join('–')+' ms（信頼区間ではありません）'+
      (estimate.warnings.length?' · '+estimate.warnings.join(' / '):'');
    details.value=JSON.stringify(saved,null,2);save.disabled=false;
    // Keep the prediction on the same sensor axis; never move observed data.
    const xs=[],ys=[];let previous=null;
    const gaps=sensor.points.slice(1).map((p,i)=>p[0]-sensor.points[i][0]).sort((a,b)=>a-b);
    const gapLimit=Math.max(.1,3*gaps[Math.floor(gaps.length/2)]);
    for(const p of predictions){if(previous!==null&&p[0]-previous>gapLimit){xs.push(null);ys.push(null);}
      xs.push(p[0]);ys.push(p[1]);previous=p[0];}
    Plotly.restyle(chart,{x:[xs],y:[ys],visible:true},[payload.model_index]);
    cost.hidden=false;
    Plotly.newPlot(cost,[{x:estimate.curve.map(p=>p.tau_ms),y:estimate.curve.map(p=>p.rmse),
      type:'scatter',mode:'lines',name:'適合区間RMSE'}],{
        height:250,margin:{l:70,r:25,t:35,b:55},template:'plotly_white',
        title:'探索した遅れとモデル誤差（谷の形も確認）',
        xaxis:{title:'仮定した τ [ms]'},yaxis:{title:'角速度 RMSE [rad/s]'},
        shapes:[{type:'line',xref:'x',yref:'paper',x0:estimate.tau_ms,x1:estimate.tau_ms,y0:0,y1:1,line:{dash:'dot',color:'#8a3fa0'}}]
      },{responsive:true,displaylogo:false});
  };
  save.onclick=()=>{
    if(!saved)return;
    const url=URL.createObjectURL(new Blob([JSON.stringify(saved,null,2)+'\n'],{type:'application/json'}));
    const link=document.createElement('a');link.href=url;link.download='response-delay-estimate.json';
    document.body.append(link);link.click();link.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);
  };
}
