const {test}=require('node:test');
const assert=require('node:assert/strict');
const {estimateResponseDelay}=require('./response_delay.js');
function data({delay=.12,changeGain=false,gap=false,ramp=false,periodic=false}={}){
 const command=[];for(let i=0;i<=2000;i++){
  const t=i/100;if(gap&&t>4&&t<5)continue;
  const value=ramp?t/20:periodic?Math.sin(2*Math.PI*t/.2):.55*Math.sin(i/23)+.3*Math.sin(i/71);
  command.push([t,value]);
 }
 const sensor=[];let j=0;
 for(let i=0;i<=4000;i++){
  const t=i/200,q=t-delay;while(j+1<command.length&&command[j+1][0]<=q+1e-12)j++;
  sensor.push([t,.06+(changeGain&&t>14?2:-3.2)*command[j][1]+(ramp?0:.008*Math.sin(i*1.31))]);
 }
 return {command,sensor};
}
const options={start:1,end:19,maxLagMs:500};
test('known negative gain and bias recover delay with held-out validation, without mutation',()=>{
 const {command,sensor}=data(),before=JSON.stringify([command,sensor]);
 const result=estimateResponseDelay(command,sensor,options);
 assert.equal(result.status,'candidate');assert.ok(Math.abs(result.tau_ms-120)<=5);
 assert.ok(Math.abs(result.gain+3.2)<.02);assert.ok(Math.abs(result.bias-.06)<.005);
 assert.ok(result.train_r2>.99&&result.validation_r2>.99);assert.ok(result.train_correlation<-.99);
 assert.equal(result.near_minimum_is_confidence_interval,false);
 assert.equal(JSON.stringify([command,sensor]),before);
});
test('constant inputs and constant response cannot identify delay',()=>{
 const {command,sensor}=data();
 assert.equal(estimateResponseDelay(command.map(p=>[p[0],.2]),sensor,options).status,'unavailable');
 assert.equal(estimateResponseDelay(command,sensor.map(p=>[p[0],0]),options).status,'unavailable');
});
test('pure ramp with free bias is ambiguous, never a confident delay estimate',()=>{
 const {command,sensor}=data({ramp:true});
 const result=estimateResponseDelay(command,sensor,options);
 assert.equal(result.status,'unstable');
 assert.deepEqual(result.near_minimum_range_ms,[0,500]);
 assert.ok(result.warnings.some(v=>v.includes('谷')));
});
test('periodic ambiguity and boundary solutions are flagged',()=>{
 const periodic=data({periodic:true});
 assert.equal(estimateResponseDelay(periodic.command,periodic.sensor,options).status,'unstable');
 const zero=data({delay:0});const result=estimateResponseDelay(zero.command,zero.sensor,options);
 assert.equal(result.tau_ms,0);assert.equal(result.status,'unstable');
 assert.ok(result.warnings.some(v=>v.includes('探索端')));
});
test('changed dynamics fail on held-out data',()=>{
 const {command,sensor}=data({changeGain:true});const result=estimateResponseDelay(command,sensor,options);
 assert.equal(result.status,'unstable');assert.ok(result.validation_r2<.3);
 assert.ok(result.warnings.some(v=>v.includes('検証区間')));
});
test('command gaps are excluded at all candidate lags, with no endpoint extrapolation',()=>{
 const {command,sensor}=data({gap:true});const result=estimateResponseDelay(command,sensor,{start:0,end:20,maxLagMs:500});
 assert.ok(Math.abs(result.tau_ms-120)<=5);assert.ok(result.excluded_coverage_samples>250);
 assert.ok(result.effective_range_s[0]>=.5);
 assert.ok(result.predictions.every(p=>!(p[0]>4&&p[0]<5.5)));
});
test('bad bounds, short data and reused timestamps are refused',()=>{
 const {command,sensor}=data();
 for(const overrides of [{maxLagMs:0},{maxLagMs:3000},{start:NaN},{end:0}])
  assert.equal(estimateResponseDelay(command,sensor,{...options,...overrides}).status,'unavailable');
 assert.equal(estimateResponseDelay(command,sensor,{start:1,end:1.1}).status,'unavailable');
 command[2][0]=command[1][0];assert.equal(estimateResponseDelay(command,sensor,options).status,'unavailable');
});

test('report controls fit the selected axis/range and invalidate stale predictions/exports',()=>{
 const fs=require('node:fs'),vm=require('node:vm');const {command,sensor}=data();
 const element=(value='')=>({value,disabled:false,hidden:false,listeners:{},addEventListener(name,fn){this.listeners[name]=fn;}});
 const ids=['response-fit','response-fit-result','response-fit-save','response-fit-json','response-fit-cost',
            'response-gyro','start','end','response-fit-max'];
 const elements=Object.fromEntries(ids.map(id=>[id,element()]));
 elements['response-gyro'].value='2';elements.start.value='1';elements.end.value='19';elements['response-fit-max'].value='500';
 let plotted=null,restyled=null;const callbacks={};const chart={on(name,fn){callbacks[name]=fn;}};
 const context=vm.createContext({document:{getElementById:id=>elements[id]},
   Plotly:{restyle(chart,changes,indices){restyled={changes,indices};},newPlot(chart,curves){plotted=curves;}},JSON,Number,Object,String});
 vm.runInContext(fs.readFileSync(require.resolve('./response_delay.js'),'utf8')+'\n'+fs.readFileSync(require.resolve('./response_delay_ui.js'),'utf8'),context);
 context.bindResponseDelayUI(chart,{}, {command:{name:'/vehicle/control_cmd steering',points:command},
   gyro:{'2':{name:'/realsense/imu gyro_y',points:sensor}},model_index:8,duration_s:20,origin_bag_ns:'1790000000000000000'});
 elements['response-fit'].onclick();
 assert.ok(elements['response-fit-result'].textContent.includes('120 ms'));
 assert.equal(elements['response-fit-save'].disabled,false);
 const result=JSON.parse(elements['response-fit-json'].value);
 assert.equal(result.sensor,'/realsense/imu gyro_y');assert.ok(result.gain<0&&result.validation_r2>.99);
 assert.equal(result.origin_bag_ns,'1790000000000000000');assert.equal(result.predictions,undefined);
 assert.ok(plotted[0].x.length===101&&restyled.changes.visible===true);
 elements['response-gyro'].listeners.change();assert.equal(elements['response-fit-save'].disabled,true);
 assert.equal(restyled.changes.visible,false);
 callbacks.plotly_relayout({'xaxis.range[0]':4,'xaxis.range[1]':8});
 assert.equal(elements.start.value,4);assert.equal(elements.end.value,8);
 assert.equal(elements['response-fit-json'].value,'');
});
