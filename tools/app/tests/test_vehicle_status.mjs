import assert from 'node:assert/strict';
const elements=new Map();
globalThis.document={getElementById(id){if(!elements.has(id))elements.set(id,{dataset:{}});return elements.get(id);}};
const {renderVehicle}=await import('../frontend/vehicle_status.js');
const el=id=>elements.get('vehicle-'+id);
const status={mode:1,vehicle:{bridge:{availability:'fresh',level:2,message:'USB serial disconnected',values:{host_active:'false',steering_offset:'0.01',steering_trim:'-0.043'}},recovery:{availability:'fresh',state:'suspect',reason:'low_flow',attempts:1}}};
renderVehicle(status);
assert.equal(el('mode').textContent,'AUTO');
assert.match(el('host').textContent,/無効/);
assert.match(el('recovery').textContent,/疑い/);
assert.equal(el('health').dataset.tone,'error');
assert.match(el('calibration').textContent,/0.01/);
for(const [state,label] of [['post_reverse_pulse','制動準備'],['post_reverse_braking','後退後の制動'],['reversing','後退中'],['braking','制動中'],['fault','中断']]){
 status.vehicle.recovery.state=state;renderVehicle(status);assert.match(el('recovery').textContent,new RegExp(label));
}
status.vehicle.recovery={availability:'stale'};renderVehicle(status);
assert.match(el('recovery').textContent,/受信停止/);assert.doesNotMatch(el('recovery').textContent,/後退中/);
renderVehicle(null);assert.equal(el('mode').textContent,'モード不明');assert.match(el('host').textContent,/未接続/);
console.log('Vehicle panel separates mode, authority, inferred contact, recovery, current faults and stale data');
