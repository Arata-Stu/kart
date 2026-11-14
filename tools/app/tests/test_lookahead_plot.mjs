import assert from 'node:assert/strict';
import {lookaheadPlot,lookaheadValue} from '../frontend/lookahead_plot.js';

const row=(t,values={})=>({t,controller:'pure_pursuit',lookahead_mode:'speed_interpolated',
  lookahead_m:.8,target_distance_m:.75,measured_speed_mps:1,target_speed_mps:2,...values});
const moves=path=>(path.match(/M/g)||[]).length;
const lines=path=>(path.match(/L/g)||[]).length;
let plot=lookaheadPlot([row(1),row(1.1),row(1.2,{lookahead_m:null}),row(1.3),row(1.8)],2);
assert.equal(moves(plot.distance.paths.lookahead_m),3);
assert.equal(lines(plot.distance.paths.lookahead_m),1);
assert.equal(moves(plot.distance.paths.target_distance_m),2,'one missing distance does not hide the other series');
assert.equal(lines(plot.speed.paths.measured_speed_mps),3,'a distance gap does not interrupt valid speed');
assert.equal(moves(lookaheadPlot([row(1),row(1.5)],2).speed.paths.measured_speed_mps),2,'exactly 0.5 seconds breaks the line');
assert.equal(moves(lookaheadPlot([row(.9),row(1.4)],2).speed.paths.measured_speed_mps),2,'decimal timestamp rounding does not connect a half-second gap');

plot=lookaheadPlot([row(1,{measured_speed_mps:null,target_speed_mps:Infinity}),row(1.1,{measured_speed_mps:-.4})],2);
assert.equal(lines(plot.speed.paths.measured_speed_mps),0);
assert.equal(moves(plot.speed.paths.measured_speed_mps),1);
assert.ok(plot.speed.min<=-.4,'signed reverse speed is retained');
assert.ok(plot.speed.max>=2);
assert.equal(lookaheadValue(row(1,{measured_speed_mps:null}),'measured_speed_mps'),null,'null never becomes zero');
assert.equal(lookaheadValue(row(1,{lookahead_m:-1}),'lookahead_m'),null,'a negative distance is invalid');
assert.ok(!plot.speed.paths.target_speed_mps.includes('NaN'));

plot=lookaheadPlot([row(1),row(1.1,{lookahead_mode:'fixed'}),row(1.2,{controller:'mpc',lookahead_mode:null}),
  row(1.3,{controller:'mpc',lookahead_mode:null})],2);
assert.equal(moves(plot.distance.paths.lookahead_m),3,'controller and mode changes start new segments');
assert.equal(lines(plot.distance.paths.lookahead_m),1,'MPC allows an absent PP mode');
assert.equal(moves(plot.speed.paths.target_speed_mps),3);
assert.equal(lookaheadPlot([row(1,{controller:'unknown'})],2).distance.paths.lookahead_m,'');
assert.equal(lookaheadPlot([row(1,{lookahead_mode:null})],2).distance.paths.lookahead_m,'','old PP diagnostics without mode are unsupported');

plot=lookaheadPlot([row(1.1),row(1),row(1.1,{lookahead_m:null}),row(2)],2);
assert.equal(plot.rows.length,3,'duplicate timestamps use the latest observation');
assert.equal(plot.rows[1].lookahead_m,null,'the latest invalid observation remains a gap');
assert.equal(moves(plot.distance.paths.lookahead_m),2);
assert.ok(plot.rows.every((point,index,rows)=>index===0 || point.t>rows[index-1].t));
assert.equal(lookaheadPlot([row(0),row(1),row(61),row(62),null],61).rows.length,2,'window includes its start and excludes future samples');
assert.equal(lookaheadPlot([row(0)],61).distance.paths.lookahead_m,'');
assert.equal(lookaheadPlot([row(1)],NaN).distance.paths.lookahead_m,'');
assert.equal(lookaheadPlot(Array.from({length:1000},(_,i)=>row(i/100)),10).rows.length,650,'rendering is bounded even for malformed oversized snapshots');
assert.equal(plot.x(2),748);
assert.equal(plot.x(-58),48,'both charts share the same time coordinates');
assert.ok(Number.isFinite(lookaheadPlot([],0).distance.y(0)));
console.log('Lookahead plots preserve signed values, independent gaps, mode boundaries and bounded time windows');
