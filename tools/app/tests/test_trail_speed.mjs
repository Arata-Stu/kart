import assert from 'node:assert/strict';
import {paintTrailSpeed} from '../frontend/trail_speed.js';
const gradients=[], strokes=[];
const ctx={save(){},restore(){},setLineDash(){},beginPath(){},moveTo(){},lineTo(){},
  stroke(){strokes.push(this.strokeStyle);},createLinearGradient(){const colors=[];gradients.push(colors);return {addColorStop(t,c){colors.push([t,c]);}};}};
paintTrailSpeed(ctx,[[0,0,0],[1,1,3],null,[2,2,1],[3,3,null]],p=>p.slice(0,2),3);
assert.equal(strokes.length,2,'no line across missing interval');
assert.equal(gradients.length,1);
assert.equal(gradients[0][0][1],'hsl(225,75%,43%)');
assert.equal(gradients[0].at(-1)[1],'hsl(0,75%,43%)');
assert.equal(strokes[1],'#88919b','unknown speed is gray, not zero');
console.log('Measured speed gradient, unknown values and trajectory gaps passed');
