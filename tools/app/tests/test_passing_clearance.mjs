import assert from 'node:assert/strict';
import { passingSection } from '../frontend/passing_clearance.js';

const loop={left:[[0,1],[10,1],[10,8],[0,8]],right:[[-2,-1],[12,-1],[12,10],[-2,10]],obstacles:[]};
const inspect=(offset, lane=loop)=>passingSection([5,0],[1,0],lane,.19,offset,.08);
const left=inspect(.65);
assert(left.usable && left.fits);
assert.equal(left.available,1);
assert(Math.abs(left.needed-.825)<1e-9);
assert(Math.abs(left.between-.46)<1e-9);
assert(!inspect(.9).fits);
assert(!inspect(.20).fits);  // cars fit inside the lane, but the gap is too narrow
const box={polygon:[[4,.5],[6,.5],[6,.9],[4,.9]]};
const withBox={...loop,obstacles:[box]};
assert.equal(inspect(.65,withBox).available,.5);
assert(!inspect(.65,withBox).fits);
assert(inspect(-.65,withBox).fits);
assert(!passingSection([5,.6],[1,0],withBox,.19,.65,.08).usable);
assert(!passingSection([5,1],[1,0],loop,.19,.65,.08).usable);
assert(!passingSection([5,4],[1,0],loop,.19,.65,.08).usable);
console.log('Passing clearance: track edges, opposite side, box, vehicle and margin verified');
