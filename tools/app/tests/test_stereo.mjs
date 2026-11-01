import assert from 'node:assert/strict';
import {stereoState,setStereo} from '../frontend/stereo.js';
const defaults={profile:'424x240x60',infra_profile:'424x240x60',depth_profile:'0'};
assert.equal(stereoState({},defaults).depthOn,false);
assert.equal(stereoState({}, {...defaults,depth_profile:'424x240x60'}).depthOn,true);
const saved={mode:'collect',infra_profile:'424x240x90',depth_profile:'848x480x30'};
assert.equal(stereoState(saved,defaults).conflict,true);
assert.equal(saved.depth_profile,'848x480x30');
for(const infraOn of [true,false])for(const depthOn of [true,false]) {
 const values={infra_fps:'90',rgb_profile:'640x480x30'};
 setStereo(values,{profile:'424x240x60',infraOn,depthOn});
 assert.equal(values.infra_profile,infraOn?'424x240x60':'0');
 assert.equal(values.depth_profile,depthOn?'424x240x60':'0');
 assert.equal(values.infra_fps,undefined);
 assert.equal(values.rgb_profile,'640x480x30');
 assert.equal(stereoState(values,defaults).conflict,false);
}
console.log('Stereo: defaults, saved conflicts, independent enable flags and shared profile passed');
