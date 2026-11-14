import assert from 'node:assert/strict';
import {snapshotCustom, loadCustomVersion, nextVersionId} from '../frontend/custom_versions_model.js';
import {reverseLane} from '../frontend/direction.js';
import {invalidateLines} from '../frontend/api.js';

const lane = {closed:false, left:[[0,1],[5,1]], right:[[0,-1],[5,-1]], custom:[[0,0],[5,0]],
  custom_speeds:[{start:1,end:3,speed:.4}], custom_speed_mode:'step', lines:{customline:{points:[[0,0],[5,0]]}}};
const first = snapshotCustom(lane, nextVersionId(lane), '守り');
lane.custom_versions = [first];
lane.custom[0][0] = 1;
assert.equal(first.custom[0][0], 0, 'saved versions must not share editing arrays');
assert.equal(nextVersionId(lane), 'custom_002');
loadCustomVersion(lane, first.id);
assert.equal(lane.custom[0][0], 0);
assert.equal(lane.custom_speed_mode, 'step');
lane.lines.customline.points[0][0] = 99;
assert.equal(first.line.points[0][0], 0);
reverseLane(lane);
assert.deepEqual(first.custom, [[5,0],[0,0]]);
assert.deepEqual(first.custom_speeds, [{start:2,end:4,speed:.4}]);
assert.equal(first.line, undefined, 'direction changes invalidate saved generated lines');
assert.throws(() => snapshotCustom(lane, 'id', '  '));
assert.throws(() => loadCustomVersion(lane, 'absent'));
first.line = {points:[[5,0],[0,0]]};
invalidateLines(lane);
assert.equal(first.line, undefined);
assert.deepEqual(first.custom, [[5,0],[0,0]], 'boundary edits preserve saved controls for regeneration');
console.log('custom version copy, restore and reverse tests passed');
