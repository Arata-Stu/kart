import assert from 'node:assert/strict';
import {pipelineRows,pipelineSteps,isMulti} from '../frontend/learning_pipeline.js';
const catalog={datasets:[{id:'raw',pipeline:'shared'},{id:'expert',routing_sha256:'route'},{id:'stable',pipeline:'multi'}],
 runs:[{id:'single',pipeline:'single'},{id:'joint',section_heads:['curve','stable']}],
 models:[{id:'old-single'},{id:'joint',routing:{heads:['curve','stable']}}],encoders:[{id:'tiny'}]};
assert.deepEqual(pipelineRows(catalog,'datasets','single').map(r=>r.id),['raw']);
assert.deepEqual(pipelineRows(catalog,'datasets','multi').map(r=>r.id),['expert','stable']);
assert.deepEqual(pipelineRows(catalog,'datasets','multi',{teacher:true}).map(r=>r.id),['raw']);
assert.deepEqual(pipelineRows(catalog,'runs','single').map(r=>r.id),['single']);
assert.deepEqual(pipelineRows(catalog,'models','multi').map(r=>r.id),['joint']);
assert.equal(pipelineRows(catalog,'encoders','multi').length,1);
assert.ok(!pipelineSteps('single').includes('sections'));
assert.ok(!pipelineSteps('multi').includes('features'));
assert.equal(pipelineSteps('multi')[0],'sections');
assert.ok(isMulti({routing:{heads:['one']}}));
console.log('Learning pipelines: passed');
