import assert from 'node:assert/strict';
import { AnnotationDraft, MAX_BOXES, normalizeBox, fitView, imagePoint, screenPoint,
  zoomView, boxHandles, hitBox, moveBox, resizeBox, filteredFrames, orderedFrames, projectProgress } from '../frontend/perception_model.js';

const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-9,`${a} != ${b}`);
const view=fitView(640,480,1000,500);
const p=screenPoint(view,123.25,234.5),original=imagePoint(view,...p);
near(original[0],123.25);near(original[1],234.5);
assert.ok(view.x>0,'letterboxing is included in the pixel transform');
const zoom=zoomView(view,2,p,view.scale);
const anchored=imagePoint(zoom,...p);near(anchored[0],123.25);near(anchored[1],234.5);
// Pointer coordinates use CSS pixels; DPR only controls the canvas backing store.
const cssRect={left:40,top:70,width:1000,height:500},client=[p[0]+40,p[1]+70];
assert.deepEqual(imagePoint(view,client[0]-cssRect.left,client[1]-cssRect.top),original);
assert.deepEqual(normalizeBox([700,490,-10,10],640,480),[0,10,640,480]);
assert.equal(normalizeBox([0,0,1,20],640,480),null);
assert.throws(()=>normalizeBox([0,0,NaN,10],640,480),/座標/);
const box=[20,30,120,130];
assert.deepEqual(moveBox(box,-100,1000,640,480),[0,380,100,480]);
assert.deepEqual(resizeBox(box,'se',[150,180],640,480),[20,30,150,180]);
assert.deepEqual(resizeBox(box,'nw',[-50,-50],640,480),[0,0,120,130]);
assert.equal(Object.keys(boxHandles(box)).length,8);
assert.deepEqual(hitBox([box],[20,30],0,6),{index:0,handle:'nw'});
assert.deepEqual(hitBox([box],[50,60],-1,6),{index:0,handle:null});
assert.equal(hitBox([box],[500,400],0,6),null);

const frame={width:640,height:480,reviewed:false,boxes:[],suggestions:[box]};
const draft=new AnnotationDraft(frame);
assert.deepEqual(draft.boxes,[],'automatic suggestions are never accepted by opening a frame');
assert.equal(draft.dirty,false);
draft.change([box]);assert.equal(draft.dirty,true);assert.equal(draft.reviewed,false);
draft.change([box],true);assert.equal(draft.reviewed,true);
draft.undo();assert.equal(draft.reviewed,false);
draft.undo();assert.equal(draft.dirty,false);assert.deepEqual(draft.boxes,[]);
draft.undo(true);assert.deepEqual(draft.boxes,[box]);assert.equal(draft.dirty,true);
draft.discard();assert.equal(draft.dirty,false);
draft.change([],true);assert.equal(draft.reviewed,true);assert.equal(draft.dirty,true,'reviewed negative is distinct from an untouched empty frame');
const before=draft.snapshot();assert.throws(()=>draft.change(Array.from({length:MAX_BOXES+1},()=>box)),/100/);
assert.deepEqual(draft.snapshot(),before,'a rejected oversized annotation keeps the current draft');
const reviewed=new AnnotationDraft({...frame,boxes:[box],reviewed:true});
reviewed.change([[21,30,121,130]]);assert.equal(reviewed.reviewed,false,'editing an approved frame needs a new explicit review');
reviewed.undo();assert.equal(reviewed.reviewed,true);assert.equal(reviewed.dirty,false);

const frames=[{id:'a',split:'train',reviewed:false,boxes:[]},
  {id:'b',split:'train',reviewed:true,boxes:[box]},{id:'c',split:'val',reviewed:true,boxes:[]}];
assert.deepEqual(filteredFrames(frames,'negative').map(f=>f.id),['c']);
assert.deepEqual(filteredFrames(frames,'unreviewed').map(f=>f.id),['a']);
assert.deepEqual(filteredFrames(frames,'reviewed','train').map(f=>f.id),['b']);
assert.deepEqual(projectProgress(frames),{total:3,reviewed:2,positive:1,negative:1,train:2,val:1});
const ranking={stale:false,annotations_changed:true,project_revision:1,
  ranked:[{frame_id:'c',rank:2},{frame_id:'b',rank:1},{frame_id:'b',rank:3},{frame_id:'unknown',rank:4}]};
assert.deepEqual(orderedFrames(frames,ranking,true).map(f=>f.id),['b','c','a'],'priority ordering keeps negative, val and unranked frames and ignores duplicate/unknown IDs');
assert.deepEqual(orderedFrames(frames,{...ranking,stale:true},true),frames,'a stale image selection never changes navigation order');
assert.deepEqual(orderedFrames(frames,ranking,false),frames,'ordinary navigation is always available');
console.log('RC-car annotation geometry, zoom/CSS coordinates, bounded undo, explicit negatives and review filters passed');
