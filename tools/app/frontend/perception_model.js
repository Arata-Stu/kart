export const MAX_BOXES=100;
export const MIN_BOX_SIZE=2;
export const copyBoxes=boxes=>(boxes || []).map(box=>box.slice());
const clamp=(value,low,high)=>Math.max(low,Math.min(high,value));

export function normalizeBox(box,width,height) {
  if(!Array.isArray(box) || box.length!==4 || !box.every(Number.isFinite))throw Error('矩形の座標が不正です');
  const [x1,x2]=[clamp(box[0],0,width),clamp(box[2],0,width)].sort((a,b)=>a-b);
  const [y1,y2]=[clamp(box[1],0,height),clamp(box[3],0,height)].sort((a,b)=>a-b);
  if(x2-x1<MIN_BOX_SIZE || y2-y1<MIN_BOX_SIZE)return null;
  return [x1,y1,x2,y2];
}
export function fitView(width,height,viewportWidth,viewportHeight) {
  const scale=Math.min(viewportWidth/width,viewportHeight/height);
  return {scale,x:(viewportWidth-width*scale)/2,y:(viewportHeight-height*scale)/2};
}
export const imagePoint=(view,x,y)=>[(x-view.x)/view.scale,(y-view.y)/view.scale];
export const screenPoint=(view,x,y)=>[x*view.scale+view.x,y*view.scale+view.y];
export function zoomView(view,factor,point,fitScale) {
  const position=imagePoint(view,...point),scale=clamp(view.scale*factor,fitScale*.25,fitScale*8);
  return {scale,x:point[0]-position[0]*scale,y:point[1]-position[1]*scale};
}
export function boxHandles(box) {
  const [x1,y1,x2,y2]=box,x=(x1+x2)/2,y=(y1+y2)/2;
  return {nw:[x1,y1],n:[x,y1],ne:[x2,y1],e:[x2,y],se:[x2,y2],s:[x,y2],sw:[x1,y2],w:[x1,y]};
}
export function hitBox(boxes,point,selected=-1,tolerance=6) {
  if(boxes[selected])for(const [handle,p] of Object.entries(boxHandles(boxes[selected])))
    if(Math.abs(point[0]-p[0])<=tolerance && Math.abs(point[1]-p[1])<=tolerance)return {index:selected,handle};
  for(let index=boxes.length-1;index>=0;index--) {
    const [x1,y1,x2,y2]=boxes[index];
    if(point[0]>=x1 && point[0]<=x2 && point[1]>=y1 && point[1]<=y2)return {index,handle:null};
  }
  return null;
}
export function moveBox(box,dx,dy,width,height) {
  dx=clamp(dx,-box[0],width-box[2]);dy=clamp(dy,-box[1],height-box[3]);
  return [box[0]+dx,box[1]+dy,box[2]+dx,box[3]+dy];
}
export function resizeBox(box,handle,point,width,height) {
  const result=box.slice();
  if(handle.includes('w'))result[0]=point[0];
  if(handle.includes('e'))result[2]=point[0];
  if(handle.includes('n'))result[1]=point[1];
  if(handle.includes('s'))result[3]=point[1];
  return normalizeBox(result,width,height);
}
export function filteredFrames(frames,filter='all',split='all') {
  return (frames || []).filter(frame=>(split==='all' || frame.split===split) &&
    (filter==='all' || filter==='unreviewed' && !frame.reviewed || filter==='reviewed' && frame.reviewed ||
      filter==='positive' && frame.reviewed && frame.boxes.length>0 ||
      filter==='negative' && frame.reviewed && frame.boxes.length===0));
}
export function orderedFrames(frames,selection,enabled=false) {
  if(!enabled || selection?.stale || !Array.isArray(selection?.ranked))return frames.slice();
  const available=new Map(frames.map(frame=>[frame.id,frame])),ordered=[];
  const ranked=selection.ranked.map((row,index)=>({row,index})).sort((a,b)=>
    (Number.isFinite(a.row.rank)?a.row.rank:a.index+1)-(Number.isFinite(b.row.rank)?b.row.rank:b.index+1));
  for(const {row} of ranked)if(available.has(row.frame_id)) {
    ordered.push(available.get(row.frame_id));available.delete(row.frame_id);
  }
  // Selection only changes navigation order; unranked/negative/val images stay available.
  return [...ordered,...frames.filter(frame=>available.has(frame.id))];
}
export function projectProgress(frames) {
  const progress={total:frames.length,reviewed:0,positive:0,negative:0,train:0,val:0};
  for(const frame of frames) {
    if(frame.split in progress)progress[frame.split]++;
    if(frame.reviewed) {progress.reviewed++;progress[frame.boxes.length?'positive':'negative']++;}
  }
  return progress;
}

export class AnnotationDraft {
  constructor(frame) {
    this.width=frame.width;this.height=frame.height;
    this.boxes=copyBoxes(frame.boxes);this.reviewed=!!frame.reviewed;
    this.saved=this.snapshot();this.undoStack=[];this.redoStack=[];
  }
  snapshot(){return {boxes:copyBoxes(this.boxes),reviewed:this.reviewed};}
  get dirty(){return JSON.stringify(this.snapshot())!==JSON.stringify(this.saved);}
  change(boxes,reviewed=false) {
    if(boxes.length>MAX_BOXES)throw Error(`矩形は1画像${MAX_BOXES}個までです`);
    const next=boxes.map(box=>normalizeBox(box,this.width,this.height));
    if(next.some(box=>!box))throw Error('矩形は縦横2px以上で指定してください');
    if(JSON.stringify({boxes:next,reviewed})===JSON.stringify(this.snapshot()))return;
    this.undoStack.push(this.snapshot());if(this.undoStack.length>60)this.undoStack.shift();
    this.redoStack=[];this.boxes=next;this.reviewed=reviewed;
  }
  undo(redo=false) {
    const from=redo?this.redoStack:this.undoStack,to=redo?this.undoStack:this.redoStack;
    if(!from.length)return;
    to.push(this.snapshot());Object.assign(this,from.pop());
  }
  discard(){this.boxes=copyBoxes(this.saved.boxes);this.reviewed=this.saved.reviewed;this.undoStack=[];this.redoStack=[];}
}
