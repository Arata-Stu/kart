import { $, on, emit } from './api.js';
import { fit, zoom } from './canvas.js';

const stage=$('map-stage'), panel=$('tuning-panel'), divider=$('tuning-divider');
let drag=null, wide=false, only=false;
function render() {
  document.body.classList.toggle('map-wide',wide);
  document.body.classList.toggle('map-only',only);
  $('map-wide').textContent=wide?'編集パネルを戻す':'地図を広く';
  $('map-wide').setAttribute('aria-pressed',String(wide));
  $('map-only').textContent=only?'通常表示に戻す（Esc）':'地図だけ';
  $('map-only').setAttribute('aria-pressed',String(only));
  divider.hidden=!panel.open||only;
  requestAnimationFrame(()=>emit('view'));
}
function setWidth(value) {
  const max=Math.max(240,Math.min(650,stage.clientWidth-220));
  const width=Math.max(240,Math.min(max,value));
  stage.style.setProperty('--tuning-panel-width',`${width}px`);
  divider.setAttribute('aria-valuenow',String(Math.round(width)));
  divider.setAttribute('aria-valuemax',String(Math.round(max)));
}
$('map-wide').addEventListener('click',()=>{wide=!wide;render();});
$('map-only').addEventListener('click',()=>{only=!only;render();});
$('map-zoom-in').addEventListener('click',()=>zoom(1.4));
$('map-zoom-out').addEventListener('click',()=>zoom(1/1.4));
$('map-view-fit').addEventListener('click',()=>fit());
panel.addEventListener('toggle',render);
divider.addEventListener('pointerdown',event=>{
  if(event.button!==0)return;
  drag={x:event.clientX,width:panel.getBoundingClientRect().width};
  divider.setPointerCapture(event.pointerId);
  document.body.classList.add('resizing-view');
});
divider.addEventListener('pointermove',event=>{if(drag)setWidth(drag.width+drag.x-event.clientX);});
const stop=()=>{drag=null;document.body.classList.remove('resizing-view');};
divider.addEventListener('pointerup',stop);
divider.addEventListener('pointercancel',stop);
divider.addEventListener('lostpointercapture',stop);
divider.addEventListener('dblclick',()=>setWidth(340));
divider.addEventListener('keydown',event=>{
  if(!['ArrowLeft','ArrowRight','Home'].includes(event.key))return;
  event.preventDefault();
  setWidth(event.key==='Home'?340:panel.getBoundingClientRect().width+(event.key==='ArrowLeft'?20:-20));
});
document.addEventListener('keydown',event=>{
  if(event.key==='Escape'&&only&&!document.querySelector('dialog[open]')){only=false;render();}
});
on('page',event=>{if(event.detail!=='map'){wide=false;only=false;render();}});
new ResizeObserver(()=>setWidth(parseFloat(stage.style.getPropertyValue('--tuning-panel-width'))||340)).observe(stage);
render();
