// Inline controls avoid native select popups overlapping the opening click.
export function profilePicker({id, profiles, value='', optional=false, defaultValue='', title}) {
  const root=document.createElement('div');root.className='profile-picker';
  root.setAttribute('role','group');root.setAttribute('aria-label',title);
  const input=document.createElement('input');input.type='hidden';input.id=id;input.value=value;
  const heading=document.createElement('strong');heading.className='profile-current';
  const controls=document.createElement('div');root.append(input,heading,controls);
  const choices=profiles.filter(p=>/^\d+x\d+x\d+$/.test(p));
  let resolution=choices.includes(value)?value.split('x').slice(0,2).join('x'):null;
  const select=next=>{input.value=next;draw();input.onchange?.();};
  const group=(name, items)=>{
    const row=document.createElement('div');row.className='profile-options';row.setAttribute('role','group');row.setAttribute('aria-label',name);
    for(const {text,selected,click} of items){
      const b=document.createElement('button');b.type='button';b.textContent=text;
      b.setAttribute('aria-pressed',String(selected));b.onclick=click;row.append(b);
    }
    controls.append(row);
  };
  function draw(){
    controls.replaceChildren();
    const effective=input.value || defaultValue;
    resolution=choices.includes(effective)?effective.split('x').slice(0,2).join('x'):null;
    const display=effective==='0'?'OFF':effective?effective.replace(/x/g,' × ').replace(/ × (\d+)$/,' / $1 Hz'):(optional?'既定値未取得':'プロファイル未選択');
    heading.textContent=display+(input.value==='' && defaultValue?'（既定）':'');
    if(optional)group('RGBの使用',[{text:'既定',selected:input.value==='',click:()=>select('')},{text:'OFF',selected:input.value==='0',click:()=>select('0')}]);
    const sizes=[...new Set(choices.map(p=>p.split('x').slice(0,2).join('x')))];
    group('解像度',sizes.map(size=>({text:size.replace('x',' × '),selected:choices.includes(effective)&&effective.startsWith(size+'x'),click:()=>{
      resolution=size;
      const rates=choices.filter(p=>p.startsWith(size+'x'));
      const old=effective.split('x')[2];
      select(rates.find(p=>p.endsWith('x'+old)) || rates.find(p=>p.endsWith('x30')) || rates[0]);
    }})));
    if(resolution)group('FPS',choices.filter(p=>p.startsWith(resolution+'x')).map(p=>({text:p.split('x')[2]+' Hz',selected:effective===p,click:()=>select(p)})));
    else {const hint=document.createElement('small');hint.textContent='解像度を選ぶと、対応するHzが表示されます。';controls.append(hint);}
  }
  draw();return {root,input};
}
