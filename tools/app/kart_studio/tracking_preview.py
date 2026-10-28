"""Bounded local-only preview using the production C++ lateral controllers."""
import hashlib
import math
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading

_lock=threading.Lock()
_cache=Path(tempfile.mkdtemp(prefix='kart-tracking-preview-'))


def preview(body):
    with _lock:
        return _preview(body)


def _preview(body):
    root=Path(__file__).resolve().parents[3]
    source=root/'tools/app/native/tracking_preview.cpp'
    include=root/'ros2_ws/src/kart_control/include'
    digest=hashlib.sha256(source.read_bytes()+b''.join(p.read_bytes() for p in sorted((include/'kart_control').glob('*.hpp')))).hexdigest()
    executable=_cache/digest
    if not executable.exists():
        compiler=shutil.which('c++')
        if not compiler: raise ValueError('試走にはStudio実行環境のC++コンパイラが必要です（Dockerでは導入済み）')
        result=subprocess.run([compiler,'-std=c++17','-O2','-I'+str(include),str(source),'-o',str(executable)],capture_output=True,text=True,timeout=60)
        if result.returncode: raise ValueError('試走計算のビルドに失敗: '+result.stderr[-2000:])
    line=body['line'];points=line['points'];closed=body['closed'];cfg=body['options']
    if type(closed) is not bool or not (3 if closed else 2)<=len(points)<=5000: raise ValueError('試走ラインの点数が不正です')
    method=cfg.get('controller','pure_pursuit')
    if method not in ('pure_pursuit','mpc'): raise ValueError('未知の試走方式です')
    keys=[('speed',0,20),('wheelbase',.05,2),('maxSteer',.01,1.2),('horizon',.3,2),('tracking',.1,100),('smoothness',.1,100),('steeringRate',.1,5)]
    values=[]
    for key,low,high in keys:
        v=cfg[key]
        if type(v) not in (int,float) or not math.isfinite(v) or not low<=v<=high: raise ValueError(key+': 試走設定が範囲外です')
        values.append(v)
    profile=line.get('profile')
    if profile is not None and len(profile)!=len(points): raise ValueError('速度profileの点数が不一致です')
    rows=[]
    for i,p in enumerate(points):
        if profile is not None and len(profile[i])<6: raise ValueError('速度profileが不正です')
        v=profile[i][5] if profile is not None else cfg['speed']
        if len(p)!=2 or any(type(x) not in (int,float) or not math.isfinite(x) for x in [*p,v]) or not 0<=v<=20: raise ValueError('試走ラインが不正です')
        rows.append(f'{p[0]} {p[1]} {v}')
    header=[len(points),int(closed),int(method=='mpc'),int(line.get('speed_mode')=='step'),*values]
    result=subprocess.run([str(executable)],input=' '.join(map(str,header))+'\n'+'\n'.join(rows),capture_output=True,text=True,timeout=30)
    if result.returncode: raise ValueError('試走計算に失敗しました')
    frames=[];reason=''
    for row in result.stdout.splitlines():
        if row.startswith('END '): reason=row[4:]
        else:
            frame=list(map(float,row.split()))
            if len(frame)!=13 or not all(math.isfinite(v) for v in frame): raise ValueError('試走出力が不正です')
            frames.append(frame)
    if not frames: raise ValueError('試走開始に失敗: '+reason)
    return dict(frames=frames,reason=reason,controller=method)
