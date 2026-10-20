#!/usr/bin/env python3
"""Sequential training-only batch/loss/scheduler sweep, with resumable results."""
import argparse
import csv
from datetime import datetime
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]


def save(path,value):
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False));temporary.replace(path)


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def signatures(plan, *, weights=None):
    files=[Path(plan['weights'])] if weights is None else [Path(path) for path in weights]
    for root in sorted(set(plan['train']+plan['validation'])):
        root=Path(root);files.append(root/'samples.jsonl')
        if (root/'metadata.json').is_file():files.append(root/'metadata.json')
    result={str(path):sha(path) for path in files}
    # Detect normal image/count edits without re-reading all image bytes every run.
    for root in sorted(set(plan['train']+plan['validation'])):
        root=Path(root);state=hashlib.sha256()
        for line in (root/'samples.jsonl').read_text().splitlines():
            row=json.loads(line);path=(root/row.get('image',row.get('counts',''))).resolve(strict=True)
            if not path.is_relative_to(root):raise ValueError('Sample outside dataset')
            stat=path.stat();state.update(f'{path}:{stat.st_size}:{stat.st_mtime_ns}\n'.encode())
        result[str(root)+'#input_file_state']=state.hexdigest()
    for path in sorted((ROOT/'ros2_ws/src/kart_e2e/kart_e2e').glob('*.py')):
        result[str(path)]=sha(path)
    return result


def create_plan(a):
    train=[str(path.resolve(strict=True)) for path in a.train]
    validation=[str(path.resolve(strict=True)) for path in (a.validation or a.train)]
    weights=a.weights.resolve(strict=True);repo=a.repo.resolve(strict=True)
    if not (repo/'dinov3').is_dir():raise ValueError('--repo must contain dinov3/')
    if a.num_workers<0 or a.epochs<1 or any(v<1 for v in a.batches) or any(not math.isfinite(v) or v<=0 for v in a.learning_rates):
        raise ValueError('epochs, batches and learning rates must be positive')
    if not math.isfinite(a.beta) or a.beta<=0 or not 0<=a.seed<=2147483647:
        raise ValueError('beta must be positive; seed must be 0..2147483647')
    if not 0<=a.fixed_throttle<=a.max_throttle<=1:raise ValueError('Require 0 <= fixed-throttle <= max-throttle <= 1')
    cases=[]
    for index,(batch,loss,scheduler,lr) in enumerate(itertools.product(dict.fromkeys(a.batches),dict.fromkeys(a.losses),dict.fromkeys(a.schedulers),dict.fromkeys(a.learning_rates)),1):
        name=f'{index:02d}_b{batch}_{loss}_{scheduler}_lr{lr:g}'.replace('.','p')
        cases.append(dict(name=name,batch_size=batch,loss=loss,scheduler=scheduler,learning_rate=lr))
    plan=dict(schema=1,created=datetime.now().isoformat(),python=sys.executable,train=train,validation=validation,
              same_train_validation=train==validation,weights=str(weights),repo=str(repo),epochs=a.epochs,
              architecture=a.architecture,mode=a.mode,seed=a.seed,beta=a.beta,ram_preload=not a.stream,
              device=a.device,precision=a.precision,num_workers=a.num_workers,cases=cases,
              fixed_throttle=a.fixed_throttle,max_throttle=a.max_throttle,
              budgets=sorted(set([v for v in (5,10,20,30,50,75,100) if v<=a.epochs]+[a.epochs])))
    plan['signatures']=signatures(plan)
    return plan


def command(plan,case,output):
    return [plan['python'],'-m','kart_e2e.train','--train',*plan['train'],'--validation',*plan['validation'],
            '--repo',plan['repo'],'--weights',plan['weights'],'--output',str(output),
            '--architecture',plan['architecture'],'--mode',plan['mode'],'--finetune',
            '--fixed-throttle',str(plan['fixed_throttle']),'--max-throttle',str(plan['max_throttle']),
            '--epochs',str(plan['epochs']),'--batch-size',str(case['batch_size']),
            '--learning-rate',str(case['learning_rate']),'--loss',case['loss'],'--smooth-l1-beta',str(plan['beta']),
            '--lr-scheduler',case['scheduler'],'--lr-factor','0.5','--lr-patience','2','--min-lr',str(min(1e-7,case['learning_rate'])),
            '--seed',str(plan['seed']),'--device',plan['device'],'--precision',plan['precision'],
            '--num-workers',str(plan['num_workers']),'--no-feature-cache',
            '--ram-preload' if plan['ram_preload'] else '--no-ram-preload']


def summarize(root,plan):
    summaries=[];budgets=[]
    for case in plan['cases']:
        folder=root/case['name'];status_path=folder/'status.json'
        status=json.loads(status_path.read_text()) if status_path.exists() else dict(status='pending')
        rows=[];metrics=folder/'training/metrics.jsonl'
        if metrics.exists():
            for line in metrics.read_text().splitlines():
                try:
                    row=json.loads(line)
                    if math.isfinite(row['validation_mse']):rows.append(row)
                except (ValueError,KeyError):continue
        info={**case,**status,'completed_epochs':len(rows)}
        if rows:
            best=min(rows,key=lambda row:row['validation_mse'])
            info.update(best_epoch=best['epoch'],best_mse=best['validation_mse'],
                        best_mae=best['validation_mae'][0],best_loss=best['validation_loss'],
                        best_in_last_10_percent=best['epoch']>=.9*plan['epochs'])
            for budget in plan['budgets']:
                prefix=[row for row in rows if row['epoch']<=budget]
                if not prefix or prefix[-1]['epoch']!=budget:continue
                best_prefix=min(prefix,key=lambda row:row['validation_mse'])
                budgets.append(dict(name=case['name'],status=status['status'],batch_size=case['batch_size'],loss=case['loss'],
                                    scheduler=case['scheduler'],learning_rate=case['learning_rate'],budget_epochs=budget,
                                    best_epoch=best_prefix['epoch'],best_mse=best_prefix['validation_mse'],
                                    elapsed_seconds=prefix[-1].get('elapsed_seconds'),
                                    phase_seconds=sum(row.get('train_seconds',0)+row.get('validation_seconds',0) for row in prefix)))
        summaries.append(info)
    for filename,rows in [('summary.csv',summaries),('epoch_budgets.csv',budgets)]:
        path=root/filename;temporary=path.with_suffix('.csv.tmp')
        fields=list(dict.fromkeys(key for row in rows for key in row))
        with temporary.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
        temporary.replace(path)
    save(root/'summary.json',dict(same_train_validation=plan['same_train_validation'],selection_metric='validation_mse',results=summaries))
    successful=[row for row in summaries if row['status']=='succeeded']
    text=['# Training sweep results','',f'Train and validation identical: {plan["same_train_validation"]}',
          'Compare MSE across losses; raw Smooth L1 and MSE loss values have different scales.','',
          '| condition | status | best epoch | MSE | total seconds |','|---|---|---:|---:|---:|']
    for row in sorted(summaries,key=lambda row:row.get('best_mse',float('inf'))):
        text.append(f'| {row["name"]} | {row["status"]} | {row.get("best_epoch","—")} | {row.get("best_mse","—")} | {row.get("wall_seconds","—")} |')
    if successful:
        best=min(successful,key=lambda row:row['best_mse'])
        text+=['',f'Lowest MSE: {best["name"]}, best epoch {best["best_epoch"]}.',
               'If best epoch is near the epoch limit, the optimal stopping epoch is not yet established.']
    (root/'summary.md').write_text('\n'.join(text)+'\n')
    return summaries


def execute(root,plan,*,dry_run=False):
    print(f'Sweep: {len(plan["cases"])} conditions × {plan["epochs"]} epochs; train=val: {plan["same_train_validation"]}\nResults: {root}',flush=True)
    if signatures(plan)!=plan['signatures']:raise ValueError('Data, weights or implementation changed; start a new sweep')
    for index,case in enumerate(plan['cases'],1):
        folder=root/case['name'];status=folder/'status.json'
        if status.exists() and json.loads(status.read_text())['status'] in ('succeeded','failed','oom'):continue
        args=command(plan,case,folder/'training')
        if dry_run:
            import shlex
            print(shlex.join(args));continue
        if signatures(plan)!=plan['signatures']:raise ValueError('Data, weights or implementation changed during sweep')
        folder.mkdir(exist_ok=True)
        if (folder/'training').exists():
            # Preserve interrupted artifacts and restart the condition from its seed.
            (folder/'training').rename(folder/('interrupted-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')))
        save(status,dict(status='running'));save(folder/'command.json',args)
        print(f'[{index}/{len(plan["cases"])}] {case["name"]}',flush=True)
        started=time.monotonic()
        with (folder/'train.log').open('a') as log:
            process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
            try:code=process.wait()
            except KeyboardInterrupt:
                process.send_signal(__import__('signal').SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:process.kill();process.wait()
                save(status,dict(status='interrupted',wall_seconds=time.monotonic()-started));summarize(root,plan)
                raise
        with (folder/'train.log').open('rb') as log:
            log.seek(max(0,(folder/'train.log').stat().st_size-16000));tail=log.read().decode(errors='replace')
        state='succeeded' if code==0 else ('oom' if 'out of memory' in tail.lower() or 'OutOfMemoryError' in tail else 'failed')
        if state=='succeeded' and (not (folder/'training/best.pt').is_file() or len((folder/'training/metrics.jsonl').read_text().splitlines())!=plan['epochs']):state='failed'
        save(status,dict(status=state,returncode=code,wall_seconds=time.monotonic()-started))
        summarize(root,plan);print(f'  {state} · log: {folder/"train.log"}',flush=True)
    summarize(root,plan)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--train',type=Path,nargs='+');p.add_argument('--validation',type=Path,nargs='+')
    p.add_argument('--repo',type=Path,default=ROOT/'python_ws/dinov3')
    p.add_argument('--weights',type=Path,default=ROOT/'weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth')
    p.add_argument('--output',type=Path);p.add_argument('--resume',type=Path);p.add_argument('--summarize',type=Path)
    p.add_argument('--dry-run',action='store_true');p.add_argument('--epochs',type=int,default=100)
    p.add_argument('--batches',type=int,nargs='+',default=[32,64,128])
    p.add_argument('--losses',choices=['smooth_l1','mse'],nargs='+',default=['smooth_l1','mse'])
    p.add_argument('--schedulers',choices=['none','plateau'],nargs='+',default=['none','plateau'])
    p.add_argument('--learning-rates',type=float,nargs='+',default=[1e-5]);p.add_argument('--beta',type=float,default=.1)
    p.add_argument('--architecture',choices=['dinov3_vits16','dinov3_vitt16','dinov3_vitb16'],default='dinov3_vits16')
    p.add_argument('--mode',choices=['steer_only','steer_throttle'],default='steer_only')
    p.add_argument('--device',default='cuda');p.add_argument('--precision',choices=['auto','fp32','fp16','bf16'],default='auto')
    p.add_argument('--fixed-throttle',type=float,default=.15);p.add_argument('--max-throttle',type=float,default=.2)
    p.add_argument('--seed',type=int,default=42);p.add_argument('--num-workers',type=int,default=8);p.add_argument('--stream',action='store_true')
    a=p.parse_args()
    if a.resume or a.summarize:
        if a.resume and a.summarize:p.error('Choose --resume or --summarize')
        allowed={'--resume','--summarize','--dry-run'}
        if any(arg.split('=')[0] not in allowed for arg in sys.argv[1:] if arg.startswith('--')):p.error('Resume uses saved settings; do not supply new training options')
        root=(a.resume or a.summarize).resolve(strict=True);plan=json.loads((root/'plan.json').read_text())
    else:
        if not a.train:p.error('--train is required; --validation defaults to the same datasets')
        plan=create_plan(a)
        root=(a.output or ROOT/'e2e/sweeps'/datetime.now().strftime('%Y-%m-%d/%H-%M-%S')).resolve()
        root.mkdir(parents=True,exist_ok=False);save(root/'plan.json',plan)
    import fcntl
    with (root/'.lock').open('w') as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        if a.summarize:summarize(root,plan)
        else:execute(root,plan,dry_run=a.dry_run)
    print(f'Results: {root/"summary.md"}\nEpoch comparison: {root/"epoch_budgets.csv"}',flush=True)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:raise SystemExit(130)
    except (ValueError,OSError) as error:raise SystemExit(str(error))
