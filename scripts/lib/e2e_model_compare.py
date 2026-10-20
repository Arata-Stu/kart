#!/usr/bin/env python3
"""RGB size/pretraining/distillation comparison within a measured wall-time budget."""
import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

from e2e_sweep import ROOT, save, sha, signatures as data_signatures


def cases(budgets, *, preset='full', size_distill_epochs=50):
    def train(name, architecture, initialization='pretrained', finetune=False, **extra):
        return dict(name=name,kind='train',architecture=architecture,
                    initialization=initialization,finetune=finetune,**extra)
    if preset=='sizes':
        tiny=f'tiny_distill_{size_distill_epochs}'
        return [train('s_freeze','dinov3_vits16'),train('b_freeze','dinov3_vitb16'),
                train(tiny,'dinov3_vitt16',distill_epochs=size_distill_epochs),
                train('s_finetune','dinov3_vits16',finetune=True),
                train('b_finetune','dinov3_vitb16',finetune=True),
                train(tiny+'_finetune','dinov3_vitt16',finetune=True,
                      distill_epochs=size_distill_epochs,encoder_name=tiny+'_encoder')]
    return [
        train('s_freeze','dinov3_vits16'),
        train('b_freeze','dinov3_vitb16'),
        train('s_finetune','dinov3_vits16',finetune=True),
        train('s_scratch','dinov3_vits16','scratch',True),
        train('tiny_scratch','dinov3_vitt16','scratch',True),
        train('cnn_scratch','simple_cnn','scratch',True),
        *[train(f'tiny_distill_{n}','dinov3_vitt16',distill_epochs=n) for n in budgets],
        train('b_finetune','dinov3_vitb16',finetune=True),
        train('b_scratch','dinov3_vitb16','scratch',True),
    ]


def signatures(plan):
    paths=[Path(plan[key]) for key in ('small_weights','base_weights') if plan.get(key)]
    result=data_signatures(plan,weights=[p for p in paths if p.is_file()])
    for path in paths:
        if not path.is_file():result[str(path)]='missing'
    for path in [Path(__file__),ROOT/'scripts/lib/e2e_sweep.py']:
        result[str(path)]=sha(path)
    repo=Path(plan['repo'])
    if (repo/'dinov3').is_dir():
        # The trainer enforces the pinned clean revision. Also detect changes across resume.
        for command in (['rev-parse','HEAD'],['diff','HEAD','--binary']):
            value=subprocess.check_output(['git','-C',str(repo),*command])
            result['encoder_repo:'+command[0]]=__import__('hashlib').sha256(value).hexdigest()
    return result


def create_plan(a):
    for value in (a.hours,a.safety_factor,a.head_lr,a.finetune_lr,a.scratch_lr):
        if not math.isfinite(value) or value<=0:raise ValueError('hours/safety-factor/LR must be positive and finite')
    if a.safety_factor<1 or min(a.epochs,a.pilot_epochs,a.batch_size,a.size_distill_epochs,*a.distill_epochs)<1 or a.num_workers<0:
        raise ValueError('Invalid epochs/batch/workers/safety-factor')
    if not 0<=a.seed<=2147483647 or not 0<=a.fixed_throttle<=a.max_throttle<=1:
        raise ValueError('Invalid seed or throttle range')
    budgets=list(dict.fromkeys(a.distill_epochs))
    selected=cases(budgets,preset=a.preset,size_distill_epochs=a.size_distill_epochs)
    if a.cases:
        by_name={c['name']:c for c in [*cases(budgets),*cases(budgets,preset='sizes',size_distill_epochs=a.size_distill_epochs)]}
        unknown=set(a.cases)-by_name.keys()
        if unknown:raise ValueError('Unknown --cases: '+', '.join(sorted(unknown)))
        selected=[by_name[key] for key in dict.fromkeys(a.cases)]
    train=[str(p.resolve(strict=True)) for p in a.train]
    validation=[str(p.resolve(strict=True)) for p in (a.validation or a.train)]
    for folder in set(train+validation):
        root=Path(folder)
        metadata=json.loads((root/'metadata.json').read_text()) if (root/'metadata.json').is_file() else {}
        if metadata.get('modality','rgb')!='rgb' or metadata.get('routing_sha256') or metadata.get('routing'):
            raise ValueError('Use RGB single-head datasets for this comparison')
        if not (root/'samples.jsonl').read_text().strip():raise ValueError('Empty dataset')
    plan={key:getattr(a,key) for key in ('hours','epochs','pilot_epochs','safety_factor','batch_size','seed','device',
            'precision','num_workers','head_lr','finetune_lr','scratch_lr','mode','fixed_throttle','max_throttle')}
    plan.update(schema=1,python=sys.executable,created=datetime.now().isoformat(),train=train,validation=validation,
                same_train_validation=train==validation,repo=str(a.repo.resolve()),
                small_weights=str(a.small_weights.resolve()),base_weights=str(a.base_weights.resolve()),
                ram_preload=not a.stream,cases=selected,preset='custom' if a.cases else a.preset)
    needs_small=any(c.get('distill_epochs') or (c['architecture']=='dinov3_vits16' and c['initialization']=='pretrained') for c in selected)
    if needs_small and not Path(plan['small_weights']).is_file():raise ValueError('ViT-S weights missing: '+plan['small_weights'])
    if any(c['architecture']!='simple_cnn' for c in selected) and not (a.repo/'dinov3').is_dir():
        raise ValueError('--repo must be a local pinned DINOv3 checkout containing dinov3/')
    plan['signatures']=signatures(plan)
    return plan


def command(plan,case,root,*,pilot=False,encoder=None):
    distill=case['kind']=='distill'
    epochs=min(plan['pilot_epochs'],case['epochs']) if pilot else case['epochs']
    args=[plan['python'],'-m','kart_e2e.'+('distill' if distill else 'train'),
          '--train',*plan['train'],'--validation',*plan['validation'],'--repo',plan['repo'],
          '--output',str(root/'output'),'--epochs',str(epochs),'--batch-size',str(plan['batch_size']),
          '--seed',str(plan['seed']),'--device',plan['device'],'--num-workers',str(plan['num_workers']),
          '--cache-dir',str(root.parents[1]/'.feature_cache'),
          '--ram-preload' if plan['ram_preload'] else '--no-ram-preload']
    if distill:
        return args+['--teacher-weights',plan['small_weights'],'--learning-rate','0.0001','--feature-cache']
    scratch=case['initialization']=='scratch'
    finetune=case['finetune'] or scratch
    lr=plan['scratch_lr'] if scratch else plan['finetune_lr'] if finetune else plan['head_lr']
    args+=['--architecture',case['architecture'],'--initialization',case['initialization'],
           '--mode',plan['mode'],'--fixed-throttle',str(plan['fixed_throttle']),'--max-throttle',str(plan['max_throttle']),
           '--learning-rate',str(lr),'--loss','smooth_l1','--smooth-l1-beta','0.1',
           '--lr-scheduler','plateau','--lr-factor','0.5','--lr-patience','2','--min-lr',str(min(1e-7,lr)),
           '--precision',plan['precision'],'--no-feature-cache' if finetune else '--feature-cache']
    if finetune:args+=['--finetune']
    if not scratch:
        weights=encoder or plan['base_weights' if case['architecture']=='dinov3_vitb16' else 'small_weights']
        args+=['--weights',str(weights)]
    return args


def metrics(folder):
    rows=[]
    path=folder/'output/metrics.jsonl'
    if path.is_file():
        for line in path.read_text().splitlines():
            try:
                row=json.loads(line)
                if math.isfinite(row.get('validation_loss',float('nan'))):rows.append(row)
            except (ValueError,TypeError):pass
    return rows


def status(folder):
    path=folder/'status.json'
    return json.loads(path.read_text()) if path.is_file() else {'status':'pending'}


def stop(process):
    if process.poll() is not None:return
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        process.wait()


def run(folder,args,deadline):
    folder.mkdir(parents=True,exist_ok=True)
    if (folder/'output').exists():
        (folder/'output').rename(folder/('interrupted-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')))
    save(folder/'command.json',args);save(folder/'status.json',dict(status='running'))
    started=time.monotonic();state='failed';code=None
    try:
        with (folder/'train.log').open('a') as log:
            process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                code=process.wait(timeout=max(.001,deadline-time.monotonic()))
                state='succeeded' if code==0 else 'failed'
            except subprocess.TimeoutExpired:
                stop(process);state='budget_timeout'
            except KeyboardInterrupt:
                stop(process);state='interrupted';raise
        if state=='succeeded':
            checkpoint='encoder.pt' if 'kart_e2e.distill' in args else 'best.pt'
            rows=metrics(folder);expected=int(args[args.index('--epochs')+1])
            if len(rows)!=expected or not (folder/'output'/checkpoint).is_file():state='failed'
        if state=='failed' and 'out of memory' in (folder/'train.log').read_text()[-16000:].lower():state='oom'
    finally:
        save(folder/'status.json',dict(status=state,returncode=code,wall_seconds=time.monotonic()-started))
    return state=='succeeded'


def estimate(folder,epochs,factor):
    rows=metrics(folder)
    if not rows or status(folder)['status']!='succeeded':return None
    phases=[r['train_seconds']+r['validation_seconds'] for r in rows]
    wall=status(folder)['wall_seconds']
    overhead=max(0,wall-sum(phases))
    # Full-data pilots include imports, weight loading, RAM/cache construction and validation.
    return factor*(overhead+max(phases)*epochs)


def stages(plan,case):
    if case.get('distill_epochs'):
        return [dict(name=case.get('encoder_name',case['name']+'_encoder'),kind='distill',epochs=case['distill_epochs']),
                {**case,'epochs':plan['epochs']}]
    return [{**case,'epochs':plan['epochs']}]


def summarize(root,plan):
    results=[];budgets=[]
    for case in plan['cases']:
        row={**case,**status(root/'cases'/case['name'])}
        steps=stages(plan,case)
        for stage in steps:
            folder=root/'runs'/stage['name'];s=status(folder)
            row[stage['kind']+'_status']=s['status']
            row[stage['kind']+'_seconds']=s.get('wall_seconds')
        folder=root/'runs'/case['name'];values=metrics(folder)
        if values:
            best=min(values,key=lambda r:r['validation_mse'])
            row.update(completed_epochs=len(values),best_epoch=best['epoch'],best_mse=best['validation_mse'],
                       steering_mae=best['validation_mae'][0],best_in_last_10_percent=best['epoch']>=.9*plan['epochs'])
            for limit in sorted(set([n for n in (10,25,50,75,100) if n<=plan['epochs']]+[plan['epochs']])):
                prefix=[r for r in values if r['epoch']<=limit]
                if not prefix or prefix[-1]['epoch']!=limit:continue
                best_prefix=min(prefix,key=lambda r:r['validation_mse'])
                budgets.append(dict(name=case['name'],budget_epochs=limit,best_epoch=best_prefix['epoch'],best_mse=best_prefix['validation_mse']))
            info=json.loads((folder/'output/run.json').read_text())
            for key in ('total_parameters','trainable_parameters','effective_precision','learning_rate','loss','lr_scheduler'):
                row[key]=info.get(key)
            checkpoint=folder/'output/best.pt'
            if checkpoint.is_file():row['checkpoint_bytes']=checkpoint.stat().st_size
        if case.get('distill_epochs'):
            values=metrics(root/'runs'/steps[0]['name'])
            if values:row['encoder_best_epoch']=min(values,key=lambda r:r['validation_loss'])['epoch']
        row['preparation_seconds']=sum(row.get(k+'_seconds') or 0 for k in ('train','distill'))
        results.append(row)
    for filename,rows in [('summary.csv',results),('epoch_budgets.csv',budgets)]:
        fields=list(dict.fromkeys(k for row in rows for k in row))
        with (root/filename).open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    save(root/'summary.json',dict(same_train_validation=plan['same_train_validation'],results=results))
    lines=['# RGB model comparison','',f'Train = validation: {plan["same_train_validation"]}',
           'Same-data MSE measures fit, not unseen driving accuracy. Budget estimates are not hardware guarantees.',
           'Preparation time excludes timing pilots. Pilot logs and duration are under pilots/.',
           '', '| case | status | MSE | best epoch | encoder epoch | prep seconds |',
           '|---|---|---:|---:|---:|---:|']
    for row in results:
        lines.append('| '+' | '.join(str(row.get(k,'—')) for k in ('name','status','best_mse','best_epoch','encoder_best_epoch','preparation_seconds'))+' |')
    (root/'summary.md').write_text('\n'.join(lines)+'\n')
    size_report(root,results)
    return results


def size_report(root,results):
    """Compare completed runs against pretrained ViT-B within the same update scope."""
    references={r['finetune']:r for r in results if r['status']=='succeeded' and r['architecture']=='dinov3_vitb16'
                and r['initialization']=='pretrained' and r.get('best_mse') is not None}
    rows=[]
    for result in results:
        row={key:result.get(key) for key in ('name','architecture','status','best_mse','steering_mae','best_epoch',
                'total_parameters','checkpoint_bytes','preparation_seconds')}
        row.update(scope='finetune' if result['finetune'] else 'freeze',
                   encoder_origin='distilled' if result.get('distill_epochs') else result['initialization'])
        reference=references.get(result['finetune'])
        if reference and result['status']=='succeeded' and result.get('best_mse') is not None:
            row['reference']=reference['name']
            if reference['best_mse']>0:row['mse_ratio_to_b']=result['best_mse']/reference['best_mse']
            if reference.get('total_parameters') and result.get('total_parameters'):
                row['parameter_ratio_to_b']=result['total_parameters']/reference['total_parameters']
        rows.append(row)
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with (root/'size_comparison.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    text=['# Model size comparison','',
          'Ratios use completed pretrained ViT-B runs in the same freeze/finetune group; lower is smaller/better.',
          'Tiny uses ViT-S distillation, so this compares practical model choices, not architecture alone.',
          'Checkpoint bytes are PyTorch storage, not TensorRT engine size or inference speed.','',
          '| scope | model | origin | status | parameters | MSE | MSE / B | parameters / B |',
          '|---|---|---|---|---:|---:|---:|---:|']
    for row in rows:
        text.append('| '+' | '.join(str(row.get(k,'—')) for k in ('scope','architecture','encoder_origin','status',
                    'total_parameters','best_mse','mse_ratio_to_b','parameter_ratio_to_b'))+' |')
    (root/'size_comparison.md').write_text('\n'.join(text)+'\n')


def execute(root,plan,*,hours=None,dry_run=False):
    if signatures(plan)!=plan['signatures']:raise ValueError('Data, weights, source or DINOv3 changed; start a new comparison')
    deadline=time.monotonic()+3600*(hours if hours is not None else plan['hours'])
    print(f'Results: {root}\nBudget: {(deadline-time.monotonic())/3600:.2f} hours; {len(plan["cases"])} comparisons',flush=True)
    for case in plan['cases']:
        case_dir=root/'cases'/case['name'];case_dir.mkdir(parents=True,exist_ok=True)
        previous=status(case_dir)['status']
        if previous in ('succeeded','failed','oom','unavailable'):continue
        if signatures(plan)!=plan['signatures']:raise ValueError('Inputs changed during comparison')
        steps=stages(plan,case)
        if case['architecture']=='dinov3_vitb16' and case['initialization']=='pretrained' and not Path(plan['base_weights']).is_file():
            save(case_dir/'status.json',dict(status='unavailable',reason='ViT-B weights missing'));continue
        if dry_run:
            print('# '+case['name'])
            for stage in steps:
                encoder=root/'runs'/steps[0]['name']/'output/encoder.pt' if case.get('distill_epochs') and stage['kind']=='train' else None
                print(shlex.join(command(plan,stage,root/'runs'/stage['name'],encoder=encoder)))
            continue
        predicted=0.;ready=True
        # Reuse timing profiles by architecture/scope, and one Tiny distillation timing profile.
        for stage in steps:
            target=root/'runs'/stage['name']
            if status(target)['status']=='succeeded':continue
            profile='distill' if stage['kind']=='distill' else (
                'tiny_distilled_'+('finetune' if case['finetune'] else 'freeze') if case.get('distill_epochs') else case['name'])
            pilot=root/'pilots'/profile
            if status(pilot)['status']!='succeeded':
                if deadline-time.monotonic()<=1:ready=False;break
                encoder=root/'pilots/distill/output/encoder.pt' if case.get('distill_epochs') and stage['kind']=='train' else None
                # A previous completed encoder stage can provide the pilot input on resume.
                if encoder and not encoder.is_file():encoder=root/'runs'/steps[0]['name']/'output/encoder.pt'
                print(f'Pilot: {profile} ({min(plan["pilot_epochs"],stage["epochs"])} epochs)',flush=True)
                if not run(pilot,command(plan,stage,pilot,pilot=True,encoder=encoder),deadline):
                    save(case_dir/'status.json',dict(status=status(pilot)['status'],reason='Timing pilot failed'));ready=False;break
            predicted+=estimate(pilot,stage['epochs'],plan['safety_factor'])
        if not ready:
            if status(case_dir)['status'] not in ('failed','oom','interrupted','budget_timeout'):
                save(case_dir/'status.json',dict(status='budget_skipped',reason='No time for pilot'))
            summarize(root,plan);continue
        remaining=deadline-time.monotonic()
        if predicted>remaining:
            print(f'Skip {case["name"]}: estimate {predicted/60:.1f} min > remaining {remaining/60:.1f} min',flush=True)
            save(case_dir/'status.json',dict(status='budget_skipped',estimate_seconds=predicted,remaining_seconds=max(0,remaining)))
            summarize(root,plan);continue
        save(case_dir/'status.json',dict(status='running',estimate_seconds=predicted))
        print(f'Run: {case["name"]}, estimate {predicted/60:.1f} min; remaining {remaining/60:.1f} min',flush=True)
        for stage in steps:
            folder=root/'runs'/stage['name']
            if status(folder)['status']=='succeeded':continue
            if signatures(plan)!=plan['signatures']:raise ValueError('Inputs changed before training stage')
            encoder=root/'runs'/steps[0]['name']/'output/encoder.pt' if case.get('distill_epochs') and stage['kind']=='train' else None
            if not run(folder,command(plan,stage,folder,encoder=encoder),deadline):
                save(case_dir/'status.json',dict(status=status(folder)['status'],estimate_seconds=predicted));break
        else:save(case_dir/'status.json',dict(status='succeeded',estimate_seconds=predicted))
        summarize(root,plan)
    summarize(root,plan)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--train',type=Path,nargs='+');p.add_argument('--validation',type=Path,nargs='+')
    p.add_argument('--repo',type=Path,default=ROOT/'python_ws/dinov3')
    p.add_argument('--small-weights','--weights',dest='small_weights',type=Path,default=ROOT/'weights/dinov3/dinov3_vits16_pretrain_lvd1689m-08c60483.pth')
    p.add_argument('--base-weights',type=Path,default=ROOT/'weights/dinov3/dinov3_vitb16.pth')
    p.add_argument('--output',type=Path);p.add_argument('--resume',type=Path);p.add_argument('--summarize',type=Path)
    p.add_argument('--hours',type=float,default=None,help='Wall-time budget per invocation, default 6; resume starts a new budget')
    p.add_argument('--preset',choices=['sizes','full'],default='sizes',help='sizes: 6 size-focused cases (default); full: previous 11 cases')
    p.add_argument('--size-distill-epochs',type=int,default=50,help='Shared Tiny encoder budget for sizes preset')
    p.add_argument('--epochs',type=int,default=100);p.add_argument('--distill-epochs',type=int,nargs='+',default=None,help='full preset distillation budgets; default 10 50 100')
    p.add_argument('--pilot-epochs',type=int,default=2);p.add_argument('--safety-factor',type=float,default=1.3)
    p.add_argument('--cases',nargs='+',help='Subset of named comparisons; see --dry-run')
    p.add_argument('--batch-size',type=int,default=32);p.add_argument('--num-workers',type=int,default=8)
    p.add_argument('--head-lr',type=float,default=1e-4);p.add_argument('--finetune-lr',type=float,default=1e-5)
    p.add_argument('--scratch-lr',type=float,default=1e-4);p.add_argument('--seed',type=int,default=42)
    p.add_argument('--device',default='cuda');p.add_argument('--precision',choices=['auto','fp32','bf16','fp16'],default='auto')
    p.add_argument('--mode',choices=['steer_only','steer_throttle'],default='steer_only')
    p.add_argument('--fixed-throttle',type=float,default=.15);p.add_argument('--max-throttle',type=float,default=.2)
    p.add_argument('--stream',action='store_true');p.add_argument('--dry-run',action='store_true')
    a=p.parse_args()
    if a.distill_epochs is not None and a.preset=='sizes' and not a.cases:
        p.error('--distill-epochs compares budgets with --preset full; sizes uses --size-distill-epochs')
    if a.distill_epochs is None:a.distill_epochs=[10,50,100]
    if a.hours is not None and (not math.isfinite(a.hours) or a.hours<=0):p.error('--hours must be positive and finite')
    if a.resume or a.summarize:
        if a.resume and a.summarize:p.error('Choose --resume or --summarize')
        allowed={'--resume','--summarize','--hours','--dry-run'}
        if any(v.split('=')[0] not in allowed for v in sys.argv[1:] if v.startswith('--')):p.error('Resume uses saved settings; only --hours may change')
        root=(a.resume or a.summarize).resolve(strict=True);plan=json.loads((root/'plan.json').read_text())
    else:
        if not a.train:p.error('--train required; --validation defaults to the same dataset')
        a.hours=6 if a.hours is None else a.hours;plan=create_plan(a)
        root=(a.output or ROOT/'e2e/comparisons'/datetime.now().strftime('%Y-%m-%d/%H-%M-%S')).resolve()
        root.mkdir(parents=True,exist_ok=False);save(root/'plan.json',plan)
    import fcntl
    with (root/'.lock').open('w') as lock:
        fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        started=time.monotonic()
        started_at=datetime.now().isoformat()
        try:
            if a.summarize:summarize(root,plan)
            else:execute(root,plan,hours=a.hours,dry_run=a.dry_run)
        finally:
            rows=summarize(root,plan)
            if not a.summarize and not a.dry_run:
                with (root/'invocations.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(started=started_at,wall_seconds=time.monotonic()-started,
                                     budget_hours=a.hours if a.hours is not None else plan['hours']))+'\n')
    counts={key:sum(row['status']==key for row in rows) for key in sorted({r['status'] for r in rows})}
    print('Results: '+json.dumps(counts),flush=True)
    print('Summary: '+str(root/'summary.md'),flush=True)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:raise SystemExit(130)
    except (ValueError,OSError) as error:raise SystemExit(str(error))
