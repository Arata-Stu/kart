#!/usr/bin/env python3
"""Offline recording audit, longitudinal identification and PID candidate preview."""
import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import shutil
import tempfile

from analysis import analyze, simulate, validate_config
from bag import read_bag, REQUIRED
from report import analysis_report, page, table, write_csv, write_json

ROOT=Path(__file__).resolve().parents[2]
DEFAULT_CONFIG=Path(__file__).with_name('config.json')


def publish(output, callback):
    output=Path(output).expanduser().resolve()
    if output.exists(): raise ValueError(f'Output already exists: {output}')
    output.parent.mkdir(parents=True,exist_ok=True)
    temporary=Path(tempfile.mkdtemp(prefix='.'+output.name+'-',dir=output.parent))
    try:
        callback(temporary)
        if output.exists(): raise ValueError(f'Output already exists: {output}')
        temporary.rename(output)
    except BaseException:
        shutil.rmtree(temporary,ignore_errors=True)
        raise
    print(f'Report: {output / "report.html"}')


def tune(args):
    import yaml
    summary_path=Path(args.analysis)/'summary.json'
    summary=json.loads(summary_path.read_text())
    model=summary['model']
    document=yaml.safe_load(Path(args.base_config).read_text())
    settings=document['/**/speed_controller']['ros__parameters']
    original=dict(settings)
    for key in ('kp','ki','kd','kff','brake_limit','throttle_limit'):
        value=getattr(args,key)
        if value is not None: settings[key]=value
    targets=[float(v) for v in args.targets.split(',')]
    if any(v>settings['max_target_speed_mps'] for v in targets):
        raise ValueError('Target exceeds max_target_speed_mps in base config')
    delays=[model.get('assumed_delay_s',.15)] if args.delays is None else [float(v) for v in args.delays.split(',')]
    if not 1<=len(delays)<=10 or any(not math.isfinite(v) or v<0 or v>2 for v in delays):
        raise ValueError('Use 1..10 finite delays in [0,2] seconds')
    if not 0<args.stage_seconds<=60 or len(targets)>20:
        raise ValueError('stage-seconds must be (0,60]; at most 20 targets')
    runs=[]
    for delay in delays:
        rows=simulate(model,settings,targets,args.stage_seconds,delay)
        rms=math.sqrt(sum((r['target_mps']-r['speed_mps'])**2 for r in rows)/len(rows))
        runs.append((delay,rows,{'delay_s':delay,'speed_rmse_mps':rms,
            'extrapolated_fraction':sum(r['extrapolated'] for r in rows)/len(rows),
            'max_speed_mps':max(r['speed_mps'] for r in rows)}))
    def save(out):
        result={'schema':1,'analysis':str(summary_path.resolve()),
                'analysis_sha256':hashlib.sha256(summary_path.read_bytes()).hexdigest(),
                'base_config':str(Path(args.base_config).resolve()),
                'settings_before':original,'settings_candidate':settings,'targets_mps':targets,
                'stage_seconds':args.stage_seconds,'runs':[r[2] for r in runs],
                'limitations':['Offline scalar fitted-model simulation, not rosbag counterfactual replay or real-vehicle validation.',
                    'Delay is an assumption/sensitivity sweep, not an identified physical actuator delay.',
                    'ROS gates, estimator noise, steering, tire saturation and ESC state transitions are not simulated.',
                    'Initial speed equals the first target. Values outside observed domain are marked extrapolated.']}
        write_json(out/'tuning.json',result)
        (out/'speed_controller.candidate.yaml').write_text(
            '# REVIEW CANDIDATE ONLY: no automatic application; see tuning.json and report.html.\n'+yaml.safe_dump(document,sort_keys=False))
        for index,(delay,rows,_) in enumerate(runs): write_csv(out/f'simulation_{index}.csv',rows)
        body='<p class="notice">同定したモデル上の試算です。実車検証ではありません。実車設定は変更していません。</p>'
        body+='<p><a href="speed_controller.candidate.yaml">設定候補 YAML</a> · <a href="tuning.json">設定差分・条件 JSON</a></p>'
        body+='<h2>変更する値</h2>'+table([{'parameter':k,'before':original[k],'candidate':settings[k]} for k in settings if original[k]!=settings[k]])
        body+='<h2>遅延仮定ごとの比較</h2>'+table(result['runs'])
        body+='<p>グラフは最初の遅延条件。全条件の時系列はsimulation_N.csvに保存。</p><ul>'+''.join('<li>'+html.escape(v)+'</li>' for v in result['limitations'])+'</ul>'
        page(out/'report.html','kart PID候補のモデル試算',body,runs[0][1],[
            ('速度 [m/s]',['target_mps','speed_mps']),('操作指令',['throttle','brake']),('加速度 [m/s²]',['acceleration_mps2'])],
            speed_note='速度は簡易モデルの予測値です。実測値ではありません。')
    publish(args.output,save)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    for action in ('check','analyze'):
        p=sub.add_parser(action,help='Read a bag without publishing ROS topics')
        source=p.add_mutually_exclusive_group(required=True)
        source.add_argument('--bag',type=Path)
        source.add_argument('--normalized-json',type=Path,help='Portable extracted data/test fixture')
        p.add_argument('--config',type=Path,default=DEFAULT_CONFIG)
        p.add_argument('--session',type=Path,help='Optional JSON with tire/surface/battery and trial notes')
        p.add_argument('--output',required=True,type=Path)
    p=sub.add_parser('tune',help='Preview gains on a fitted scalar model and export a review-only YAML')
    p.add_argument('--analysis',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--base-config',type=Path,default=ROOT/'ros2_ws/src/kart_bringup/config/control/speed_controller.yaml')
    for key in ('kp','ki','kd','kff','brake-limit','throttle-limit'):
        p.add_argument('--'+key,type=float)
    p.add_argument('--targets',required=True,help='Comma-separated target speeds, m/s; start at first target')
    p.add_argument('--stage-seconds',type=float,default=4.)
    p.add_argument('--delays',help='Assumed delays in seconds, comma separated; e.g. 0.1,0.2,0.3')
    args=parser.parse_args(argv)
    try:
        if args.action=='tune': tune(args);return 0
        cfg=json.loads(args.config.read_text());validate_config(cfg)
        data=read_bag(args.bag,cfg) if args.bag else json.loads(args.normalized_json.read_text())
        if args.session:
            notes=json.loads(args.session.read_text())
            if not isinstance(notes,dict): raise ValueError('session must be a JSON object')
            data['session']=notes
        if args.action=='check':
            missing=[k for k in REQUIRED if not data.get(k)]
            result={'source':data.get('source'),'inventory':data.get('inventory',{}),
                    'missing_required_streams':missing,'config':cfg,
                    'note':'Presence check only; analyze additionally checks clocks, frames, freshness and usable segments.'}
            def save(out):
                write_json(out/'recording_check.json',result)
                body='<p>必須stream不足: '+html.escape(', '.join(missing) or 'なし')+'</p><p>存在確認だけです。時刻・座標系・有効区間はanalyzeで検査します。</p>'
                body+=table([{'topic':k,**v} for k,v in result['inventory'].items()])
                page(out/'report.html','kart 記録要件チェック',body,[],[])
            publish(args.output,save);return 2 if missing else 0
        summary,rows,segments,bins=analyze(data,cfg)
        summary['session']=data.get('session')
        summary['analysis_config_sha256']=hashlib.sha256(args.config.read_bytes()).hexdigest()
        def save(out):
            write_json(out/'normalized.json',data)
            analysis_report(out,summary,rows,segments,bins)
        publish(args.output,save)
        print(f"Selected {summary['selected_samples']}/{summary['samples']} samples; {summary['segments']} segments; model usable={summary['model']['usable']}")
        return 0 if summary['selected_samples'] else 2
    except (ValueError,KeyError,FileNotFoundError,ImportError) as error:
        parser.exit(2,f'Error: {error}\n')


if __name__=='__main__':
    raise SystemExit(main())
