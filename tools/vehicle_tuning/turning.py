"""Low-slip equivalent bicycle calibration; normalized steering is not wheel angle."""
import math
import numpy as np
from analysis import analyze
from report import page, table, write_csv, write_json


def identify(data, cfg, wheelbase):
    if not math.isfinite(wheelbase) or not 0 < wheelbase < 2:
        raise ValueError('wheelbase must be finite, in (0, 2) metres')
    summary, rows, segments, _ = analyze(data, cfg, turning=True)
    for row in rows:
        valid = not row['reason'] and row['segment'] is not None
        curvature = row['yaw_rate_rps']/row['speed_mps'] if valid else None
        row['curvature_per_m'] = curvature
        row['effective_steering_rad'] = math.atan(wheelbase*curvature) if valid else None
    trials = []
    for segment in segments:
        samples = [r for r in rows if r['segment'] == segment['id'] and not r['reason']]
        if not samples:
            continue
        trials.append(dict(id=segment['id'], samples=len(samples),
            steering=float(np.median([r['steering'] for r in samples])),
            speed_mps=float(np.median([r['speed_mps'] for r in samples])),
            curvature_per_m=float(np.median([r['curvature_per_m'] for r in samples])),
            effective_steering_rad=float(np.median([r['effective_steering_rad'] for r in samples])),
            angle_p10_rad=float(np.percentile([r['effective_steering_rad'] for r in samples],10)),
            angle_p90_rad=float(np.percentile([r['effective_steering_rad'] for r in samples],90))))
    model = dict(usable=False, reason='Need >= 6 constant-command segments per side with distinct commands')
    # Independent fits on each side expose left/right asymmetry; reserve whole trials.
    fits = {}
    for side, sign in [('left',1), ('right',-1)]:
        group = [r for r in trials if sign*r['steering'] > cfg['command_bin']]
        if len(group) < 6:
            continue
        train, val = group[:-max(2,len(group)//4)], group[-max(2,len(group)//4):]
        x = np.array([[r['steering'],1.] for r in train])
        y = np.array([r['effective_steering_rad'] for r in train])
        if np.linalg.matrix_rank(x) < 2 or np.linalg.cond(x) > 1000:
            continue
        coef = np.linalg.lstsq(x,y,rcond=None)[0]
        error = np.array([coef[0]*r['steering']+coef[1]-r['effective_steering_rad'] for r in val])
        domain = [min(r['steering'] for r in train),max(r['steering'] for r in train)]
        rmse = float(np.sqrt(np.mean(error**2)))
        fits[side] = dict(slope_rad=float(coef[0]), intercept_rad=float(coef[1]),
            neutral_extrapolation=-float(coef[1]/coef[0]) if coef[0] != 0 else None,
            validation_rmse_rad=rmse,
            observed_speed_range_mps=[min(r['speed_mps'] for r in group),max(r['speed_mps'] for r in group)],
            train_segment_ids=[r['id'] for r in train],
            validation_segment_ids=[r['id'] for r in val], command_range=domain,
            usable=bool(coef[0]>0 and rmse <= .05 and all(domain[0]<=r['steering']<=domain[1] for r in val)))
    if fits:
        model = dict(usable=len(fits)==2 and all(f['usable'] for f in fits.values()),
                     sides=fits, validation_limit_rad=.05)
    summary.update(model=model, wheelbase_m=wheelbase, analysis='turning')
    summary['warnings'] += [
        'Equivalent bicycle effective angle, not measured individual wheel angles.',
        'Low-speed, low-slip approximation. Tire parameters, steering delay, deadband and saturation are not identified.',
        'Side fits are linear local approximations; neutral extrapolation is not a calibrated trim.',
        'Pose derivatives include estimator and smoothing errors; offline poses do not provide online tracking diagnostics.']
    return summary, rows, trials


def save_report(out, summary, rows, trials):
    write_json(out/'summary.json',summary)
    write_json(out/'model.json',summary['model'])
    write_csv(out/'samples.csv',rows)
    write_csv(out/'turning_segments.csv',trials)
    import html, json
    body='<p>後輪軸基準の等価二輪モデル。実効舵角の概算で、実車設定は変更しません。</p>'
    body+='<p>採用点数: '+str(summary['selected_samples'])+'</p>'
    body+=table(trials)
    body+='<h2>検証・品質・除外理由</h2><pre>'+html.escape(json.dumps(summary,ensure_ascii=False,indent=2))+'</pre>'
    page(out/'report.html','kart 旋回特性',body,rows,[
        ('速度 [m/s]',['speed_mps']),('操舵指令',['steering']),
        ('旋回率 [rad/s]',['yaw_rate_rps']),('曲率 [1/m]',['curvature_per_m']),
        ('実効舵角 [rad]',['effective_steering_rad'])])
