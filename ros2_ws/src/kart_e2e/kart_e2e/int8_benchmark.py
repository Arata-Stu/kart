"""Calibrate INT8 and benchmark FP32/INT8 on the same labeled inputs and GPU."""
import argparse
import csv
import json
from pathlib import Path
import tempfile

import numpy as np

from .benchmark_data import DatasetInputs, digest
from .contract import model_contract, runtime_settings, input_contract
from .engine_cache import fingerprint
from .evaluate import metrics
from .int8_quantize import quantize
from .trt_benchmark_runtime import Runner


def statistics(values):
    values=np.asarray(values,dtype=np.float64)
    if not values.size or not np.isfinite(values).all() or (values<0).any():
        raise ValueError('Invalid inference timings')
    return dict(mean=float(values.mean()),median=float(np.median(values)),p95=float(np.percentile(values,95)))


def accuracy(predictions,labels,runtime,mode):
    raw=np.asarray(predictions,dtype=np.float64)
    if raw.shape!=(len(labels),1 if mode=='steer_only' else 2) or not np.isfinite(raw).all():
        raise ValueError('Invalid TensorRT predictions')
    steer=raw[:,0]
    throttle=np.full(len(labels),runtime['fixed_throttle']) if mode=='steer_only' else raw[:,1]
    if (np.abs(steer)>1.00001).any() or (throttle < -1e-5).any() or (throttle>1.00001).any():
        raise ValueError('TensorRT output outside control range')
    result={}
    for key,value,target in [('steering_raw',steer,labels[:,0]),('steering_command',np.clip(steer,-1,1),labels[:,0]),
                             ('throttle_raw',throttle,labels[:,1]),('throttle_command',np.clip(throttle,0,runtime['max_throttle']),labels[:,1])]:
        result[key]={**metrics(value,target),'mse':float(np.mean((value-target)**2))}
    return result


def benchmark_pair(plans,model,data,*,runner_factory=Runner,warmup=10,repeats=3):
    _,mode=model_contract(model);runtime=runtime_settings(model)
    _,name,shape=input_contract(model)
    results={key:dict(wall=[],compute=[],predictions=None,layer_information='') for key in plans}
    # Alternate whole passes; only one engine/context is resident at a time.
    for repeat in range(repeats):
        order=list(plans) if repeat%2==0 else list(reversed(plans))
        for key in order:
            values=[]
            with runner_factory(plans[key],mode,name,shape) as runner:
                first=data.tensor(0)
                for _ in range(warmup): runner.predict(first)
                for index in range(len(data)):
                    prediction,wall,compute=runner.predict(data.tensor(index))
                    values.append(prediction[0])
                    results[key]['wall'].append(wall);results[key]['compute'].append(compute)
                results[key]['layer_information']=runner.layer_information
            if results[key]['predictions'] is None: results[key]['predictions']=np.asarray(values)
            print(f'TensorRT {key} benchmark pass {repeat+1}/{repeats}: {len(data)} real inputs',flush=True)
    data.verify()
    predictions={key:value['predictions'] for key,value in results.items()}
    reports={key:dict(metrics=accuracy(value['predictions'],data.labels,runtime,mode),
                      inference_ms=statistics(value['wall']),gpu_compute_ms=statistics(value['compute']),
                      engine_size_bytes=Path(plans[key]).stat().st_size,
                      int8_layer_info_detected='int8' in value['layer_information'].lower()) for key,value in results.items()}
    return reports,predictions,{key:value['layer_information'] for key,value in results.items()}


def run(model,calibration,evaluation,output,*,calibration_samples=256,evaluation_samples=256,seed=42,warmup=10,repeats=3):
    from .build_engine import build
    if not 1<=warmup<=1000 or not 1<=repeats<=20: raise ValueError('Invalid warmup/repeats')
    model,output=Path(model),Path(output)
    if output.exists(): raise FileExistsError(output)
    try:
        hardware=fingerprint() # Fail before calibration on systems without a real GPU/TRT.
    except (ImportError, OSError) as error:
        raise RuntimeError("INT8比較にはTensorRT/CUDAとNVIDIA GPUのあるkart Dockerが必要です（CPU速度への代用なし）") from error
    _,mode=model_contract(model);source_hash=digest(model/'model.onnx');metadata_hash=digest(model/'metadata.json')
    from .contract import routing_settings
    if routing_settings(model):
        raise ValueError('Section multi-head INT8 comparison is not yet supported; use FP32/FP16')
    data=DatasetInputs(model,evaluation,evaluation_samples,seed)
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.int8-benchmark-',dir=output.parent) as temporary:
        stage=Path(temporary)/'output'
        quantize(model,calibration,stage,samples=calibration_samples,seed=seed,evaluation=data)
        folder=stage/'benchmark';folder.mkdir()
        builds={}
        for precision,bundle in [('fp32',model),('int8',stage)]:
            builds[precision]=build(bundle,folder/precision,precision,notebook_only=False)
        plans={key:folder/key/'model.plan' for key in builds}
        reports,predictions,layers=benchmark_pair(plans,model,data,warmup=warmup,repeats=repeats)
        for key,text in layers.items(): (folder/(key+'-layers.json')).write_text(text)
        baseline,candidate=reports['fp32'],reports['int8']
        deltas={key:{metric:candidate['metrics'][key][metric]-baseline['metrics'][key][metric]
                     for metric in baseline['metrics'][key]} for key in baseline['metrics']}
        drift={axis:metrics(predictions['int8'][:,i],predictions['fp32'][:,i])
               for i,axis in enumerate(['steering'] if mode=='steer_only' else ['steering','throttle'])}
        overlap=bool(set(Path(p).resolve() for p in json.loads((stage/'metadata.json').read_text())['quantization']['calibration']['input_sha256']) & set(data.paths))
        report=dict(schema='kart.int8_benchmark.v1',mode=mode,hardware=hardware,precision='int8',io_dtype='fp32',
                    floating_point_fallback='fp16_allowed_for_int8; fp32_reference_tf32_disabled',
                    evaluation=data.provenance(),calibration=json.loads((stage/'metadata.json').read_text())['quantization']['calibration'],
                    calibration_evaluation_overlap=overlap,warmup_runs=warmup,repeats=repeats,
                    timing_scope='native TensorRT; same batch1 real inputs; CUDA event GPU compute; synchronized H2D+execute+D2H wall time; preprocessing/build/warmup excluded',
                    pass_order='fp32,int8 then int8,fp32 alternating; one resident engine at a time',
                    fp32=baseline,int8=candidate,metric_delta_int8_minus_fp32=deltas,prediction_drift=drift,
                    speedup_wall_median=baseline['inference_ms']['median']/candidate['inference_ms']['median'] if candidate['inference_ms']['median']>0 else None,
                    speedup_gpu_median=baseline['gpu_compute_ms']['median']/candidate['gpu_compute_ms']['median'] if candidate['gpu_compute_ms']['median']>0 else None,
                    engine_size_ratio=candidate['engine_size_bytes']/baseline['engine_size_bytes'],
                    build_reports=builds,source_onnx_sha256=source_hash,quantized_onnx_sha256=digest(stage/'model.onnx'),
                    samples=len(data),scope='open_loop_real_recorded_inputs_not_closed_loop_driving_accuracy')
        (folder/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
        with (folder/'predictions.csv').open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['sample_index','stamp_ns','steering_label','throttle_label','fp32_steering','int8_steering','fp32_throttle','int8_throttle'])
            for i,row in enumerate(data.rows):
                writer.writerow([data.indices[i],row['stamp_ns'],*data.labels[i],predictions['fp32'][i,0],predictions['int8'][i,0],
                                 predictions['fp32'][i,1] if mode!='steer_only' else runtime_settings(model)['fixed_throttle'],
                                 predictions['int8'][i,1] if mode!='steer_only' else runtime_settings(model)['fixed_throttle']])
        data.verify()
        if digest(model/'model.onnx')!=source_hash or digest(model/'metadata.json')!=metadata_hash:
            raise ValueError('FP32 model changed during benchmark')
        stage.rename(output)
    print(f'INT8 vs FP32 complete: steering MAE delta={deltas["steering_command"]["mae"]:+.6f}; wall speedup={report["speedup_wall_median"]}',flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('model','calibration','evaluation','output'): p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--calibration-samples',type=int,default=256);p.add_argument('--evaluation-samples',type=int,default=256)
    p.add_argument('--seed',type=int,default=42);p.add_argument('--warmup',type=int,default=10);p.add_argument('--repeats',type=int,default=3)
    run(**vars(p.parse_args()))


if __name__=='__main__': main()
