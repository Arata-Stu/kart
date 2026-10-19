"""Real CPU Q/DQ conversion plus mocked GPU orchestration; no hardware claims."""
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np
import onnx
from PIL import Image
import pytest
import torch

from kart_e2e.benchmark_data import DatasetInputs, digest
from kart_e2e.int8_quantize import quantize
from kart_e2e.int8_benchmark import benchmark_pair, run, accuracy
from kart_e2e.model import Policy, backbone
from kart_e2e.export_onnx import export
from kart_e2e.contract import model_contract, runtime_settings


@pytest.fixture
def inputs(tmp_path):
    torch.set_num_threads(2);torch.manual_seed(42)
    policy=Policy(backbone('/unused','simple_cnn'),'steer_only',architecture='simple_cnn')
    policy.runtime={'fixed_throttle':.1,'max_throttle':.2}
    model=tmp_path/'model';export(policy,model)
    data=tmp_path/'data';data.mkdir()
    rng=np.random.default_rng(42);rows=[]
    for index in range(3):
        filename=f'{index}.png'
        Image.fromarray(rng.integers(0,256,(120,212,3),dtype=np.uint8)).save(data/filename)
        rows.append(dict(image=filename,stamp_ns=index*100000000,steering=.2,throttle=.1))
    (data/'metadata.json').write_text('{"modality":"rgb"}')
    (data/'samples.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
    return model,data


def test_real_qdq_preserves_fp32_contract_and_source(inputs,tmp_path):
    model,data=inputs;source=digest(model/'model.onnx');output=tmp_path/'int8'
    meta=quantize(model,data,output,samples=2)
    assert digest(model/'model.onnx')==source
    assert model_contract(output)[1]=='steer_only'
    assert runtime_settings(output)==runtime_settings(model)
    q=meta['quantization'];assert q['format']=='QDQ' and q['calibration']['samples']==2
    assert q['calibration']['input_sha256'] and len(q['quantized_nodes'])==4
    assert all('head' not in node for node in q['quantized_nodes'])
    graph=onnx.load(output/'model.onnx')
    assert graph.graph.input[0].type.tensor_type.elem_type==onnx.TensorProto.FLOAT
    assert graph.graph.output[0].type.tensor_type.elem_type==onnx.TensorProto.FLOAT
    for tensor in graph.graph.initializer:
        if tensor.name.endswith('_zero_point'): assert not np.any(onnx.numpy_helper.to_array(tensor))
    with pytest.raises(ValueError,match='already quantized'): quantize(output,data,tmp_path/'again')


def test_data_selection_contract_confinement_and_mutation(inputs,tmp_path):
    model,root=inputs
    data=DatasetInputs(model,root,2,42);assert data.indices==DatasetInputs(model,root,2,42).indices
    value=data.tensor(0);assert value.shape==(1,3,120,212) and value.dtype==np.float32
    data.paths[0].write_bytes(b'changed')
    with pytest.raises(ValueError,match='changed'): data.verify()
    (root/'metadata.json').write_text('{"modality":"evs"}')
    with pytest.raises(ValueError,match='modality'): DatasetInputs(model,root)
    (root/'metadata.json').write_text('{}')
    (root/'samples.jsonl').write_text(json.dumps(dict(image='../model/model.onnx',steering=0,throttle=0)))
    with pytest.raises(ValueError,match='outside'): DatasetInputs(model,root)


class FakeRunner:
    order=[];active=0
    def __init__(self,plan,*args):
        assert self.__class__.active==0
        self.__class__.active+=1
        self.key=Path(plan).parent.name;self.__class__.order.append(self.key)
        self.layer_information='{"dtype":"INT8"}' if self.key=='int8' else '{}'
    def __enter__(self):return self
    def __exit__(self,*args):self.__class__.active-=1
    def predict(self,value):
        assert value.dtype==np.float32
        return np.array([[.15 if self.key=='fp32' else .1]],np.float32), (4 if self.key=='fp32' else 2), (2 if self.key=='fp32' else 1)


def test_paired_pass_order_accuracy_timing_and_report(inputs,tmp_path):
    model,data=inputs;FakeRunner.order=[]
    def build(bundle,folder,precision,**kwargs):
        assert kwargs=={'notebook_only':False}
        folder.mkdir();(folder/'model.plan').write_bytes(b'plan')
        return {'precision':precision}
    def pair(*args,**kwargs):return benchmark_pair(*args,runner_factory=FakeRunner,**kwargs)
    with patch('kart_e2e.int8_benchmark.fingerprint',return_value={'gpu_name':'MOCK NO GPU'}), \
         patch('kart_e2e.build_engine.build',side_effect=build), \
         patch('kart_e2e.int8_benchmark.benchmark_pair',side_effect=pair):
        result=run(model,data,data,tmp_path/'result',warmup=1,repeats=2)
    assert FakeRunner.order==['fp32','int8','int8','fp32'] and FakeRunner.active==0
    assert result['calibration_evaluation_overlap'] and result['samples']==3
    assert result['speedup_wall_median']==2 and result['speedup_gpu_median']==2
    assert result['metric_delta_int8_minus_fp32']['steering_command']['mae']==pytest.approx(.05)
    assert result['prediction_drift']['steering']['mae']==pytest.approx(.05)
    assert result['int8']['int8_layer_info_detected']
    assert len((tmp_path/'result/benchmark/predictions.csv').read_text().splitlines())==4
    assert json.loads((tmp_path/'result/benchmark/report.json').read_text())['schema']=='kart.int8_benchmark.v1'
    model_contract(tmp_path/'result')


def test_no_cpu_speed_fallback(inputs,tmp_path):
    model,data=inputs
    with patch('kart_e2e.int8_benchmark.fingerprint',side_effect=RuntimeError('no GPU')), pytest.raises(RuntimeError,match='no GPU'):
        run(model,data,data,tmp_path/'result')
    assert not (tmp_path/'result').exists()


def test_invalid_output_rejected():
    with pytest.raises(ValueError,match='Invalid TensorRT'):
        accuracy(np.array([[np.nan]]),np.array([[0,.1]]),{'fixed_throttle':.1,'max_throttle':.2},'steer_only')


def test_evs_raw_count_calibration_contract(tmp_path):
    from evs_fixtures import metadata
    from kart_e2e.contract import evs_spec
    model=tmp_path/'model';model.mkdir();data=tmp_path/'data';data.mkdir()
    evs=metadata();spec=evs_spec(evs)
    x=onnx.helper.make_tensor_value_info('input_tensor',onnx.TensorProto.FLOAT,[1,20,120,160])
    y=onnx.helper.make_tensor_value_info('control',onnx.TensorProto.FLOAT,[1,1])
    nodes=[onnx.helper.make_node('Conv',['input_tensor','weights'],['features'],name='encoder/conv'),
           onnx.helper.make_node('GlobalAveragePool',['features'],['mean']),
           onnx.helper.make_node('Flatten',['mean'],['flat'],axis=1),
           onnx.helper.make_node('Tanh',['flat'],['control'])]
    weight=onnx.numpy_helper.from_array(np.full((1,20,1,1),.01,np.float32),'weights')
    graph=onnx.helper.make_model(onnx.helper.make_graph(nodes,'evs',[x],[y],[weight]),opset_imports=[onnx.helper.make_opsetid('',17)])
    graph.ir_version=10;onnx.save(graph,model/'model.onnx')
    meta=dict(schema=1,modality='evs',model_spec=spec,evs_metadata=evs,input_name='input_tensor',input_shape=[1,20,120,160],
              output_name='control',output_shape=[1,1],output_mode='steer_only',runtime={'fixed_throttle':.1,'max_throttle':.2},onnx_sha256=digest(model/'model.onnx'))
    (model/'metadata.json').write_text(json.dumps(meta))
    np.save(data/'counts.npy',np.ones((20,120,160),np.float32))
    (data/'metadata.json').write_text(json.dumps(dict(modality='evs',model_spec=spec)))
    (data/'samples.jsonl').write_text(json.dumps(dict(counts='counts.npy',steering=.2,throttle=.1,stamp_ns=0)))
    q=quantize(model,data,tmp_path/'int8')
    assert q['input_shape']==[1,20,120,160] and q['quantization']['calibration']['samples']==1
    np.testing.assert_array_equal(DatasetInputs(model,data).tensor(0),np.ones((1,20,120,160),np.float32))


@pytest.mark.skipif(not __import__('os').environ.get('KART_DINOV3_REPO'),reason='optional pinned DINO source required')
def test_real_vit_qdq_static_linear_excludes_control_head(tmp_path):
    import os
    torch.set_num_threads(2);torch.manual_seed(42)
    policy=Policy(backbone(os.environ['KART_DINOV3_REPO'],'dinov3_vitt16'),'steer_only',architecture='dinov3_vitt16')
    model=tmp_path/'model';export(policy,model)
    data=tmp_path/'data';data.mkdir();Image.new('RGB',(212,120),(123,40,90)).save(data/'image.png')
    (data/'metadata.json').write_text('{}')
    (data/'samples.jsonl').write_text(json.dumps(dict(image='image.png',steering=0,throttle=0,stamp_ns=0)))
    meta=quantize(model,data,tmp_path/'int8')
    selected=meta['quantization']['quantized_nodes']
    assert len(selected)>24 and any('qkv' in name for name in selected)
    assert not any('head' in name.lower() for name in selected)


def test_native_runner_cuda_bindings_timing_and_cleanup(tmp_path):
    import ctypes as C
    import sys
    from types import SimpleNamespace
    from unittest.mock import MagicMock
    from kart_e2e.trt_benchmark_runtime import Runner
    calls=[];counter=100
    class Cuda:
        def __getattr__(self,name):
            def call(*args):
                nonlocal counter
                calls.append(name)
                if name in ('cudaMalloc','cudaStreamCreate','cudaEventCreate'):
                    counter+=1;args[0]._obj.value=counter
                if name=='cudaEventElapsedTime':args[0]._obj.value=1.25
                if name=='cudaMemcpyAsync' and args[3]==2:
                    value=np.array([[.2]],np.float32);C.memmove(args[0],value.ctypes.data,4)
                return 0
            return call
    context=MagicMock();context.set_tensor_address.return_value=True;context.execute_async_v3.return_value=True
    engine=MagicMock();engine.create_execution_context.return_value=context
    engine.create_engine_inspector.return_value.get_engine_information.return_value='{}'
    runtime=MagicMock();runtime.deserialize_cuda_engine.return_value=engine
    logger=MagicMock();logger.WARNING=1
    trt=SimpleNamespace(Logger=logger,Runtime=MagicMock(return_value=runtime),LayerInformationFormat=SimpleNamespace(JSON=1))
    plan=tmp_path/'model.plan';plan.write_bytes(b'plan')
    with patch.dict(sys.modules,{'tensorrt':trt}), patch('kart_e2e.trt_benchmark_runtime.cuda_runtime',return_value=Cuda()),patch('kart_e2e.trt_benchmark_runtime.validate_engine'):
        with Runner(plan,'steer_only','image',[1,3,120,212]) as runner:
            value,wall,compute=runner.predict(np.zeros((1,3,120,212),np.float32))
            assert value[0,0]==pytest.approx(.2) and wall>=0 and compute==1.25
            assert context.set_tensor_address.call_args_list[0].args[0]=='image'
            assert context.set_tensor_address.call_args_list[1].args[0]=='control'
            with pytest.raises(ValueError):runner.predict(np.zeros((1,3,120,212),np.float16))
    assert calls.count('cudaFree')==2 and calls.count('cudaEventDestroy')==2
    assert calls.count('cudaStreamDestroy')==1
    assert calls.index('cudaStreamSynchronize')<calls.index('cudaEventElapsedTime')
