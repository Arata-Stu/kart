"""Symmetric signed INT8 Q/DQ PTQ of static-weight Conv/Linear; FP32 I/O."""
import argparse
import json
from pathlib import Path
import tempfile

import numpy as np
import onnx
import onnxruntime as ort
from onnxruntime.quantization import CalibrationDataReader, CalibrationMethod, QuantFormat, QuantType, quantize_static

from .benchmark_data import DatasetInputs, digest
from .contract import model_contract


class Reader(CalibrationDataReader):
    def __init__(self,data): self.data=data;self.index=0
    def get_next(self):
        if self.index>=len(self.data): return None
        value={self.data.name:self.data.tensor(self.index)};self.index+=1
        return value
    def rewind(self): self.index=0


def validate_trt_qdq(graph, reference=None):
    """Reject ORT-valid integer bias DQ and enforce the signed INT8 TRT contract."""
    tensors={tensor.name:tensor for tensor in graph.graph.initializer}
    for node in graph.graph.node:
        if node.op_type not in ('QuantizeLinear','DequantizeLinear'): continue
        if len(node.input)<3 or node.input[2] not in tensors:
            raise ValueError(f'INT8 Q/DQ requires an explicit constant zero point: {node.name}')
        zero=tensors[node.input[2]]
        if zero.data_type!=onnx.TensorProto.INT8:
            raise ValueError(f'TensorRT INT8 Q/DQ cannot use INT32/other zero points (including bias): {node.name}')
        if np.any(onnx.numpy_helper.to_array(zero)):
            raise ValueError(f'TensorRT requires symmetric zero-point=0: {node.name}')
        scale=tensors.get(node.input[1])
        values=onnx.numpy_helper.to_array(scale) if scale is not None else None
        if values is None or scale.data_type!=onnx.TensorProto.FLOAT or not np.isfinite(values).all() or (values<=0).any():
            raise ValueError(f'Invalid FP32 positive Q/DQ scale: {node.name}')
        if node.op_type=='DequantizeLinear' and node.input[0] in tensors:
            if tensors[node.input[0]].data_type!=onnx.TensorProto.INT8:
                raise ValueError(f'TensorRT INT8 DequantizeLinear input must be INT8; keep bias FP32: {node.name}')
    if reference is not None:
        originals={tensor.name:tensor for tensor in reference.graph.initializer}
        for node in reference.graph.node:
            if node.op_type not in ('Conv','Gemm') or len(node.input)<3: continue
            name=node.input[2]
            if name not in originals: continue
            before,after=originals[name],tensors.get(name)
            if before.data_type!=onnx.TensorProto.FLOAT: continue
            if after is None or after.data_type!=onnx.TensorProto.FLOAT or not np.array_equal(onnx.numpy_helper.to_array(before),onnx.numpy_helper.to_array(after)):
                raise ValueError(f'TensorRT calibration must preserve the original FP32 bias: {name}')


def quantize(model, calibration, output, *, samples=256, seed=42, evaluation=None):
    model,output=Path(model),Path(output)
    contract,mode=model_contract(model)
    from .contract import routing_settings
    if routing_settings(model):
        raise ValueError("Section multi-head INT8 comparison is not yet supported; use FP32/FP16")
    source=Path(contract['model_file_path']);source_hash=digest(source)
    metadata_hash=digest(model/'metadata.json')
    meta=json.loads((model/'metadata.json').read_text())
    if meta.get('quantization'): raise ValueError('Select an original FP32 model, not an already quantized model')
    data=DatasetInputs(model,calibration,samples,seed)
    graph=onnx.load(source)
    if any(t.data_location==onnx.TensorProto.EXTERNAL for t in graph.graph.initializer):
        raise ValueError('External ONNX weights are not supported')
    initializers={t.name for t in graph.graph.initializer}
    selected=[]
    for i,node in enumerate(graph.graph.node):
        if not node.name: node.name=f'node_{i}'
        if (node.op_type in ('Conv','Gemm','MatMul') and len(node.input)>1
                and node.input[1] in initializers and 'head' not in node.name.lower()):
            selected.append(node.name)
    if not selected: raise ValueError('No supported static-weight Conv/Linear nodes to quantize')
    if output.exists(): raise FileExistsError(output)
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.int8-',dir=output.parent) as temporary:
        stage=Path(temporary)/'model';stage.mkdir()
        prepared=Path(temporary)/'prepared.onnx'
        onnx.save(graph,prepared)
        quantize_static(str(prepared),str(stage/'model.onnx'),Reader(data),quant_format=QuantFormat.QDQ,
                        activation_type=QuantType.QInt8,weight_type=QuantType.QInt8,per_channel=True,
                        calibrate_method=CalibrationMethod.MinMax,nodes_to_quantize=selected,
                        extra_options={'ActivationSymmetric':True,'WeightSymmetric':True,
                                       'CalibTensorRangeSymmetric':True,
                                       # ORT defaults to INT32 bias DQ, which TRT rejects.
                                       'QuantizeBias':False})
        quantized=onnx.load(stage/'model.onnx');onnx.checker.check_model(quantized)
        if not any(n.op_type=='QuantizeLinear' for n in quantized.graph.node):
            raise ValueError('Quantization did not produce Q/DQ nodes')
        for tensor in [*quantized.graph.input,*quantized.graph.output]:
            if tensor.type.tensor_type.elem_type!=onnx.TensorProto.FLOAT:
                raise ValueError('INT8 conversion changed FP32 I/O')
        validate_trt_qdq(quantized, graph)
        # CPU ORT preflight checks Q/DQ executability and numerical drift; NOT a GPU speed measurement.
        checked=evaluation or data
        options=ort.SessionOptions();options.intra_op_num_threads=2
        baseline=ort.InferenceSession(str(source),sess_options=options,providers=['CPUExecutionProvider'])
        candidate=ort.InferenceSession(str(stage/'model.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
        error=0.
        for i in range(min(8,len(checked))):
            value={checked.name:checked.tensor(i)}
            a=baseline.run(['control'],value)[0];b=candidate.run(['control'],value)[0]
            if b.shape!=a.shape or b.shape!=(1,1 if mode=='steer_only' else 2) or not np.isfinite(a).all() or not np.isfinite(b).all():
                raise ValueError('Invalid quantized prediction')
            error=max(error,float(np.abs(a-b).max()))
        data.verify();checked.verify()
        if digest(source)!=source_hash or digest(model/'metadata.json')!=metadata_hash: raise ValueError('Source model changed during calibration')
        meta.update(onnx_sha256=digest(stage/'model.onnx'),ort_max_abs_error=error,
                    ort_comparison='int8_qdq_vs_fp32_cpu_preflight_up_to_8_real_samples',
                    quantization=dict(format='QDQ',precision='int8',method='MinMax',
                                      activation='signed_symmetric_per_tensor',weight='signed_symmetric_per_channel',
                                      bias='fp32_unquantized',
                                      source_onnx_sha256=source_hash,seed=seed,calibration=data.provenance(),
                                      quantized_nodes=selected,
                                      excluded='head; activation-only attention MatMul; normalization; Softmax'))
        (stage/'metadata.json').write_text(json.dumps(meta,indent=2,allow_nan=False))
        model_contract(stage);stage.rename(output)
    return meta


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for field in ('model','calibration','output'): p.add_argument('--'+field,type=Path,required=True)
    p.add_argument('--samples',type=int,default=256);p.add_argument('--seed',type=int,default=42)
    quantize(**vars(p.parse_args()))


if __name__=='__main__': main()
