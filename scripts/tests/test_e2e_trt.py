"""Interactive build precision, explicit flags and precision-aware cache reuse; no GPU."""
import importlib.util
import io
import json
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/lib'))
spec=importlib.util.spec_from_file_location('kart_trt_cli',ROOT/'scripts/lib/e2e_trt.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


@pytest.mark.parametrize('requested', [None,'--fp16','--fp32','--int8'])
@pytest.mark.parametrize('selected',['fp16','fp32'])
def test_model_then_precision_and_command(tmp_path,requested,selected):
    model=tmp_path/'policy';model.mkdir()
    (model/'model.onnx').write_bytes(b'onnx');(model/'metadata.json').write_text(json.dumps({'quantization':{'format':'QDQ','precision':'int8'}} if requested=='--int8' else {}))
    engine=model/'model.plan';trtexec=tmp_path/'trtexec';trtexec.touch()
    calls=[];commands=[]
    expected=requested.removeprefix('--') if requested else selected
    def choose(title,values,**kwargs):
        calls.append(title)
        return model if 'モデル' in title else selected
    class Process:
        def __init__(self,command,**kwargs):
            commands.append(command)
            Path(next(a.split('=',1)[1] for a in command if a.startswith('--saveEngine='))).write_bytes(b'engine')
            Path(next(a.split('=',1)[1] for a in command if a.startswith('--exportTimes='))).write_text('[{"computeMs":1.0}]')
            self.stdout=io.StringIO('fake build\n')
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def wait(self,**kwargs): return 0
    argv=['e2e_trt.py','--models-root',str(tmp_path)] + ([requested] if requested else [])
    with patch.object(sys,'argv',argv), \
         patch.object(module,'choose',side_effect=choose), \
         patch.object(module,'model_contract',return_value=({'model_file_path':str(model/'model.onnx'),'engine_file_path':str(engine)},'steer_only')), \
         patch.object(module,'runtime_settings'), \
         patch.object(module,'fingerprint',return_value={'tensorrt':'10.7'}), \
         patch.object(module,'reusable',return_value=False), \
         patch.object(module,'input_contract',return_value=('rgb','image',[1,3,120,212])), \
         patch.object(module,'validate_engine') as validate, \
         patch.object(module.shutil,'which',return_value=str(trtexec)), \
         patch.object(module.subprocess,'Popen',Process):
        module.main()
    assert calls == ['TensorRTを作るモデル'] + ([] if requested else ['TensorRT build精度'])
    assert ('--fp16' in commands[0])==(expected in ('fp16','int8'))
    assert ('--noTF32' in commands[0])==(expected=='fp32')
    assert json.loads(engine.with_suffix('.json').read_text())['precision']==expected
    validate.assert_called_once()


def test_explicit_precision_reuses_only_matching_engine(tmp_path):
    (tmp_path/'metadata.json').write_text('{}')
    engine=tmp_path/'model.plan';engine.write_bytes(b'old')
    engine.with_suffix('.json').write_text('{"precision":"fp16"}')
    paths={'model_file_path':str(tmp_path/'model.onnx'),'engine_file_path':str(engine)}
    binary=tmp_path/'trtexec';binary.touch()
    with patch.object(sys,'argv',['e2e_trt.py',str(tmp_path),'--fp16']), \
         patch.object(module,'model_contract',return_value=(paths,'steer_only')), \
         patch.object(module,'runtime_settings'), \
         patch.object(module,'fingerprint',return_value={'tensorrt':'10.7'}), \
         patch.object(module,'reusable',return_value=True), \
         patch.object(module,'choose') as choose, \
         patch.object(module.shutil,'which',return_value=str(binary)), \
         patch.object(module.subprocess,'Popen') as process:
        module.main()
    choose.assert_not_called();process.assert_not_called()


def test_precision_change_rebuild_failure_preserves_previous_engine(tmp_path):
    (tmp_path/'metadata.json').write_text('{}')
    engine=tmp_path/'model.plan';engine.write_bytes(b'old')
    engine.with_suffix('.json').write_text('{"precision":"fp16"}')
    binary=tmp_path/'trtexec';binary.touch()
    paths={'model_file_path':str(tmp_path/'model.onnx'),'engine_file_path':str(engine)}
    with patch.object(sys,'argv',['e2e_trt.py',str(tmp_path),'--fp32']), \
         patch.object(module,'model_contract',return_value=(paths,'steer_only')), \
         patch.object(module,'runtime_settings'), \
         patch.object(module,'fingerprint',return_value={'tensorrt':'10.7'}), \
         patch.object(module,'reusable',return_value=True), \
         patch.object(module.shutil,'which',return_value=str(binary)), \
         patch.object(module.subprocess,'Popen',side_effect=RuntimeError('failed')) as process:
        with pytest.raises(RuntimeError,match='failed'): module.main()
    assert '--noTF32' in process.call_args.args[0]
    assert engine.read_bytes()==b'old'
    assert json.loads(engine.with_suffix('.json').read_text())['precision']=='fp16'


@pytest.mark.parametrize('quantized,flag',[(True,'--fp32'),(True,'--fp16'),(False,'--int8')])
def test_quantization_cannot_be_enabled_without_calibration_or_undone(tmp_path,quantized,flag):
    (tmp_path/'metadata.json').write_text(json.dumps({'quantization':{'format':'QDQ','precision':'int8'}} if quantized else {}))
    with patch.object(sys,'argv',['e2e_trt.py',str(tmp_path),flag]),patch.object(module,'model_contract',return_value=({'engine_file_path':str(tmp_path/'model.plan')},'steer_only')),patch.object(module,'runtime_settings'),patch.object(module,'fingerprint') as fingerprint:
        with pytest.raises(ValueError):module.main()
    fingerprint.assert_not_called()
