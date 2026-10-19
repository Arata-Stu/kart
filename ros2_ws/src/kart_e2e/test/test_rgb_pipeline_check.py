"""Independent HWC reference and recorded-tensor diagnostics; no ROS/GPU needed."""
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import torch
from PIL import Image
from kart_e2e.rgb_pipeline_check import compare,tensor_array
from kart_e2e.model import preprocess


def probe(tmp_path,corruption=None):
    folder=tmp_path/'probe';folder.mkdir()
    rgb=np.random.default_rng(42).integers(0,256,(240,424,3),dtype=np.uint8)
    # At exact 2x reduction, half-pixel bilinear equals the 2x2 pixel mean.
    resized=np.rint(rgb.reshape(120,2,212,2,3).astype(np.float32).mean((1,3))).astype(np.uint8)
    normalized=((resized.astype(np.float32)/255-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32))
    tensor=normalized.transpose(2,0,1)[None].copy()
    if corruption=='double_scale': tensor/=255
    if corruption=='bgr': tensor=tensor[:,::-1].copy()
    np.savez(folder/'0000.npz',rgb=rgb,resized_rgb=resized,tensor_input=tensor,prediction=np.array([[.25]],np.float32))
    (folder/'probe.json').write_text(json.dumps(dict(schema='kart.rgb_pipeline_probe.v1',frames=[dict(stamp_ns=123,file='0000.npz')])))
    return folder


def test_normalization_layout_and_resize_reference(tmp_path):
    report=compare(probe(tmp_path),tmp_path/'report.json')
    assert report['normalization_matches'] and report['resize_within_one_lsb']
    assert report['frames'][0]['training_to_ros_tensor_max_abs_error']<1e-5
    value=preprocess(Image.new('RGB',(424,240),(255,0,0)))
    assert value.shape==(3,128,224)
    assert torch.count_nonzero(value[:,:4])==0
    assert torch.count_nonzero(value[:,:,:6])==0
    torch.testing.assert_close(value[:,4,6],torch.tensor([(1-.485)/.229,-.456/.224,-.406/.225]))


@pytest.mark.parametrize('corruption',['double_scale','bgr'])
def test_detects_bad_pipeline(tmp_path,corruption):
    report=compare(probe(tmp_path,corruption),tmp_path/'bad.json')
    assert not report['normalization_matches']
    assert report['resize_within_one_lsb']


def test_serialized_tensor_validation():
    values=np.ones((1,3,120,212),np.float32)
    tensor=NS(dtype_code=2,dtype_bits=32,dtype_lanes=1,byte_offset=0,strides=[],shape=list(values.shape),data=values.tobytes())
    msg=NS(names=['image'],tensors=[tensor])
    np.testing.assert_array_equal(tensor_array(msg,'image',list(values.shape)),values)
    tensor.dtype_bits=16
    with pytest.raises(ValueError): tensor_array(msg,'image')


def test_optional_model_compares_recorded_output(tmp_path):
    import hashlib
    import onnx
    from onnx import helper,TensorProto,numpy_helper
    from kart_e2e.contract import SPEC
    folder=tmp_path/'model';folder.mkdir()
    graph=helper.make_graph([helper.make_node('Constant',[],['control'],value=numpy_helper.from_array(np.array([[.1]],np.float32)))],
                            'probe',[helper.make_tensor_value_info('image',TensorProto.FLOAT,[1,3,120,212])],
                            [helper.make_tensor_value_info('control',TensorProto.FLOAT,[1,1])])
    model=helper.make_model(graph,opset_imports=[helper.make_opsetid('',17)]);model.ir_version=9
    onnx.save(model,folder/'model.onnx')
    meta=dict(schema=1,model_spec=SPEC,output_mode='steer_only',input_name='image',input_shape=[1,3,120,212],
              output_name='control',output_shape=[1,1],runtime=dict(fixed_throttle=.1,max_throttle=.2),
              onnx_sha256=hashlib.sha256((folder/'model.onnx').read_bytes()).hexdigest())
    (folder/'metadata.json').write_text(json.dumps(meta))
    report=compare(probe(tmp_path),tmp_path/'report.json',folder)
    assert report['frames'][0]['input_difference_steering_effect']==0
    assert report['frames'][0]['tensorrt_vs_onnx_max_abs_error']==pytest.approx(.15)


def test_capture_matches_headers_across_bag_order(tmp_path,monkeypatch):
    from types import ModuleType
    from kart_e2e.rgb_pipeline_check import capture,TOPICS
    original=probe(tmp_path)
    with np.load(original/'0000.npz') as data:
        rgb,resized,tensor=data['rgb'],data['resized_rgb'],data['tensor_input']
    header=NS(stamp=NS(sec=0,nanosec=123))
    def image(array):
        return NS(header=header,encoding='rgb8',height=array.shape[0],width=array.shape[1],step=array.shape[1]*3,data=array.tobytes())
    def message(array,name):
        return NS(header=header,names=[name],tensors=[NS(dtype_code=2,dtype_bits=32,dtype_lanes=1,byte_offset=0,strides=[],shape=list(array.shape),data=array.tobytes())])
    rows=[(TOPICS['input'],message(tensor,'image'),500),
          (TOPICS['image'],image(rgb),600),(TOPICS['resized'],image(resized),700),
          (TOPICS['prediction'],message(np.array([[.2]],np.float32),'control'),800)]
    types={value:'sensor_msgs/msg/Image' if key in ('image','resized') else 'isaac_ros_tensor_msgs/msg/TensorList' for key,value in TOPICS.items()}
    class Reader:
        def open(self,*args): pass
        def get_all_topics_and_types(self): return [NS(name=k,type=v) for k,v in types.items()]
        def set_filter(self,selected): self.rows=iter(r for r in rows if r[0] in selected.topics);self.next=next(self.rows,None)
        def has_next(self): return self.next is not None
        def read_next(self):
            result=self.next;self.next=next(self.rows,None);return result
    import sys
    modules={key:ModuleType(key) for key in ('rosbag2_py','rclpy.serialization','rosidl_runtime_py.utilities')}
    ros=modules['rosbag2_py'];ros.SequentialReader=Reader
    for key in ('StorageOptions','ConverterOptions','StorageFilter'): setattr(ros,key,lambda *a,**kw:NS(**kw))
    modules['rclpy.serialization'].deserialize_message=lambda raw,_:raw
    modules['rosidl_runtime_py.utilities'].get_message=lambda _:None
    for key,module in modules.items():monkeypatch.setitem(sys.modules,key,module)
    output=tmp_path/'captured';capture(tmp_path/'bag',output,max_frames=1)
    result=compare(output,tmp_path/'report.json')
    assert result['samples']==1 and result['normalization_matches']
    assert json.loads((output/'probe.json').read_text())['frames'][0]['stamp_ns']==123
