"""Read-only RGB pipeline audit: ROS bag capture -> offline numerical comparison.

Separate subcommands keep ROS serialization and the Torch/ORT environment independent.
No control commands are published and no model/configuration is changed.
"""
import argparse
import json
from pathlib import Path
import tempfile

import numpy as np

TOPICS = dict(image='/realsense/color/image_raw', resized='/e2e/rgb/image',
              input='/e2e/tensor_input', prediction='/e2e/tensor_output')


def tensor_array(msg, name, shape=None):
    if list(msg.names) != [name] or len(msg.tensors) != 1:
        raise ValueError('Expected one named tensor: '+name)
    tensor = msg.tensors[0]
    if (tensor.dtype_code != 2 or tensor.dtype_bits != 32 or tensor.dtype_lanes != 1
            or tensor.byte_offset != 0 or len(tensor.strides) != 0):
        raise ValueError('Expected contiguous FP32 tensor serialized on CPU')
    dimensions = list(tensor.shape)
    if shape is not None and dimensions != shape:
        raise ValueError(f'Unexpected {name} shape: {dimensions}')
    if not dimensions or any(v <= 0 for v in dimensions):
        raise ValueError('Invalid tensor dimensions')
    values = np.frombuffer(bytes(tensor.data), dtype=np.float32)
    if values.size != int(np.prod(dimensions)) or not np.isfinite(values).all():
        raise ValueError('Invalid serialized tensor size/values')
    return values.reshape(dimensions).copy()


def capture(bag, output, max_frames=16, **topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from .data import decode_image
    topics = {**TOPICS, **topics}
    bag,output=Path(bag),Path(output)
    if not 1 <= max_frames <= 256:
        raise ValueError('max_frames must be 1..256')
    if output.exists():
        raise FileExistsError(output)

    def records(selected):
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=''),rosbag2_py.ConverterOptions('',''))
        types={t.name:t.type for t in reader.get_all_topics_and_types()}
        expected={topics[key]:('sensor_msgs/msg/Image' if key in ('image','resized') else
                               'isaac_ros_tensor_msgs/msg/TensorList') for key in selected}
        for topic,kind in expected.items():
            if types.get(topic)!=kind:
                raise ValueError(f'Missing/wrong topic {topic}; expected {kind}')
        reader.set_filter(rosbag2_py.StorageFilter(topics=list(expected)))
        while reader.has_next():
            topic,raw,_=reader.read_next()
            msg=deserialize_message(raw,get_message(types[topic]))
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            yield topic,stamp,msg

    images={}
    for _,stamp,msg in records(['image']):
        images[stamp]=dict(rgb=np.asarray(decode_image(msg)))
        if len(images)>=max_frames:
            break
    # Match the source header stamp, never bag receipt order or nearest frames.
    for topic,stamp,msg in records(['resized','input','prediction']):
        if stamp not in images:
            continue
        if topic==topics['resized']:
            images[stamp]['resized_rgb']=np.asarray(decode_image(msg))
        elif topic==topics['input']:
            images[stamp]['tensor_input']=tensor_array(msg,'image',[1,3,120,212])
        else:
            prediction=tensor_array(msg,'control')
            if prediction.shape not in ((1,1),(1,2)):
                raise ValueError('Expected control [1,1] or [1,2]')
            images[stamp]['prediction']=prediction
    complete=[(stamp,row) for stamp,row in images.items() if 'resized_rgb' in row and 'tensor_input' in row]
    if not complete:
        raise ValueError('No matching source/resized/tensor stamps. Record all four topics for several seconds.')
    output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.rgb-probe-',dir=output.parent) as directory:
        stage=Path(directory)/'data';stage.mkdir()
        rows=[]
        for i,(stamp,arrays) in enumerate(complete):
            filename=f'{i:04d}.npz'
            np.savez(stage/filename,**arrays)
            rows.append(dict(stamp_ns=stamp,file=filename))
        manifest=dict(schema='kart.rgb_pipeline_probe.v1',bag=str(bag.resolve()),topics=topics,
                      selected_images=len(images),matched_frames=len(rows),missing_frames=len(images)-len(rows),frames=rows)
        (stage/'probe.json').write_text(json.dumps(manifest,indent=2))
        stage.rename(output)
    print(json.dumps({k:v for k,v in manifest.items() if k!='frames'},indent=2),flush=True)


def compare(probe, output, model_dir=None):
    from PIL import Image
    from .model import preprocess
    probe,output=Path(probe).resolve(),Path(output)
    if output.exists():
        raise FileExistsError(output)
    manifest=json.loads((probe/'probe.json').read_text())
    if manifest.get('schema')!='kart.rgb_pipeline_probe.v1' or not manifest.get('frames'):
        raise ValueError('Invalid RGB probe manifest')
    session=None
    if model_dir is not None:
        import onnxruntime as ort
        from .contract import model_contract, input_contract
        contract,_=model_contract(model_dir)
        if input_contract(model_dir)[0]!='rgb':
            raise ValueError('RGB model required')
        options=ort.SessionOptions();options.intra_op_num_threads=2
        session=ort.InferenceSession(contract['model_file_path'],sess_options=options,providers=['CPUExecutionProvider'])
    rows=[]
    for row in manifest['frames']:
        path=(probe/row['file']).resolve(strict=True)
        if not path.is_relative_to(probe):
            raise ValueError('Probe file outside directory')
        with np.load(path,allow_pickle=False) as data:
            rgb,resized,actual=data['rgb'],data['resized_rgb'],data['tensor_input']
            if (rgb.dtype!=np.uint8 or rgb.ndim!=3 or rgb.shape[2]!=3 or
                    resized.dtype!=np.uint8 or resized.shape!=(120,212,3) or
                    actual.dtype!=np.float32 or actual.shape!=(1,3,120,212) or not np.isfinite(actual).all()):
                raise ValueError('Invalid probe layout')
            training=preprocess(Image.fromarray(rgb))[:,4:-4,6:-6].unsqueeze(0).numpy()
            # Actual resized image removes GPU resize rounding from the normalization check.
            normalized=preprocess(Image.fromarray(resized))[:,4:-4,6:-6].unsqueeze(0).numpy()
            normalize_error=float(np.abs(actual-normalized).max())
            # Account for up to 1 uint8 LSB of GPU/CPU interpolation rounding.
            pixel_difference=np.abs(training-normalized)*np.asarray([.229,.224,.225],dtype=np.float32)[None,:,None,None]*255
            result=dict(stamp_ns=row['stamp_ns'],normalization_max_abs_error=normalize_error,
                        normalization_matches=bool(normalize_error<=1e-5),
                        resize_max_pixel_difference=float(pixel_difference.max()),
                        resize_within_one_lsb=bool(pixel_difference.max()<=1.01),
                        training_to_ros_tensor_max_abs_error=float(np.abs(actual-training).max()),
                        ros_channel_mean=actual.mean(axis=(0,2,3)).tolist())
            if session is not None:
                from_training=session.run(['control'],{'image':training})[0]
                from_ros=session.run(['control'],{'image':actual})[0]
                result.update(onnx_from_training=from_training.tolist(),onnx_from_ros_input=from_ros.tolist(),
                              input_difference_steering_effect=float(abs(from_training[0,0]-from_ros[0,0])))
                if 'prediction' in data:
                    prediction=data['prediction']
                    if prediction.shape!=from_ros.shape or not np.isfinite(prediction).all():
                        raise ValueError('Recorded TensorRT output/model shape mismatch')
                    result.update(recorded_tensorrt=prediction.tolist(),
                                  tensorrt_vs_onnx_max_abs_error=float(np.abs(prediction-from_ros).max()))
            rows.append(result)
    report=dict(schema='kart.rgb_pipeline_check.v1',frames=rows,samples=len(rows),
                normalization_matches=all(r['normalization_matches'] for r in rows),
                resize_within_one_lsb=all(r['resize_within_one_lsb'] for r in rows),
                model_dir=str(Path(model_dir).resolve()) if model_dir else None,
                scope='recorded_ros_tensors_vs_training; one_lsb_resize_is_a_rounding_tolerance_not_proof_of_gpu_kernel_parity')
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps(report,indent=2),flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    capture_parser=sub.add_parser('capture')
    capture_parser.add_argument('--bag',type=Path,required=True)
    capture_parser.add_argument('--output',type=Path,required=True)
    capture_parser.add_argument('--max-frames',type=int,default=16)
    for key,value in TOPICS.items():
        capture_parser.add_argument('--'+key+'-topic',dest=key,default=value)
    check_parser=sub.add_parser('compare')
    check_parser.add_argument('--probe',type=Path,required=True)
    check_parser.add_argument('--output',type=Path,required=True)
    check_parser.add_argument('--model-dir',type=Path)
    args=vars(parser.parse_args());action=args.pop('action')
    (capture if action=='capture' else compare)(**args)


if __name__=='__main__':
    main()
