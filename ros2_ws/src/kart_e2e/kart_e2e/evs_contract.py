"""EventState histogram input contract; safe to use without torch/ROS."""
import argparse
import json
from pathlib import Path


def validate_metadata(meta):
    if meta.get('format') != 'eventstate_encoder_bundle_v1':
        raise ValueError('Expected an EventState encoder export')
    if meta['architecture']['architecture'] != 'dinov3_vits16':
        raise ValueError('Expected ViT-S/16')
    ds = meta['dataset']
    rep = ds['representation']
    if (rep.get('temporal_interpolation') != 'none' or rep.get('window_us') != 40000
            or rep.get('event_bins') != 10 or rep.get('channel_layout') != 'polarity_major'
            or rep.get('voxel_normalization') != 'none' or ds.get('event_resize_mode') != 'sum_pool'):
        raise ValueError('Requires no-interpolation 40ms polarity-major histogram; linear weights are incompatible')
    h, w = ds.get('content_height', ds['input_height']), ds.get('content_width', ds['input_width'])
    if (h,w) not in {(120,160),(240,320)} or meta['input_shape'] != [1,20,((h+15)//16)*16,w]:
        raise ValueError('Unsupported histogram/model shape')
    stats = rep['fixed_normalization']
    if stats.get('event_resize_mode') != 'sum_pool' or stats.get('content_size') != [h,w]:
        raise ValueError('Histogram normalization resolution mismatch')
    return h,w


def main():
    p=argparse.ArgumentParser(description='Generate full OpenEB config from encoder metadata')
    p.add_argument('--model-json',type=Path,required=True)
    p.add_argument('--base-config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    import yaml
    meta=json.loads(a.model_json.read_text())
    h,w=validate_metadata(meta)
    config=yaml.safe_load(a.base_config.read_text())
    params=config['/**/event_tensor']['ros__parameters']
    params.update(width=w,height=h,bins=10,window_ms=40.0,stride_ms=4.0,output_rate_hz=250.0,
                  temporal_interpolation='none',polarity_mode='separate',polarity_layout='polarity_major',
                  channel_mean=[0.0],channel_stddev=[1.0],tensor_name='input_tensor')
    # Driver emits raw counts. Padding/normalization belong to encoder preprocessing.
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:
        f.write('# Generated from '+str(a.model_json)+'; raw histogram counts, normalization in model.\n')
        yaml.safe_dump(config,f,sort_keys=False)
    print(a.output)


if __name__=='__main__':
    main()
