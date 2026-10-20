"""Lightweight export contract: safe to import in launch without torch/onnx/CUDA."""

import hashlib
import json
from pathlib import Path

DINOV3_COMMIT = "adc254450203739c8149213a7a69d8d905b4fcfa"
SPEC = {
    "schema": 2,
    "encoder": "dinov3_vits16",
    "width": 212,
    "height": 120,
    "preprocess": "rgb_linear_halfpixel_noaa_uint8_imagenet_center_zero_pad16_v2",
    "encoder_commit": DINOV3_COMMIT,
}


RGB_ENCODERS = ("dinov3_vits16", "dinov3_vitt16", "dinov3_vitb16", "simple_cnn")


def rgb_spec(encoder="dinov3_vits16"):
    if encoder not in RGB_ENCODERS:
        raise ValueError("Unsupported RGB encoder")
    return dict(SPEC, encoder=encoder, encoder_commit="kart.simple_cnn.v1" if encoder == "simple_cnn" else DINOV3_COMMIT)


def validate_rgb_spec(spec):
    if not isinstance(spec, dict) or spec != rgb_spec(spec.get("encoder")):
        raise ValueError("Unsupported RGB model contract")
    return spec


def evs_spec(meta):
    """Canonical raw-count policy input; no Torch/ROS needed by launch."""
    import math

    try:
        ds, arch = meta['dataset'], meta['architecture']
        rep = ds['representation']
        stats = rep['fixed_normalization']
        h, w = ds['content_height'], ds['content_width']
        sh, sw = ds['sensor_height'], ds['sensor_width']
        expected_histogram = dict(
            type='voxel_grid', channels=20, event_bins=10, polarity_split=True,
            channel_order='positive_0,...,positive_B-1,negative_0,...,negative_B-1',
            normalization='none', temporal_interpolation='none', histogram_version=2,
            window_us=40000, window_boundary='[start,end)')
        if (meta['format'] != 'eventstate_encoder_bundle_v1'
                or meta['runtime']['dinov3_repository'] != 'facebookresearch/dinov3:' + DINOV3_COMMIT
                or arch['architecture'] != 'dinov3_vits16'
                or type(h) is not int or type(w) is not int
                or (h, w) not in ((120, 160), (240, 320))
                or meta['input_shape'] != [1, 20, ((h+15)//16)*16, w]
                or type(sh) is not int or type(sw) is not int
                or sh <= 0 or sw <= 0 or sh % h or sw % w or sh//h != sw//w
                or rep['temporal_interpolation'] != 'none'
                or rep['window_us'] != 40000 or rep['event_bins'] != 10
                or rep['channel_layout'] != 'polarity_major'
                or rep['voxel_normalization'] != 'none'
                or ds['event_resize_mode'] != 'sum_pool'
                or stats['event_resize_mode'] != 'sum_pool'
                or stats['content_size'] != [h, w]
                or stats['representation']['histogram_version'] != 2
                or stats['representation']['window_boundary'] != '[start,end)'
                or stats['representation'] != expected_histogram
                or stats.get('format') != 'eventstate_fixed_normalization_v1'
                or stats.get('sensor_size') != [sh, sw]
                or stats.get('includes_zero_pixels') is not True):
            raise ValueError('Incompatible EVS histogram encoder contract')
        mean, std = stats['mean'], stats['std']
        if (len(mean) != 20 or len(std) != 20
                or not all(type(x) in (int, float) and math.isfinite(x) for x in mean+std)
                or not all(x > 0 for x in std)):
            raise ValueError('Invalid EVS normalization')
    except (KeyError, TypeError, ZeroDivisionError) as error:
        raise ValueError('Incomplete EVS encoder metadata (v2 histogram statistics required)') from error
    return dict(schema=1, modality='evs', encoder='dinov3_vits16',
                encoder_commit=DINOV3_COMMIT, channels=20, height=h, width=w,
                sensor_shape=[sh, sw], window_us=40000, bins=10,
                window_boundary='[start,end)', polarity='positive_then_negative',
                coordinates='raw_sensor', spatial_resize='sum_pool',
                preprocess='raw_counts_center_pad_then_fixed_normalization_v1',
                padded_shape=meta['input_shape'], mean=mean, std=std)


def routing_settings(directory):
    metadata = json.loads((Path(directory) / 'metadata.json').read_text())
    routing = metadata.get('routing')
    if routing is not None:
        from .sections import Router
        Router(routing)
    return routing


def output_width(mode, routing=None):
    return (1 if mode == 'steer_only' else 2) * (len(routing['heads']) if routing else 1)


def input_contract(directory):
    """Return modality, input binding name and FP32 shape after validation."""
    model_contract(directory)
    meta = json.loads((Path(directory).expanduser() / 'metadata.json').read_text())
    return meta.get('modality', 'rgb'), meta['input_name'], meta['input_shape']


def model_contract(directory):
    if not directory:
        raise ValueError(
            "model_dir is required: export a kart checkpoint to ONNX first"
        )
    directory = Path(directory).expanduser().resolve(strict=True)
    path = directory / "model.onnx"
    metadata = json.loads((directory / "metadata.json").read_text())
    mode = metadata.get("output_mode")
    routing = routing_settings(directory)
    modality = metadata.get('modality', 'rgb')
    if modality == 'evs':
        spec = evs_spec(metadata.get('evs_metadata', {}))
        name, shape = 'input_tensor', [1, 20, spec['height'], spec['width']]
    elif modality == 'rgb':
        spec, name, shape = validate_rgb_spec(metadata.get('model_spec')), 'image', [1, 3, 120, 212]
    else:
        raise ValueError('Unknown E2E modality')
    if (
        metadata.get("schema") != 1
        or metadata.get("model_spec") != spec
        or mode not in ("steer_only", "steer_throttle")
        or metadata.get("input_name") != name
        or metadata.get("input_shape") != shape
        or metadata.get("output_name") != "control"
        or metadata.get("output_shape") != [1, output_width(mode, routing)]
    ):
        raise ValueError("ONNX metadata does not match the kart E2E contract")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != metadata.get("onnx_sha256"):
        raise ValueError("ONNX checksum mismatch: re-export the model")
    return {
        "model_file_path": str(path),
        "engine_file_path": str(directory / f"model_{digest[:12]}.plan"),
    }, mode


def validate_runtime(values):
    import math

    if not isinstance(values, dict) or set(values) != {
        "fixed_throttle",
        "max_throttle",
    }:
        raise ValueError(
            "Model runtime metadata missing: re-export with fixed/max throttle settings"
        )
    if any(
        type(v) not in (int, float) or not math.isfinite(v) for v in values.values()
    ):
        raise ValueError("Throttle settings must be finite numbers")
    if not 0 <= values["fixed_throttle"] <= values["max_throttle"] <= 1:
        raise ValueError("Require 0 <= fixed_throttle <= max_throttle <= 1")
    return dict(values)


def runtime_settings(directory):
    metadata = json.loads((Path(directory) / "metadata.json").read_text())
    return validate_runtime(metadata.get("runtime"))
