"""Pure E2E configuration checks, without importing ROS or inference libraries."""

import json
import re
from pathlib import Path

import yaml


def parameters(path, name=None):
    key = "/**" if name is None else f"/**/{name}"
    return yaml.safe_load(Path(path).read_text())[key]["ros__parameters"]


def boolean(value):
    if str(value).lower() not in ("true", "false"):
        raise ValueError("Expected true or false")
    return str(value).lower() == "true"


def container_config(path, overrides):
    config = json.loads(Path(path).read_text())
    expected = {
        "container_name",
        "container_type",
        "create_container",
        "model_dir",
        "image_topic",
        "camera_info_topic",
    }
    if set(config) != expected:
        raise ValueError("Unexpected E2E launch configuration keys")
    config.update({key: value for key, value in overrides.items() if value != ""})
    name = config["container_name"]
    if not isinstance(name, str) or not re.fullmatch(
        r"/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*", name
    ):
        raise ValueError("Invalid component container name")
    config["create_container"] = boolean(config["create_container"])
    if config["create_container"] and "/" in name.lstrip("/"):
        raise ValueError("Namespaced container must be externally owned")
    if config["container_type"] not in ("multithreaded", "isolated_multithreaded"):
        raise ValueError("E2E requires a multithreaded container")
    config["container_name"] = "/" + name.lstrip("/")
    for key in ("image_topic", "camera_info_topic"):
        if not isinstance(config[key], str) or not config[key].startswith("/"):
            raise ValueError(f"{key} must be an absolute ROS topic")
    return config


def validate_encoder(path):
    values = {
        name: parameters(path, name)
        for name in (
            "e2e_resize",
            "e2e_format",
            "e2e_to_tensor",
            "e2e_normalize",
            "e2e_planar",
            "e2e_reshape",
        )
    }
    required = {
        "e2e_resize": {
            "output_width": 212,
            "output_height": 120,
            "interp_type": "linear",
            "keep_aspect_ratio": False,
            "disable_padding": True,
        },
        "e2e_format": {
            "encoding_desired": "rgb8",
            "image_width": 212,
            "image_height": 120,
        },
        "e2e_to_tensor": {"scale": True, "tensor_name": "image"},
        "e2e_normalize": {
            "mean": [0.485, 0.456, 0.406],
            "stddev": [0.229, 0.224, 0.225],
            "input_tensor_name": "image",
            "output_tensor_name": "image",
        },
        "e2e_planar": {
            "input_tensor_shape": [120, 212, 3],
            "output_tensor_name": "image",
        },
        "e2e_reshape": {
            "input_tensor_layout": "NCHW",
            "output_tensor_layout": "NCHW",
            "input_tensor_shape": [1, 3, 120, 212],
            "output_tensor_shape": [1, 3, 120, 212],
            "output_tensor_name": "image",
            "batch": 1,
        },
    }
    for name, contract in required.items():
        for key, expected in contract.items():
            if values[name].get(key) != expected:
                raise ValueError(
                    f"{name}.{key} breaks the exported preprocessing contract"
                )
    return values
