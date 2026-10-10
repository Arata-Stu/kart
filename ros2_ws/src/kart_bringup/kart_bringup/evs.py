"""EVS startup configuration; no ROS or camera access."""
from pathlib import Path
import yaml

BACKENDS = ("cpu", "cuda", "cuda_async")
DEFAULT_BIAS = "@default"


def bias_selection(value):
    if not value:
        return None  # Preserve driver YAML.
    if value == DEFAULT_BIAS:
        return ""  # Explicit camera defaults.
    path = Path(value).expanduser().resolve()
    if path.suffix.lower() != ".bias" or not path.is_file():
        raise ValueError(f"Readable .bias file required: {path}")
    with path.open("rb") as stream:
        if not stream.read(1):
            raise ValueError(f"Empty bias file: {path}")
    return str(path)


def bias_files(root):
    root = Path(root).expanduser()
    return sorted(p.resolve() for p in root.rglob("*.bias")
                  if p.is_file() and not any(x.startswith(".") for x in p.relative_to(root).parts))


def pipeline_configuration(root, backend="", serial="", bias_file=""):
    root = Path(root)
    selector = root / "openeb_tensor_pipeline.yaml"
    configured = yaml.safe_load(selector.read_text())["/**/tensor_pipeline"]["ros__parameters"]["tensor_backend"]
    backend = backend or configured
    if backend not in BACKENDS:
        raise ValueError(f"Invalid EVS backend: {backend}")
    paths = [root / "openeb.yaml", root / f"openeb_tensor_{backend}.yaml",
             root / "openeb_visualization.yaml", selector]
    for path in paths:
        if not path.is_file():
            raise ValueError(f"Missing EVS config: {path}")
    overrides = {"tensor_backend": backend}
    if serial:
        overrides["serial"] = serial
    bias = bias_selection(bias_file)
    if bias is not None:
        overrides["bias_file"] = bias
    else:
        configured_bias = yaml.safe_load(paths[0].read_text())["/**/event_camera_driver"]["ros__parameters"].get("bias_file", "")
        if configured_bias:
            bias_selection(configured_bias)
    return paths, overrides
