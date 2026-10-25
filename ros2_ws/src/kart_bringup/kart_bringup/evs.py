"""EVS startup configuration; no ROS or camera access."""
from pathlib import Path
import yaml

BACKENDS = ("cpu", "cuda", "cuda_async")
AGGREGATIONS = ("events", "evt3_runs", "evt3_runs_gpu", "evt3_runs_gpu_direct", "evt3_runs_grouped_gpu")
AGGREGATION_LABELS = {
    "events": "events: 既存decode → 全イベントをGPU集計",
    "evt3_runs": "evt3_runs: CPU直接集計 → 全Tensor転送",
    "evt3_runs_gpu": "evt3_runs_gpu: CPU直接集計 → 変更bin転送",
    "evt3_runs_gpu_direct": "evt3_runs_gpu_direct: CPU直接集計 → 中間コピー削減",
    "evt3_runs_grouped_gpu": "evt3_runs_grouped_gpu: 圧縮RAW＋run → GPU集計",
}
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


def pipeline_configuration(root, backend="", serial="", bias_file="", tensor_config="", aggregation=""):
    root = Path(root)
    selector = root / "openeb_tensor_pipeline.yaml"
    configured = yaml.safe_load(selector.read_text())["/**/tensor_pipeline"]["ros__parameters"]["tensor_backend"]
    if aggregation and aggregation not in AGGREGATIONS:
        raise ValueError(f"Invalid EVS aggregation: {aggregation}")
    backend = backend or ("cuda_async" if aggregation else configured)
    if aggregation and backend != "cuda_async":
        raise ValueError("EVS aggregation selection requires cuda_async")
    if backend not in BACKENDS:
        raise ValueError(f"Invalid EVS backend: {backend}")
    paths = [root / "openeb.yaml", root / f"openeb_tensor_{backend}.yaml",
             root / "openeb_visualization.yaml", selector]
    if tensor_config:
        paths[1] = Path(tensor_config).expanduser().resolve()
        tensor = yaml.safe_load(paths[1].read_text())["/**/event_tensor"]["ros__parameters"]
        if tensor.get("temporal_interpolation") != "none":
            raise ValueError("EVS runtime requires temporal_interpolation=none")
    for path in paths:
        if not path.is_file():
            raise ValueError(f"Missing EVS config: {path}")
    overrides = {"tensor_backend": backend}
    if aggregation:
        overrides["event_aggregation"] = aggregation
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
