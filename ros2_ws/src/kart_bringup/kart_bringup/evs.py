"""EVS startup configuration; no ROS or camera access."""
from pathlib import Path
import yaml

BACKENDS = ("cpu", "cuda", "cuda_async")
BACKEND_LABELS = {
    "cpu": "CPU同期処理: CPUでTensorを生成し、全体をGPUへ転送（cpu）",
    "cuda": "GPU同期処理: 取得callback内でGPU集計・Tensor生成（cuda）",
    "cuda_async": "非同期処理: workerで集計・Tensor生成。次に集計方式を選択（cuda_async）",
}
AGGREGATIONS = ("events", "evt3_runs", "evt3_runs_gpu", "evt3_runs_gpu_direct", "evt3_runs_grouped_gpu")
AGGREGATION_LABELS = {
    "events": "GPUイベント集計: CPUでデコードし、全イベントをGPUへ転送（events）",
    "evt3_runs": "CPU集計・全Tensor転送: CPUで直接集計・Tensor生成し、全体をGPUへ転送（evt3_runs）",
    "evt3_runs_gpu": "CPU集計・差分転送: 変更binだけGPUへ転送し、GPUでTensor生成（evt3_runs_gpu）",
    "evt3_runs_gpu_direct": "CPU集計・差分直接転送: 差分転送の中間コピーを省き、GPUでTensor生成（evt3_runs_gpu_direct）",
    "evt3_runs_grouped_gpu": "GPU圧縮データ集計: CPUで状態復元・run生成し、圧縮RAWとrunをGPUへ転送・集計（evt3_runs_grouped_gpu）",
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
