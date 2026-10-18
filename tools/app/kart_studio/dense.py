"""Read-only, allowlisted downloads of official dense map artifacts."""

from urllib.parse import urlencode
import math

from kart_mapping.dense_outputs import DENSE_FILES
from .storage import read_json, within


def overlay(maps, key):
    """Use the original-map raster only; never guess a ground-frame transform."""
    manifest = maps.folder(key) / "mapping_result.json"
    if not manifest.is_file() or not read_json(manifest).get("dense"):
        return dict(available=False, reason="占有地図なし（点群のみで編集できます）")
    try:
        metadata = artifact(maps, key, "occupancy_map_before_adjustment.yaml")
        artifact(maps, key, "occupancy_map_before_adjustment.png")
    except (ValueError, FileNotFoundError):
        return dict(available=False, reason="元map座標の占有地図がありません")
    import yaml
    try:
        data = yaml.safe_load(metadata.read_text())
    except yaml.YAMLError as exc:
        raise ValueError("占有地図YAMLが不正です") from exc
    if not isinstance(data, dict):
        raise ValueError("占有地図YAMLはmappingである必要があります")
    resolution = float(data["resolution"])
    origin = [float(v) for v in data["origin"]]
    if len(origin) != 3 or not all(math.isfinite(v) for v in [resolution, *origin]) or resolution <= 0:
        raise ValueError("占有地図の原点・解像度が不正です")
    # Only the paired allowlisted PNG is served, never a path supplied by YAML.
    if str(data.get("image", "")).split("/")[-1] != "occupancy_map_before_adjustment.png":
        raise ValueError("占有地図YAMLとPNGの対応が不明です")
    return dict(available=True, resolution=resolution, origin=origin,
                url="/dense-file?" + urlencode(dict(id=key, file="occupancy_map_before_adjustment.png")),
                frame="map", source="before_adjustment")


def artifact(maps, key, filename):
    if filename not in DENSE_FILES:
        raise ValueError("Unknown dense artifact")
    folder = maps.folder(key)
    result = read_json(within(folder, "mapping_result.json"))
    if not result.get("dense"):
        raise ValueError("この地図にはdense成果物がありません")
    run = within(folder, result.get("dense_output", result["official_output"]))
    path = within(run, filename)
    if not path.is_file():
        raise ValueError("Artifact is not a file")
    return path


def catalog(maps, key):
    folder = maps.folder(key)
    manifest = folder / "mapping_result.json"
    result = read_json(manifest) if manifest.is_file() else {}
    summary = result.get("dense")
    if not summary:
        return dict(summary=None, files=[])
    files = []
    for filename in summary["files"]:
        path = artifact(maps, key, filename)
        files.append(dict(name=filename, bytes=path.stat().st_size,
                          url="/dense-file?" + urlencode(dict(id=key, file=filename))))
    return dict(summary=summary, files=files)
