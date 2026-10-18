"""Read-only, allowlisted downloads of official dense map artifacts."""

from urllib.parse import urlencode
import math

from kart_mapping.dense_outputs import DENSE_FILES
from .storage import read_json, within


def mesh_overlay(maps, key):
    """The official pre-ground mesh uses the original reconstruction map frame."""
    folder = maps.folder(key)
    manifest = folder / 'mapping_result.json'
    result = read_json(manifest) if manifest.is_file() else {}
    if result.get('backend') != 'nvidia.create_map_offline':
        raise ValueError('公式offline生成元を確認できないためmeshの座標系を決定できません')
    doc = maps.load(key)
    if doc.get('frame') != 'map':
        raise ValueError('HDMapのframeがmapではありません')
    filename = 'mesh_before_adjustment.ply'
    color = result.get('mesh_color', {})
    colored = color.get('dense_source') == result.get('dense_output', result.get('official_output'))
    if colored and color.get('output'):
        filename = 'mesh_rgb.ply'
    try:
        path = artifact(maps, key, filename)
    except (ValueError, OSError) as exc:
        raise ValueError('地面合わせ前のmeshがありません。占有地図・メッシュを生成してください') from exc
    if path.stat().st_size > 256 * 1024 * 1024:
        raise ValueError('meshが256 MiBを超えています。外部PLYビューアを利用してください')
    with path.open('rb') as stream:
        header = stream.read(65536).split(b'end_header', 1)
    if len(header) != 2 or header[0].splitlines()[:1] != [b'ply']:
        raise ValueError('PLY headerが不正です')
    counts = {}
    for line in header[0].splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[0] == b'element' and parts[1] in (b'vertex', b'face'):
            counts[parts[1].decode()] = int(parts[2])
    if not 0 < counts.get('vertex', 0) <= 5_000_000 or not 0 < counts.get('face', 0) <= 10_000_000:
        raise ValueError('空のmeshまたはブラウザ表示上限を超えたmeshです')
    return dict(frame='map', source='before_adjustment', units='m',
                transform_policy='native_map_no_additional_transform',
                hdmap_dimension=2, colored=bool(colored), color_report=color.get('report') if colored else None, **counts,
                url='/dense-file?' + urlencode(dict(id=key, file=filename)),
                bytes=path.stat().st_size,
                stale=bool(result.get('dense_stale')))


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
    if filename == 'mesh_rgb.ply':
        folder = maps.folder(key)
        result = read_json(within(folder, 'mapping_result.json'))
        color = result.get('mesh_color', {})
        if color.get('dense_source') != result.get('dense_output', result.get('official_output')):
            raise ValueError('着色meshは旧meshの成果物です。再着色してください')
        return within(folder, color['output'])
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
    color = result.get('mesh_color', {})
    if color.get('output') and color.get('dense_source') == result.get('dense_output', result.get('official_output')):
        path = artifact(maps, key, 'mesh_rgb.ply')
        files.append(dict(name='mesh_rgb.ply', bytes=path.stat().st_size,
                          url='/dense-file?' + urlencode(dict(id=key, file='mesh_rgb.ply'))))
    return dict(summary=summary, files=files)
