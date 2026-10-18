"""Read-only, allowlisted downloads of official dense map artifacts."""

from urllib.parse import urlencode

from kart_mapping.dense_outputs import DENSE_FILES
from .storage import read_json, within


def artifact(maps, key, filename):
    if filename not in DENSE_FILES:
        raise ValueError("Unknown dense artifact")
    folder = maps.folder(key)
    result = read_json(within(folder, "mapping_result.json"))
    if not result.get("dense"):
        raise ValueError("この地図にはdense成果物がありません")
    run = within(folder, result["official_output"])
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
