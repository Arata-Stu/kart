"""Use the selected localization map for Studio tuning; no second map prompt."""
from pathlib import Path


def map_root(map_file, localization_dir):
    from kart_hdmap.tuning import map_identity

    hdmap = Path(map_file).expanduser().resolve(strict=True)
    selected = Path(localization_dir).expanduser().resolve(strict=True)
    if selected.name == 'cuvslam_map':
        selected = selected.parent
    expected = map_identity(selected)
    for root in hdmap.parents:
        if (root / 'cuvslam_map').is_dir() or (root / 'snapshot.json').is_file():
            if map_identity(root) != expected:
                raise ValueError('動的調整: HDMapと選択した自己位置推定用地図の内容が一致しません')
            return str(root)
    raise ValueError('動的調整: HDMapと同じ転送フォルダーにcuvslam_mapまたはsnapshot.jsonが必要です')
