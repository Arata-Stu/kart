"""Studio annotations bound to server-owned line geometry; shared ROS-free rules."""
import importlib
import importlib.util
from pathlib import Path
import sys

_source = Path(__file__).resolve().parents[3] / 'ros2_ws/src/kart_hdmap/kart_hdmap'
_spec = importlib.util.spec_from_file_location('_kart_passing_contract', _source / '__init__.py',
                                             submodule_search_locations=[str(_source)])
sys.modules.setdefault(_spec.name, importlib.util.module_from_spec(_spec))
_module = importlib.import_module('_kart_passing_contract.passing_zones')
geometry_revision = _module.geometry_revision


def validate_passing_zones(zones, points, closed):
    try:
        return _module.validate_passing_zones(zones, points, closed)
    except ValueError as error:
        raise ValueError('追越し許可区間が不正です。対象ラインを確認して設定し直してください: ' + str(error)) from error


def decorate(document):
    """Expose server hashes. Invalid old annotations remain visible for deletion."""
    for lane in document['lanes']:
        for line in lane['lines'].values():
            line['geometry_revision'] = geometry_revision(line['points'], lane['closed'])
            line.setdefault('passing_zones', [])
            try:
                line['passing_zones'] = validate_passing_zones(line['passing_zones'], line['points'], lane['closed'])
                line.pop('passing_zones_error', None)
            except ValueError as error:
                line['passing_zones_error'] = str(error)
    return document


def save_annotations(lane, raw, previous):
    """Accept only zone edits, never client-supplied generated coordinates."""
    incoming = raw.get('lines', {})
    if not isinstance(incoming, dict):
        raise ValueError('生成ラインの形式が不正です')
    removed = sum(len(line.get('passing_zones', [])) for kind, line in previous.get('lines', {}).items()
                  if kind not in lane['lines'])
    if removed:
        lane['passing_zones_notice'] = f'形状・生成元の変更により追越し許可{removed}区間が失効しました。再生成後に設定してください。'
    elif previous.get('passing_zones_notice'):
        lane['passing_zones_notice'] = previous['passing_zones_notice']
    for kind, raw_line in incoming.items():
        if not isinstance(raw_line, dict) or 'passing_zones' not in raw_line:
            continue
        if kind not in lane['lines']:
            # Geometry edits intentionally invalidate the former generated line.
            # Do not accept permission indices for a replacement that does not exist.
            if kind not in previous.get('lines', {}) and raw_line['passing_zones']:
                raise ValueError('追越し許可は生成済みラインに設定してください')
            continue
        line = lane['lines'][kind]
        revision = geometry_revision(line['points'], lane['closed'])
        if raw_line.get('geometry_revision') not in (None, revision):
            raise ValueError('追越し許可の対象ラインが変更されています。再読み込みしてください')
        line['passing_zones'] = validate_passing_zones(raw_line['passing_zones'], line['points'], lane['closed'])
        line['geometry_revision'] = revision
        line.pop('passing_zones_error', None)
        if line['passing_zones']:
            line.pop('passing_zones_invalidated', None)
    # Unedited saved annotations also must be valid before another save/export.
    for line in lane['lines'].values():
        line['passing_zones'] = validate_passing_zones(line.get('passing_zones', []), line['points'], lane['closed'])
    return lane


def regenerated(result, previous, closed):
    result['geometry_revision'] = geometry_revision(result['points'], closed)
    old = previous or {}
    zones = old.get('passing_zones', [])
    if zones and geometry_revision(old['points'], closed) == result['geometry_revision']:
        result['passing_zones'] = validate_passing_zones(zones, result['points'], closed)
    else:
        result['passing_zones'] = []
        if zones:
            result['passing_zones_invalidated'] = f'再生成で形状が変わり、追越し許可{len(zones)}区間が失効しました。設定し直してください。'
    return result


def clear_transformed(lane):
    count = 0
    for line in lane['lines'].values():
        revision=geometry_revision(line['points'], lane['closed'])
        if revision != line.get('geometry_revision'):
            count += len(line.get('passing_zones', []))
            line['passing_zones'] = []
        line['geometry_revision'] = revision
        line.pop('passing_zones_error', None)
    if count:
        lane['passing_zones_notice'] = f'位置合わせで座標が変わり、追越し許可{count}区間が失効しました。設定し直してください。'
