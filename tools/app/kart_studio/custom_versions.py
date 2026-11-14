"""Named custom-line strategies; generated results remain server-owned."""
import copy

from .geometry import points
from .speed_sections import sections
from .storage import name

FIELDS = ('custom', 'custom_speeds', 'custom_speed_mode')


def same_controls(a, b):
    return all(a.get(key, 'profile' if key == 'custom_speed_mode' else []) ==
               b.get(key, 'profile' if key == 'custom_speed_mode' else []) for key in FIELDS)


def validate(row, old, lane, geometry_changed):
    raw = row.get('custom_versions', old.get('custom_versions', []))
    if not isinstance(raw, list) or len(raw) > 32:
        raise ValueError('Customの版はlaneごとに32個までです')
    previous = {v['id']: v for v in old.get('custom_versions', [])}
    result, ids, names = [], set(), set()
    for entry in raw:
        key = name(entry['id'])
        title = entry.get('name')
        if (not isinstance(title, str) or not title.strip() or title != title.strip()
                or len(title) > 64 or not title.isprintable()):
            raise ValueError('Customの版名は1〜64文字で指定してください')
        if key.casefold() in ids or title.casefold() in names:
            raise ValueError('Customの版ID・版名が重複しています')
        ids.add(key.casefold()); names.add(title.casefold())
        custom = points(entry.get('custom', []), minimum=0, maximum=1000)
        version = dict(id=key, name=title, custom=custom,
                       custom_speeds=sections(entry.get('custom_speeds', []), custom, lane['closed']),
                       custom_speed_mode=entry.get('custom_speed_mode', 'profile'))
        if version['custom_speed_mode'] not in ('profile', 'step'):
            raise ValueError('未知のCustom速度方式です')
        # Never accept a generated line supplied by the browser. Reuse only a
        # server-owned result for the same controls and unchanged boundaries.
        sources = [previous.get(key, {})]
        if key == old.get('custom_version') or key not in previous:
            sources.insert(0, {**old, 'line': old['lines'].get('customline')})
        if not geometry_changed:
            for source in sources:
                if source.get('line') and same_controls(version, source):
                    version['line'] = copy.deepcopy(source['line'])
                    break
        result.append(version)
    selected = row.get('custom_version', old.get('custom_version', ''))
    if not isinstance(selected, str) or (selected and selected not in {v['id'] for v in result}):
        raise ValueError('選択したCustomの版がありません')
    lane.update(custom_versions=result, custom_version=selected)
    # Switching versions restores the selected trusted result, including its
    # generation settings, even when two strategies share editing points.
    if selected and not geometry_changed:
        version = next(v for v in result if v['id'] == selected)
        if selected != old.get('custom_version') or 'customline' not in lane['lines']:
            lane['lines'].pop('customline', None)
            if version.get('line') and same_controls(lane, version):
                lane['lines']['customline'] = copy.deepcopy(version['line'])


def generated(lane):
    for version in lane.get('custom_versions', []):
        if version['id'] == lane.get('custom_version') and same_controls(lane, version):
            version['line'] = copy.deepcopy(lane['lines']['customline'])


def invalidate(lane):
    for version in lane.get('custom_versions', []):
        version.pop('line', None)


def live_document(document):
    """Only the selected working line crosses the bounded tuning protocol."""
    document = copy.deepcopy(document)
    for lane in document['lanes']:
        lane.pop('custom_versions', None)
        lane.pop('custom_version', None)
    return document
