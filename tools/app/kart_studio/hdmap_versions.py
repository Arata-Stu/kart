"""Immutable HDMap-only versions sharing one localization map and point cloud."""
import copy
import time
import uuid

from .storage import atomic_json, fingerprint, name, read_json, within

FIELDS = ('lanes', 'obstacles', 'parking_areas', 'parking_drivable')
SCHEMA = 'kart.hdmap.version.v1'


class HDMapVersions:
    def __init__(self, maps, identity):
        self.maps, self.identity = maps, identity

    def folder(self, key):
        return within(self.maps.folder(key), '.hdmap-versions', exists=False)

    def read(self, key, version_id):
        path = within(self.folder(key), name(version_id) + '.json')
        entry = read_json(path)
        if (entry.get('schema') != SCHEMA or entry.get('id') != version_id
                or entry.get('document_fingerprint') != fingerprint(entry['document'])):
            raise ValueError('HDMap版のデータが変更されています')
        return entry

    def summary(self, key, entry):
        doc = entry['document']
        lane = next((v for v in doc['lanes'] if v['id'] == entry['lane_id']), None)
        return {k: entry[k] for k in ('id', 'name', 'created', 'lane_id', 'line_type')} | dict(
            revision=doc['revision'], lane_count=len(doc['lanes']),
            compatible=entry['map_id'] == self.identity(key),
            generated=bool(lane and entry['line_type'] in lane['lines']
                           and all('centerline' in v['lines'] for v in doc['lanes'])))

    def list(self, key):
        self.maps.folder(key)  # Require an existing confined map.
        root = self.folder(key)
        if not root.exists():
            return []
        return [self.summary(key, self.read(key, path.stem))
                for path in sorted(root.glob('*.json'))]

    def entry(self, key, doc, title, version_id, lane_id, line_type):
        if lane_id not in {v['id'] for v in doc['lanes']}:
            raise ValueError('保存するlaneを選択してください')
        if line_type not in ('centerline', 'raceline', 'customline'):
            raise ValueError('保存する参照ラインを選択してください')
        return dict(schema=SCHEMA, id=version_id, name=title, created=time.time(),
                    map_id=self.identity(key), document=copy.deepcopy(doc),
                    document_fingerprint=fingerprint(doc), lane_id=lane_id, line_type=line_type)

    def create(self, body):
        key = body['id']
        self.maps.require_snapshot(key)
        doc = self.maps.load(key)
        if doc['revision'] != body['revision']:
            raise ValueError('地図が更新されています。再読み込みしてください')
        title = body.get('name')
        if (not isinstance(title, str) or not title.strip() or title != title.strip()
                or len(title) > 64 or not title.isprintable()):
            raise ValueError('HDMapの版名は1〜64文字で指定してください')
        versions = [v for v in self.list(key) if v['id'] != 'previous']
        if len(versions) >= 32:
            raise ValueError('HDMapの保存版は32個までです')
        if any(v['name'].casefold() == title.casefold() for v in versions):
            raise ValueError('同じHDMap版名があります。別名を指定してください')
        entry = self.entry(key, doc, title, 'version_' + uuid.uuid4().hex,
                           body['lane_id'], body['line_type'])
        atomic_json(within(self.folder(key), entry['id'] + '.json', exists=False), entry)
        return self.summary(key, entry)

    def restore(self, body):
        key = body['id']
        self.maps.require_snapshot(key)
        current = self.maps.load(key)
        if current['revision'] != body['revision']:
            raise ValueError('地図が更新されています。再読み込みしてください')
        entry = self.read(key, body['version_id'])
        if entry['map_id'] != self.identity(key):
            raise ValueError('自己位置推定用地図が異なるHDMap版には切り替えられません')
        source = entry['document']
        if source.get('frame', 'map') != current.get('frame', 'map'):
            raise ValueError('HDMap版の座標frameが一致しません')
        # Keep a single reversible working copy in addition to the named versions.
        # Read the target first so restoring 'previous' swaps the two documents.
        previous = self.entry(key, current, '直前の作業用HDMap', 'previous',
                              body['lane_id'], body['line_type'])
        atomic_json(within(self.folder(key), 'previous.json', exists=False), previous)
        restored = copy.deepcopy(current)
        for field in FIELDS:
            restored[field] = copy.deepcopy(source.get(field, []))
        restored.update(revision=current['revision'] + 1, updated=time.time(),
                        hdmap_version=dict(id=entry['id'], name=entry['name'],
                                           source_revision=source['revision']))
        atomic_json(self.maps.folder(key) / 'map.json', restored)
        return dict(document=self.maps.load(key), lane_id=entry['lane_id'], line_type=entry['line_type'])
