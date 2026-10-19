"""Recoverable moves for learning artifacts; original relative identities survive."""
import os
import re
import time
import uuid

from .storage import atomic_json, read_json, within


def entry(learning, token):
    if not isinstance(token, str) or not re.fullmatch(r'[a-f0-9]{32}', token):
        raise ValueError('Invalid trash token')
    root = within(learning.root, '.trash/' + token)
    record = read_json(within(root, 'entry.json'))
    learning.relative_key(record['kind'], record['id'])
    within(root, 'data/artifact.json')
    return {**record, 'token': token}


def listing(learning):
    root = within(learning.root, '.trash', exists=False)
    if not root.exists():
        return []
    result = []
    for child in root.iterdir():
        try:
            result.append(entry(learning, child.name))
        except (ValueError, KeyError, OSError):
            continue
    return sorted(result, key=lambda row: row['deleted_at'], reverse=True)


def move(learning, kind, key):
    source = learning.folder(kind, key)
    token = uuid.uuid4().hex
    root = within(learning.root, '.trash/' + token, exists=False)
    root.mkdir(parents=True)
    record = dict(kind=kind, id=key, name=source.name, deleted_at=time.time())
    atomic_json(root / 'entry.json', record)
    os.rename(source, root / 'data')
    # Empty date/time folders add no value after their last artifact was moved.
    parent, boundary = source.parent, learning.root / kind
    while parent != boundary and parent.is_relative_to(boundary):
        try:
            parent.rmdir()
        except OSError:
            break
        parent = parent.parent
    return dict(record, token=token)


def restore(learning, token):
    record = entry(learning, token)
    relative = learning.relative_key(record['kind'], record['id'])
    destination = within(within(learning.root, record['kind']), relative, exists=False)
    if destination.exists():
        raise ValueError('同じ保存先に成果物があります。上書きせず復元を中止しました')
    destination.parent.mkdir(parents=True, exist_ok=True)
    root = within(learning.root, '.trash/' + token)
    os.rename(within(root, 'data'), destination)
    # Retain the tiny receipt; entries without data are omitted from the trash list.
    return dict(kind=record['kind'], id=record['id'])
