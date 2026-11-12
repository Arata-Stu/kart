"""Bounded, local annotation prioritisation; never writes annotation state.

Scores are heuristic priorities / feature similarities, not RC-car probabilities.
Heavy image/model imports happen only inside a worker invocation.
"""
import argparse
from bisect import bisect_left
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import sys

from . import perception_runtime as runtime
from .storage import atomic_json


MAX_IMAGES = 500
MAX_SEED_POSITIVES = 32
MAX_SEED_NEGATIVES = 16
MAX_SEED_NEGATIVE_VIEWS = 32
SEED = 0


def _random(identity):
    return int.from_bytes(hashlib.sha256(f'{SEED}:{identity}'.encode()).digest()[:8], 'big') / 2**64


def _frames(value):
    if not isinstance(value, list) or len(value) > runtime.MAX_FRAMES:
        raise ValueError('選定元はproject全体で最大5000画像です')
    result, ids = [], set()
    for row in value:
        if not isinstance(row, dict):
            raise ValueError('画像metadataが不正です')
        identity = row.get('id')
        if (not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', identity)
                or identity in ids):
            raise ValueError('画像IDが不正か重複しています')
        ids.add(identity)
        if row.get('split') not in ('train', 'val') or type(row.get('reviewed')) is not bool:
            raise ValueError('画像のsplit / reviewedが不正です')
        if not isinstance(row.get('source_id'), str) or not row['source_id'] or len(row['source_id']) > 128:
            raise ValueError('画像のsource_idが不正です')
        width, height = row.get('width'), row.get('height')
        if (type(width) is not int or type(height) is not int or min(width, height) < 1
                or max(width, height) > 8192 or width * height > 16_777_216):
            raise ValueError('画像のサイズmetadataが不正です')
        stamp = row.get('stamp_ns')
        if (type(stamp) not in (int, str) or not re.fullmatch(r'-?[0-9]{1,30}', str(stamp))):
            raise ValueError('画像のstamp_nsが不正です')
        record_stamp = row.get('record_stamp_ns', stamp)
        if (type(record_stamp) not in (int, str) or not re.fullmatch(r'-?[0-9]{1,30}', str(record_stamp))):
            raise ValueError('画像のrecord_stamp_nsが不正です')
        boxes = row.get('boxes', [])
        if not isinstance(boxes, list) or len(boxes) > runtime.MAX_BOXES:
            raise ValueError('画像ごとのbboxは100件以下です')
        for box in boxes:
            if (not isinstance(box, list) or len(box) != 4
                    or not all(type(v) in (int, float) and math.isfinite(v) for v in box)
                    or not 0 <= box[0] < box[2] <= width or not 0 <= box[1] < box[3] <= height):
                raise ValueError('画像のbbox座標が不正です')
        path = row.get('path')
        if not isinstance(path, str) or not Path(path).is_absolute():
            raise ValueError('画像ファイルは絶対パスで指定してください')
        result.append(dict(row, stamp_ns=int(stamp), record_stamp_ns=int(record_stamp)))
    return result


def _time(frame):
    # Bag recording time remains usable if an image header clock freezes/resets.
    return int(frame.get('record_stamp_ns', frame['stamp_ns']))


def balanced_frames(frames, budget):
    """Round-robin source quotas, then nearest frames to uniform time-bin centres.

    Only these frames are decoded/inferred. A densely sampled beginning of a bag
    cannot consume the whole budget. Duplicate timestamps fall back to index bins.
    """
    groups = defaultdict(list)
    for frame in frames:
        groups[frame['source_id']].append(frame)
    sources = sorted(groups, key=lambda key: (_random(key), key))
    for source in sources:
        groups[source].sort(key=lambda row: (_time(row), row['id']))
    quotas = dict.fromkeys(sources, 0)
    remaining = min(budget, len(frames))
    while remaining:
        for source in sources:
            if quotas[source] < len(groups[source]):
                quotas[source] += 1
                remaining -= 1
                if not remaining:
                    break
    selected = {}
    for source in sources:
        rows, count = groups[source], quotas[source]
        if not count:
            continue
        stamps = [_time(row) for row in rows]
        used, indexes = set(), []
        for index in range(count):
            if stamps[-1] == stamps[0]:
                nearest = min(len(rows) - 1, int((index + .5) * len(rows) / count))
            else:
                target = stamps[0] + (index + .5) * (stamps[-1] - stamps[0]) / count
                insertion = bisect_left(stamps, target)
                nearest = min((j for j in (insertion - 1, insertion) if 0 <= j < len(rows)),
                              key=lambda j: (abs(stamps[j] - target), j))
            # Several time bins may hit one dense/duplicate timestamp. Fill from
            # the nearest remaining index without ever repeating an image.
            distance = 0
            while nearest in used:
                distance += 1
                options = [j for j in (nearest - distance, nearest + distance)
                           if 0 <= j < len(rows) and j not in used]
                if options:
                    nearest = options[0]
            used.add(nearest)
            indexes.append(nearest)
        selected[source] = [rows[j] for j in sorted(indexes)]
    return [selected[source][index] for index in range(max(quotas.values(), default=0))
            for source in sources if source in selected and index < len(selected[source])]


def _image(frame):
    from PIL import Image
    path = runtime._file(frame['path'])
    if path.stat().st_size > 64 * 1024 * 1024:
        raise ValueError('画像ファイルは64 MiB以下です')
    with Image.open(path) as image:
        if image.size != (frame['width'], frame['height']):
            raise ValueError('画像ファイルとサイズmetadataが一致しません')
        return image.convert('RGB')


def _unit(vector):
    values = [float(v) for v in vector]
    if not values or len(values) > 4096 or not all(math.isfinite(v) for v in values):
        raise ValueError('画像特徴が不正です')
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-12:
        raise ValueError('画像特徴のnormが0です')
    return [v / norm for v in values]


def _dot(left, right):
    if len(left) != len(right):
        raise ValueError('画像特徴の次元が一致しません')
    return max(-1., min(1., sum(a * b for a, b in zip(left, right))))


def _appearance(image):
    """Small RGB layout + colour histogram, for redundancy only, never labels."""
    from PIL import Image
    small = image.resize((16, 12), Image.Resampling.BILINEAR)
    pixels = getattr(small, 'get_flattened_data', small.getdata)()
    spatial = [channel / 255 for pixel in pixels for channel in pixel]
    histogram = small.histogram()
    colour = [sum(histogram[channel * 256 + i:channel * 256 + i + 16]) / 192
              for channel in range(3) for i in range(0, 256, 16)]
    # Separate normalisation avoids the RGB component overwhelming the histogram.
    return _unit(_unit([.01 + v for v in spatial]) + _unit(colour))


def review_options(plan):
    """Advisory review thresholds, independent of label/negative decisions."""
    return dict(
        review_confidence=runtime._number(plan.get('review_confidence', .10), .05, 1., '重点確認のscore下限'),
        expected_max_detections=runtime._number(plan.get('expected_max_detections', 3), 0, runtime.MAX_BOXES,
                                                '想定する検出数の上限', True),
        large_bbox_area_ratio=runtime._number(plan.get('large_bbox_area_ratio', .40), .01, 1.,
                                              '大きいbboxの面積比'))


def _yolo_rows(frames, weights, output, device, options=None):
    options = review_options(options or {})
    model = runtime._load_yolo(weights, output)
    names = runtime._names(model)
    if len(names) not in (1, 80) or getattr(model, 'task', 'detect') != 'detect':
        raise ValueError('選定には公式COCO80または学習済みRCカー1クラスの検出重みが必要です')
    rows = []
    for index, frame in enumerate(frames):
        with _image(frame) as image:
            feature = _appearance(image)
        prediction = model.predict(source=frame['path'], conf=.05, device=device, imgsz=640,
                                   classes=None, max_det=runtime.MAX_BOXES, verbose=False, save=False)[0]
        if tuple(prediction.orig_shape) != (frame['height'], frame['width']):
            raise ValueError('推論結果と元画像のサイズが一致しません')
        confidences, review_areas = [], []
        if prediction.boxes is not None:
            boxes = prediction.boxes.xyxy.cpu().tolist()
            scores = prediction.boxes.conf.cpu().tolist()
            if len(boxes) != len(scores):
                raise ValueError('検出bboxとscoreの件数が一致しません')
            for box, confidence in list(zip(boxes, scores))[:runtime.MAX_BOXES]:
                if (len(box) != 4 or not all(math.isfinite(v) for v in box)
                        or not math.isfinite(confidence) or not 0 < confidence <= 1):
                    continue
                x1, y1, x2, y2 = box
                x1, x2 = max(0, x1), min(frame['width'], x2)
                y1, y2 = max(0, y1), min(frame['height'], y2)
                if x1 < x2 and y1 < y2:
                    confidences.append(float(confidence))
                    if confidence >= options['review_confidence']:
                        review_areas.append((x2 - x1) * (y2 - y1) / (frame['width'] * frame['height']))
        top = max(confidences, default=0.)
        uncertainty = max((4 * p * (1 - p) for p in confidences), default=0.)
        score = .5 * top + .5 * uncertainty
        review_count, max_area = len(review_areas), max(review_areas, default=0.)
        flags, evidence = [], []
        if review_count > options['expected_max_detections']:
            flags.append('many_detections')
            evidence.append(f"検出数{review_count}件 > 想定上限{options['expected_max_detections']}件")
        if max_area >= options['large_bbox_area_ratio']:
            flags.append('large_bbox')
            evidence.append(f"最大bbox {max_area:.1%} ≥ 面積の目安{options['large_bbox_area_ratio']:.1%}")
        reason = (f"score ≥ {options['review_confidence']:.2f}の全クラス検出{review_count}件・"
                  f'最大bbox {max_area:.1%}・最大score {top:.2f}')
        if evidence:
            reason += '。要確認: ' + ' / '.join(evidence) + '（誤検出の断定ではありません）'
        elif not review_count:
            reason += f'（弱い候補を含む全候補{len(confidences)}件）。見落とし・背景の確認候補'
        rows.append(dict(frame=frame, score=score, feature=feature, background=not review_count,
                         reason=reason, detections=len(confidences), review_count=review_count,
                         max_bbox_area_ratio=max_area, review_flags=flags, hard_review=bool(flags)))
        print(f'画像選定 {index + 1}/{len(frames)}: {frame["id"]}', flush=True)
    return rows, dict(class_names=names, class_filter=None, prediction_confidence=.05, **options,
                      hard_review_candidates=sum(row['hard_review'] for row in rows),
                      review_definition='count > expected_max_detections OR max_clipped_bbox_area/image_area >= large_bbox_area_ratio; score >= review_confidence',
                      score_definition='.5*max_conf + .5*max(4*p*(1-p)); box count has no additive bonus',
                      note='検出クラスを問わない優先度。RCカーの確率・正解ではありません。')


def _regions(image):
    """Full image plus 2x2 and 3x3 local views: small objects retain more pixels."""
    width, height = image.size
    boxes = [(0, 0, width, height)]
    for divisions in (2, 3):
        boxes.extend((x * width // divisions, y * height // divisions,
                      (x + 1) * width // divisions, (y + 1) * height // divisions)
                     for y in range(divisions) for x in range(divisions)
                     if (x + 1) * width // divisions > x * width // divisions
                     and (y + 1) * height // divisions > y * height // divisions)
    return [image.crop(box) for box in boxes]


def _load_encoder(repo, weights, device):
    """Reuse the clean pinned checkout + strict raw ViT-S/16 weight contract.

    No torch.hub, pretrained=True, downloads, fine tuning, or fallback weights.
    E2E preprocessing is applied separately to every local image/crop.
    """
    root = Path(repo).resolve(strict=True)
    if not root.is_dir():
        raise ValueError('ローカルのDINOv3ソースディレクトリが必要です')
    package = runtime.REPO / 'ros2_ws/src/kart_e2e'
    if str(package) not in sys.path:
        sys.path.insert(0, str(package))
    from kart_e2e.contract import DINOV3_COMMIT
    from kart_e2e.model import load_rgb_encoder, preprocess
    import torch
    torch_device = 'cuda:' + device if device.isdigit() else device
    model = load_rgb_encoder(weights, root, 'dinov3_vits16').to(torch_device).eval()
    model.requires_grad_(False)

    def encode(images):
        features = []
        with torch.inference_mode():
            for start in range(0, len(images), 8):
                batch = torch.stack([preprocess(image) for image in images[start:start + 8]]).to(torch_device)
                result = model.forward_features(batch)
                # Combine the local CLS with local patch evidence; global CLS
                # alone can be dominated by a large road/background region.
                cls = torch.nn.functional.normalize(result['x_norm_clstoken'].float(), dim=-1)
                patches = torch.nn.functional.normalize(result['x_norm_patchtokens'].float().mean(1), dim=-1)
                features.extend(torch.nn.functional.normalize(cls + patches, dim=-1).cpu().tolist())
        return features
    return encode, dict(architecture='dinov3_vits16', encoder_commit=DINOV3_COMMIT,
                        preprocessing='kart_e2e.model.preprocess per local crop', feature='normalised CLS + mean patch tokens')


def _encode(encoder, images):
    try:
        features = encoder(images)
        if len(features) != len(images):
            raise ValueError('DINOv3の画像数と特徴数が一致しません')
        return [_unit(row) for row in features]
    finally:
        for image in images:
            image.close()


def _dinov3_rows(frames, all_frames, weights, repo, device):
    positive_frames = balanced_frames([row for row in all_frames if row['split'] == 'train'
                                       and row['reviewed'] and row['boxes']], MAX_SEED_POSITIVES)
    negative_frames = balanced_frames([row for row in all_frames if row['split'] == 'train'
                                       and row['reviewed'] and not row['boxes']], MAX_SEED_NEGATIVES)
    if not positive_frames or not negative_frames:
        raise ValueError('DINOv3選定にはtrainの確認済み正例・負例がそれぞれ1画像以上必要です')
    encoder, metadata = _load_encoder(repo, weights, device)
    positive, negative, positive_boxes = [], [], []
    # Spread the crop quota over images before taking second/third boxes.
    for index in range(runtime.MAX_BOXES):
        for frame in positive_frames:
            if index < len(frame['boxes']) and len(positive_boxes) < MAX_SEED_POSITIVES:
                positive_boxes.append((frame, frame['boxes'][index]))
        if len(positive_boxes) >= MAX_SEED_POSITIVES:
            break
    for frame, box in positive_boxes:
        with _image(frame) as image:
            x1, y1, x2, y2 = box
            pad_x, pad_y = .1 * (x2 - x1), .1 * (y2 - y1)
            crop = image.crop((max(0, math.floor(x1 - pad_x)), max(0, math.floor(y1 - pad_y)),
                               min(image.width, math.ceil(x2 + pad_x)), min(image.height, math.ceil(y2 + pad_y))))
            positive.extend(_encode(encoder, [crop]))
    negative_views = defaultdict(list)
    for index in range(14):
        for frame in negative_frames:
            if sum(map(len, negative_views.values())) >= MAX_SEED_NEGATIVE_VIEWS:
                break
            # Keep the whole-image view, then spread local windows across seeds.
            view = 0 if index == 0 else 1 + (index - 1 + int(_random(frame['id']) * 13)) % 13
            negative_views[frame['id']].append(view)
    for frame in negative_frames:
        with _image(frame) as image:
            regions = _regions(image)
            selected = set(negative_views[frame['id']])
            crops = []
            for index, crop in enumerate(regions):
                if index in selected:
                    crops.append(crop)
                else:
                    crop.close()
            negative.extend(_encode(encoder, crops))
    rows = []
    for index, frame in enumerate(frames):
        with _image(frame) as image:
            appearance = _appearance(image)
            features = _encode(encoder, _regions(image))
        similarities = []
        for feature in features:
            # Nearest positive seed permits multiple RC-car appearances. The
            # nearest confirmed background discourages road/grass similarity.
            pos = max(_dot(feature, seed) for seed in positive)
            neg = max(_dot(feature, seed) for seed in negative)
            similarities.append((pos - .5 * max(0., neg), pos, neg, feature))
        score, pos, neg, best_feature = max(similarities, key=lambda item: item[0])
        # DINO evidence handles semantic redundancy; appearance handles duplicate
        # captures with small feature changes. Both vectors are unit length.
        feature = _unit(best_feature + appearance)
        rows.append(dict(frame=frame, score=score, feature=feature, background=False,
                         reason=f'局所特徴の正例類似度 {pos:.3f}・負例類似度 {neg:.3f}（確率ではありません）'))
        print(f'DINOv3画像選定 {index + 1}/{len(frames)}: {frame["id"]}', flush=True)
    return rows, dict(metadata, positive_seed_frames=[row['id'] for row in positive_frames],
                      negative_seed_frames=[row['id'] for row in negative_frames],
                      positive_seed_crops=len(positive), negative_seed_crops=len(negative), seed_split='train',
                      views='full + 2x2 + 3x3 fixed local crops (14 views)',
                      score_definition='max_local_view(max_positive_cosine - .5*max(0,max_negative_cosine))',
                      note='凍結特徴の類似度による確認優先度。RCカーの確率・正解ではありません。')


def diverse_order(rows, limit):
    """Deterministic MMR-like order with background and exploration slots.

    YOLO hard-review candidates use slots 1/3 of each ten, when available.
    Every fifth slot explores YOLO misses; every tenth slot explores all images.
    A thresholded cosine penalty suppresses nearly identical images, plus a
    temporal penalty for adjacent captures from the same source bag.
    """
    remaining = [dict(row, _redundancy=0., _near=False) for row in rows]
    chosen, output = [], []
    bag_counts = defaultdict(int)
    low = min((row['score'] for row in rows), default=0.)
    high = max((row['score'] for row in rows), default=0.)
    hard_review = any(row.get('hard_review') for row in rows)
    for rank in range(1, min(limit, len(rows)) + 1):
        group = 'exploration' if rank % 10 == 0 else 'background' if rank % 5 == 0 else 'priority'
        if hard_review and rank % 10 in (1, 3):
            group = 'hard_review'
        if group == 'background':
            pool = [row for row in remaining if row['background']]
        elif group == 'hard_review':
            pool = [row for row in remaining if row.get('hard_review')]
        elif group == 'priority' and hard_review:
            pool = [row for row in remaining if not row.get('hard_review')]
        else:
            pool = remaining
        if not pool:
            pool = [row for row in remaining if not row.get('hard_review')] if hard_review else remaining
            pool, group = pool or remaining, 'priority'

        def utility(row):
            priority = ((row['score'] - low) / (high - low)) if high > low else .5
            if group == 'exploration':
                priority = _random(row['frame']['id'])
            duplicate_penalty = max(0., (row['_redundancy'] - .8) / .2)
            bag_count = bag_counts[row['frame']['source_id']]
            return priority - .9 * duplicate_penalty - .2 * row['_near'] - .1 * bag_count / max(1, len(chosen))

        row = max(pool, key=lambda item: (utility(item), _random(item['frame']['id']), item['frame']['id']))
        remaining.remove(row)
        chosen.append(row)
        bag_counts[row['frame']['source_id']] += 1
        for other in remaining:
            other['_redundancy'] = max(other['_redundancy'], _dot(other['feature'], row['feature']))
            if (row['frame']['source_id'] == other['frame']['source_id']
                    and abs(_time(row['frame']) - _time(other['frame'])) < 1_000_000_000):
                other['_near'] = True
        prefix = ('誤検出の重点確認候補。' if group == 'hard_review' else '背景探索枠。' if group == 'background'
                  else '再現可能な探索枠。' if group == 'exploration' else '')
        result = dict(frame_id=row['frame']['id'], rank=rank, score=row['score'],
                      reason=prefix + row['reason'], group=group)
        for field in ('detections', 'review_count', 'max_bbox_area_ratio', 'review_flags', 'hard_review'):
            if field in row:
                result[field] = row[field]
        output.append(result)
    return output


def run(plan):
    if not isinstance(plan, dict) or plan.get('action') != 'select':
        raise ValueError('action=selectを指定してください')
    method = plan.get('method', 'yolo')
    if method not in ('yolo', 'dinov3'):
        raise ValueError('画像選定方式はyolo / dinov3です')
    maximum = runtime._number(plan.get('max_images', 500), 1, MAX_IMAGES, '選定で推論する最大画像数', True)
    limit = runtime._number(plan.get('limit', 100), 1, MAX_IMAGES, '選定枚数', True)
    options = review_options(plan) if method == 'yolo' else {}
    frames = _frames(plan.get('frames'))
    candidates = [row for row in frames if row['split'] == 'train' and not row['reviewed']]
    sampled = balanced_frames(candidates, maximum)
    output = runtime._output(plan['output'], ('selection.json',))
    metadata = dict(candidate_split='train', candidates=len(candidates), evaluated=len(sampled),
                    project_frames=len(frames), max_images=maximum, limit=limit, seed=SEED,
                    sampling='round-robin source quotas + uniform bag-record-time bins (header time fallback)',
                    sampled_frame_ids=[row['id'] for row in sampled], annotations_modified=False,
                    automatic_ground_truth=False, score_is_probability=False,
                    diversity='cosine redundancy + same-source temporal spacing; fifth/tenth slots explore',
                    versions=runtime._versions())
    if method == 'yolo':
        metadata.update(options, hard_review_candidates=0,
                        diversity='cosine redundancy + same-source temporal spacing; hard-review slots 1/3, background 5, exploration 10 per ten; ordinary slots prefer non-hard candidates')
    if sampled:
        device = runtime._device(plan.get('device', 'auto'))
        weights = runtime._file(plan['weights'] if method == 'yolo' else plan['encoder_weights'])
        checksum = runtime._sha(weights)
        if method == 'yolo':
            if weights.suffix.lower() != '.pt':
                raise ValueError('YOLO重みはローカルの.ptファイルです')
            rows, details = _yolo_rows(sampled, weights, output, device, options)
            metadata.update(source_weights=str(weights), source_weights_sha256=checksum)
        else:
            repo = plan.get('encoder_repo')
            if not isinstance(repo, str) or not Path(repo).is_absolute():
                raise ValueError('DINOv3のローカルソースは絶対パスで指定してください')
            rows, details = _dinov3_rows(sampled, frames, weights, repo, device)
            metadata.update(encoder_repo=str(Path(repo).resolve()), encoder_weights=str(weights),
                            encoder_weights_sha256=checksum)
        if runtime._sha(weights) != checksum:
            raise ValueError('画像選定中に重みファイルが変更されました')
        metadata.update(details, device=device)
        ranked = diverse_order(rows, limit)
    else:
        ranked = []
    result = dict(schema='kart.perception.selection.v1', method=method, ranked=ranked, provenance=metadata)
    atomic_json(output / 'selection.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True)
    args = parser.parse_args()
    result = run(json.loads(Path(args.plan).read_text()))
    print(json.dumps(dict(method=result['method'], selected=len(result['ranked'])), ensure_ascii=False))


if __name__ == '__main__':
    main()
