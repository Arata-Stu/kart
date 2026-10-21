#!/usr/bin/env python3
"""Read-only rosbag2 timing report. Does not replay or publish any ROS messages."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.pipeline_audit.analysis import analyze, validate
from tools.pipeline_audit.bag import read_bag
from tools.pipeline_audit.report import write_report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--bag', type=Path, help='rosbag2 directory (MCAP/SQLite3), or a standalone MCAP file')
    source.add_argument('--records-json', type=Path, help='Extracted small metadata/test fixture; not image data')
    parser.add_argument('--output', type=Path, required=True, help='New report directory; existing directories are refused')
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('config.json'))
    parser.add_argument('--source-topic', help='RGB image topic or /event_camera/tensor_timing for EVS')
    args = parser.parse_args(argv)
    try:
        config = copy.deepcopy(json.loads(args.config.read_text()))
        if args.source_topic:
            config['topics']['source'] = args.source_topic
        validate(config)
        output = args.output.expanduser().resolve()
        if output.exists():
            raise ValueError(f'Output already exists: {output}')
        # Never create analysis artifacts inside the source recording.
        if args.bag:
            bag = args.bag.expanduser().resolve()
            if not bag.exists():
                raise ValueError(f'Bag not found: {bag}')
            if bag.is_dir() and output.is_relative_to(bag):
                raise ValueError('Output must be outside the input recording')
            data = read_bag(bag, config)
            files = sorted(bag.rglob('*')) if bag.is_dir() else [bag]
            manifest = [dict(path=str(f.relative_to(bag)) if bag.is_dir() else f.name,
                             bytes=f.stat().st_size, mtime_ns=f.stat().st_mtime_ns) for f in files if f.is_file()]
        else:
            data = json.loads(args.records_json.read_text())
            manifest = []
        summary, matches, events, series = analyze(data, config)
        summary['provenance'] = dict(config_sha256=hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
                                     input_manifest=manifest, bag_content_hashed=False)
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(tempfile.mkdtemp(prefix='.' + output.name + '-', dir=output.parent))
        try:
            write_report(temporary, summary, matches, events, series)
            with (temporary / 'metadata.jsonl').open('w', encoding='utf-8') as stream:
                for record in data['records']:
                    stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n')
            if output.exists():
                raise ValueError(f'Output already exists: {output}')
            temporary.rename(output)
        except BaseException:
            shutil.rmtree(temporary, ignore_errors=True)
            raise
        print(f'Report: {output / "report.html"}')
        print('Qualified source -> final first-observation ms:', json.dumps(summary['qualified_latency_ms']['final']))
        if summary['qualified_latency_ms']['final']['count'] == 0:
            print('No qualified end-to-end samples; inspect missing_topics / coverage. This is not a zero-latency result.')
        return 0
    except (ValueError, OSError, ImportError, KeyError, TypeError) as error:
        print(f'pipeline_audit: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
