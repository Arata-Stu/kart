"""Export the pinned NVIDIA core and apply the auditable fixed-reference patch."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile

REVISION = '33da60662b27c0c5b63e0597bd778066af2731ed'


def prepare(source, destination):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    patch = Path(__file__).with_name('fixed_reference.patch')
    digest = hashlib.sha256(patch.read_bytes() + Path(__file__).read_bytes()).hexdigest()
    marker = destination / '.kart-source.json'
    expected = {'revision': REVISION, 'patch_and_exporter_sha256': digest}
    if destination.exists():
        if marker.is_file() and json.loads(marker.read_text()) == expected:
            return
        raise RuntimeError(f'Export differs or is incomplete: {destination}. Use a fresh build directory.')
    commit = subprocess.check_output(
        ['git', '-C', str(source), 'rev-parse', REVISION + '^{commit}'], text=True).strip()
    if commit != REVISION:
        raise RuntimeError('Unexpected cuVSLAM revision')
    archive = subprocess.check_output(['git', '-C', str(source), 'archive', REVISION])
    destination.mkdir(parents=True)
    with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
        for member in tar.getmembers():
            if not (member.isfile() or member.isdir()) or not (destination / member.name).resolve().is_relative_to(destination):
                raise ValueError('Unsafe archive entry')
        if hasattr(tarfile, 'data_filter'):
            tar.extractall(destination, filter='data')
        else:
            tar.extractall(destination)
    # Prevent discovery of the workspace's parent Git repository.
    env = dict(os.environ, GIT_CEILING_DIRECTORIES=str(destination.parent))
    subprocess.run(['git', 'apply', '--check', str(patch.resolve())], cwd=destination, env=env, check=True)
    subprocess.run(['git', 'apply', str(patch.resolve())], cwd=destination, env=env, check=True)
    with (destination / 'libs/cuvslam/CMakeLists.txt').open('a') as f:
        f.write('\n# Isolate the fixed-map core from the apt libcuvslam.so.\n'
                'set_target_properties(cuvslam PROPERTIES OUTPUT_NAME kart_fixed_cuvslam)\n')
    marker.write_text(json.dumps(expected, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('destination')
    args = parser.parse_args()
    prepare(args.source, args.destination)
