"""Compare all vendored runtime C++ files with the pinned NVIDIA Git commit."""
import argparse
from pathlib import Path
import subprocess

REVISION = 'e4dc8f59816dfca387bcb92364ecaf9207216475'


def verify(checkout):
    package = Path(__file__).resolve().parents[1]
    count = 0
    for directory in ('src', 'include', 'test', 'launch', 'rviz'):
        prefix = 'isaac_ros_cuvslam/' + directory + '/'
        names = subprocess.check_output(
            ['git', '-C', checkout, 'ls-tree', '-r', '--name-only', REVISION, prefix], text=True).splitlines()
        expected = {name.removeprefix('isaac_ros_cuvslam/') for name in names}
        actual = {str(p.relative_to(package)) for p in (package / directory).rglob('*')
                  if p.is_file() and '__pycache__' not in p.parts}
        if expected != actual:
            raise RuntimeError(f'File set changed in {directory}: {expected ^ actual}')
        for name in names:
            baseline = subprocess.check_output(['git', '-C', checkout, 'show', REVISION + ':' + name])
            target = package / name.removeprefix('isaac_ros_cuvslam/')
            if baseline != target.read_bytes():
                raise RuntimeError(f'Upstream file changed: {target}')
            count += 1
    print(f'{count} upstream files byte-identical (src/include/test/launch/rviz) at {REVISION}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('checkout')
    verify(parser.parse_args().checkout)
