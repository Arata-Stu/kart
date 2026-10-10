#!/usr/bin/env bash
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "${KART_VGL_PYTHON:-/opt/inference/bin/python}" "$kart_root/tools/vgl/lab.py" "$@"
