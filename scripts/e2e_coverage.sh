#!/usr/bin/env bash
set -euo pipefail
KART_COVERAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${KART_COVERAGE_PYTHON:-python3}" "$KART_COVERAGE_ROOT/tools/e2e_coverage/cli.py" "$@"
