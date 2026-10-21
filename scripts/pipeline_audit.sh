#!/usr/bin/env bash
# Offline metadata analysis only: no ROS process, playback, SSH, or vehicle output.
set -euo pipefail
KART_AUDIT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${KART_AUDIT_PYTHON:-python3}" "$KART_AUDIT_ROOT/tools/pipeline_audit/cli.py" "$@"
