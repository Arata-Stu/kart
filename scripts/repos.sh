#!/usr/bin/env bash
set -euo pipefail
KART_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$KART_ROOT"
if ! command -v vcs >/dev/null 2>&1; then
    echo 'Install vcstool first (Ubuntu: sudo apt install python3-vcstool).' >&2
    exit 1
fi
case "${1:-status}" in
    import) exec vcs import --input packages.repos . ;;
    status) exec vcs status tools ros2_ws/src/sensing ;;
    *) echo 'Usage: scripts/repos.sh [import|status]' >&2; exit 2 ;;
esac
