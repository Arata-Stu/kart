#!/usr/bin/env bash
# Short entrypoint for Kart Map Studio; forward all server arguments.
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec bash "$kart_root/tools/app/start.sh" "$@"
