#!/usr/bin/env bash
# Run on the notebook host with a desktop session, not inside the ROS container.
# Official link format: https://docs.foxglove.dev/docs/visualization/shareable-links
set -euo pipefail

usage() {
  cat <<'HELP'
Usage: scripts/open-foxglove.sh [--web] [--print-url] [--preset ID | --list] [HOST[:PORT] | ws://URL | wss://URL]

Open Foxglove with the Bridge connection preselected.
  Default: desktop app, config/jetson_hosts.json default (10.42.0.1)
  KART_FOXGLOVE_URL: default connection address (command argument takes precedence)
  --preset ID  Choose a shared Jetson IP preset (notebook, lan, usb)
  --list       List shared presets without opening anything
  --web        Open the web app instead of the desktop app
  --print-url  Print the link without opening anything
  -h, --help   Show this help

Examples:
  scripts/open-foxglove.sh jetson.local
  scripts/open-foxglove.sh 192.168.1.20:8765
  scripts/open-foxglove.sh --web ws://localhost:8765

Start the Bridge separately on Jetson:
  ros2 launch kart_bringup foxglove.launch.py
HELP
}

kart_open_mode=desktop
kart_print_url=false
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
kart_target=${KART_FOXGLOVE_URL:-}
kart_preset=""
kart_list=false
kart_target_set=false
while (( $# )); do
  case "$1" in
    --web) kart_open_mode=web ;;
    --list) kart_list=true ;;
    --preset)
      if [[ $# -lt 2 || "$kart_target_set" == true ]]; then usage >&2; exit 2; fi
      kart_preset=$2
      kart_target_set=true
      kart_target=""
      shift
      ;;
    --print-url) kart_print_url=true ;;
    -h|--help) usage; exit 0 ;;
    -*) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
    *)
      if [[ "$kart_target_set" == true ]]; then usage >&2; exit 2; fi
      kart_target=$1
      kart_target_set=true
      ;;
  esac
  shift
done

kart_link=$(python3 - "$kart_target" "$kart_open_mode" "$kart_root/config/jetson_hosts.json" "$kart_preset" "$kart_list" <<'PY'
import sys
import json
from pathlib import Path
from urllib.parse import urlencode, urlsplit, urlunsplit

value, mode, config, preset, listing = sys.argv[1:]
hosts = json.loads(Path(config).read_text())
if listing == 'true':
    for entry in hosts['presets']:
        print(f"{entry['id']:10} {entry['host']}" + (' (default)' if entry['id'] == hosts['default'] else ''))
    sys.exit(0)
if not value:
    key = preset or hosts['default']
    match = next((p for p in hosts['presets'] if p['id'] == key), None)
    if match is None:
        sys.exit(f'Unknown preset: {key}; use --list')
    value = match['host']
try:
    if any(c.isspace() or ord(c) < 32 for c in value):
        raise ValueError('Whitespace is not allowed')
    url = urlsplit(value if '://' in value else 'ws://' + value)
    if url.scheme not in ('ws', 'wss') or not url.hostname or url.username or url.password or url.fragment:
        raise ValueError('Use a ws:// or wss:// Bridge URL without credentials or fragments')
    port = url.port  # Validate numeric port and range
    if port == 0:
        raise ValueError('Port must be 1..65535')
    host = '[' + url.hostname + ']' if ':' in url.hostname else url.hostname
    endpoint = urlunsplit((url.scheme, f'{host}:{port if port is not None else 8765}', url.path, url.query, ''))
except ValueError as exc:
    sys.exit(f'Invalid Bridge address: {exc}')
print('https://app.foxglove.dev/~/view?' + urlencode({
    'ds': 'foxglove-websocket', 'ds.url': endpoint, 'openIn': mode,
}))
PY
)

if [[ "$kart_print_url" == true || "$kart_list" == true ]]; then
  printf '%s\n' "$kart_link"
  exit 0
fi
case "$(uname -s)" in
  Darwin) exec open "$kart_link" ;;
  Linux)
    if command -v xdg-open >/dev/null 2>&1; then exec xdg-open "$kart_link"; fi
    printf 'xdg-open is required on the notebook host. Use --print-url to copy the link.\n' >&2
    exit 1
    ;;
  *) printf 'Unsupported OS. Use --print-url and open the link manually.\n' >&2; exit 1 ;;
esac
