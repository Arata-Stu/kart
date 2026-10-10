#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    echo 'Usage: ./scripts/screen.sh [SESSION_NAME] (default: kart)'
    echo 'LinuxホストでSSH用screenを作成／再接続。screen内ではデタッチします。'
    exit 0
fi
if (($# > 1)); then
    echo 'セッション名は1つだけ指定してください。' >&2
    exit 1
fi
if [[ -f /.dockerenv ]]; then
    echo 'screen.shはLinuxホストで実行してください。Docker内ではtmuxを使います。' >&2
    exit 1
fi
command -v screen >/dev/null 2>&1 || {
    echo 'screenが必要です。ホストで sudo apt-get install screen を実行してください。' >&2
    exit 1
}
if [[ -n "${STY:-}" ]]; then
    exec screen -d "$STY"
fi
KART_SCREEN_SESSION="${1:-kart}"
if [[ ! "$KART_SCREEN_SESSION" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo 'セッション名には英数字、_、-を使用してください。' >&2
    exit 1
fi
exec screen -D -RR -S "$KART_SCREEN_SESSION"
