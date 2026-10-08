#!/usr/bin/env bash
# Update to the kart manifest pin, then build/install the host CLI.
set -euo pipefail
KART_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
    echo 'Usage: ./scripts/update-isaac-ros-cli.sh [--help]'
    echo 'packages.reposの指定commitへ更新し、依存導入・debビルド・ホストインストールを行います。'
    echo '先にkart本体を更新してください。CLIの未commit変更がある場合は停止します。'
}
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then usage; exit 0; fi
[[ $# == 0 ]] || { usage >&2; exit 2; }
[[ "$(uname -s)" == Linux && -f /etc/debian_version ]] || {
    echo 'Ubuntuホストで実行してください。' >&2; exit 1;
}
[[ ! -e /.dockerenv && ! -e /run/.containerenv ]] || {
    echo 'コンテナではなくホストで実行してください。' >&2; exit 1;
}
[[ "$EUID" != 0 ]] || { echo 'sudoでスクリプト全体を実行せず、一般ユーザーで実行してください。' >&2; exit 1; }
CLI_DIR="${KART_ROOT}/tools/isaac-ros-cli"
# Check before package installation; do not discard local work.
if [[ -d "$CLI_DIR" ]]; then
    [[ "$(git -C "$CLI_DIR" rev-parse --show-toplevel)" == "$CLI_DIR" ]] || {
        echo 'CLIは独立したGit checkoutである必要があります。' >&2; exit 1;
    }
    [[ -z "$(git -C "$CLI_DIR" status --porcelain)" ]] || {
        echo 'CLIに未commit変更があります。commitまたは退避してから再実行してください。' >&2; exit 1;
    }
fi
sudo apt-get update
sudo apt-get install -y git vcstool python3-yaml build-essential dpkg-dev debhelper dh-python make
readarray -t CLI_SPEC < <(python3 - "$KART_ROOT/packages.repos" <<'PY'
import sys, yaml
spec = yaml.safe_load(open(sys.argv[1]))['repositories']['tools/isaac-ros-cli']
assert spec['type'] == 'git'
print(spec['url'])
print(spec['version'])
PY
)
[[ ${#CLI_SPEC[@]} == 2 && -n "${CLI_SPEC[0]}" && -n "${CLI_SPEC[1]}" ]] || {
    echo 'packages.reposのCLI定義を読み取れません。' >&2; exit 1;
}
if [[ ! -d "$CLI_DIR" ]]; then
    git clone --no-checkout "${CLI_SPEC[0]}" "$CLI_DIR"
fi
git -C "$CLI_DIR" fetch "${CLI_SPEC[0]}" "${CLI_SPEC[1]}"
git -C "$CLI_DIR" checkout --detach FETCH_HEAD
printf 'CLI commit: %s\n' "$(git -C "$CLI_DIR" rev-parse HEAD)"
# Preserve previous debs so make install selects only the new build.
shopt -s nullglob
OLD_DEBS=("${KART_ROOT}"/tools/isaac-ros-cli_*.deb)
if ((${#OLD_DEBS[@]})); then
    DEB_BACKUP="$(mktemp -d "${TMPDIR:-/tmp}/kart-cli-debs.XXXXXX")"
    mv -- "${OLD_DEBS[@]}" "$DEB_BACKUP/"
    printf 'Previous debs: %s\n' "$DEB_BACKUP"
fi
cd "$CLI_DIR"
make build
sudo make install
isaac-ros status
echo 'CLI更新完了。初回のみ sudo isaac-ros init docker を実行してください。'
echo 'イメージ更新時は走行・録画を終了し、docker stop kart_dev 後に ./scripts/dev.sh --build-local を実行してください。'
