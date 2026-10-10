#!/usr/bin/env bash
set -euo pipefail

readonly CUI_TARGET="multi-user.target"
readonly GUI_TARGET="graphical.target"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/jetson_display_mode.sh status
  ./scripts/jetson_display_mode.sh cui [--now]
  ./scripts/jetson_display_mode.sh gui [--now]

Commands:
  status  現在のデフォルト起動モードを表示します。
  cui     次回起動から CUI のみにします。
  gui     次回起動から GUI を有効にします。

Options:
  --now   デフォルトを変更した直後、そのモードへ即時移行します。
          cui --now は実行中の GUI セッションを終了します。
  -h, --help
          このヘルプを表示します。

Examples:
  ./scripts/jetson_display_mode.sh cui
  ./scripts/jetson_display_mode.sh gui
  ./scripts/jetson_display_mode.sh status
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

require_systemd() {
  command -v systemctl >/dev/null 2>&1 || die "systemctl が見つかりません。"
}

run_as_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  else
    command -v sudo >/dev/null 2>&1 || die "sudo が見つかりません。root で実行してください。"
    sudo "$@"
  fi
}

show_status() {
  local target
  target="$(systemctl get-default)"

  case "${target}" in
    "${CUI_TARGET}")
      echo "起動モード: CUI (${target})"
      ;;
    "${GUI_TARGET}")
      echo "起動モード: GUI (${target})"
      ;;
    *)
      echo "起動モード: その他 (${target})"
      ;;
  esac
}

[[ $# -ge 1 ]] || {
  usage
  exit 1
}

mode="$1"
shift

apply_now=false
if [[ $# -gt 0 ]]; then
  [[ $# -eq 1 && "$1" == "--now" ]] || die "不明なオプションです: $*"
  apply_now=true
fi

case "${mode}" in
  -h|--help)
    [[ $# -eq 0 ]] || die "--help に引数は指定できません。"
    usage
    exit 0
    ;;
esac

[[ "$(uname -s)" == Linux && "$(uname -m)" == aarch64 ]] ||
  die "Jetsonホスト（Linux aarch64）で実行してください。"
[[ ! -e /.dockerenv && ! -e /run/.containerenv ]] ||
  die "コンテナではなくJetsonホストで実行してください。"
require_systemd

case "${mode}" in
  status)
    [[ "${apply_now}" == false ]] || die "status に --now は指定できません。"
    show_status
    ;;
  cui)
    run_as_root systemctl set-default "${CUI_TARGET}"
    echo "次回起動時のモードを CUI に設定しました。"
    if [[ "${apply_now}" == true ]]; then
      echo "CUI モードへ移行します。GUI セッションは終了します。"
      run_as_root systemctl isolate "${CUI_TARGET}"
    else
      echo "反映するには Jetson を再起動してください。"
    fi
    ;;
  gui)
    run_as_root systemctl set-default "${GUI_TARGET}"
    echo "次回起動時のモードを GUI に設定しました。"
    if [[ "${apply_now}" == true ]]; then
      echo "GUI モードへ移行します。"
      run_as_root systemctl isolate "${GUI_TARGET}"
    else
      echo "反映するには Jetson を再起動してください。"
    fi
    ;;
  *)
    die "モードは status、cui、gui のいずれかを指定してください。"
    ;;
esac
