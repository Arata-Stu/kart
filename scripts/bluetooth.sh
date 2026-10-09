#!/usr/bin/env bash
# DualSense setup on the Linux host; Bluetooth is managed outside Docker.
set -euo pipefail
export LC_ALL=C

usage() {
  cat <<'HELP'
Usage: scripts/bluetooth.sh [--scan | --pair MAC | --connect MAC | --status MAC]

Linuxホスト側で実行するDualSense接続用ツール。
  引数なし       bluetoothctlを対話モードで開く
  --scan         15秒間デバイスを検索する
  --pair MAC     ペアリング・trust・接続（各操作最大30秒）
  --connect MAC  登録済みデバイスへ再接続
  --status MAC   デバイス情報を表示
  -h, --help     このヘルプを表示

初回はDualSenseのCreate＋PSを長押しし、ライトを点滅させる。
--scanでMACを確認し、--pair AA:BB:CC:DD:EE:FFを実行する。
HELP
}

kart_mode=${1:-interactive}
case "$kart_mode" in
  -h|--help) usage; exit 0 ;;
  interactive|--scan) [[ $# -le 1 ]] || { usage >&2; exit 2; } ;;
  --pair|--connect|--status)
    if [[ $# -ne 2 || ! "$2" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]]; then
      usage >&2; exit 2
    fi
    ;;
  *) usage >&2; exit 2 ;;
esac

if [[ "$(uname -s)" != Linux || -f /.dockerenv || -f /run/.containerenv ]]; then
  echo 'JetsonまたはUbuntu PCのLinuxホスト側で実行してください。' >&2
  exit 1
fi
if ! command -v bluetoothctl >/dev/null 2>&1; then
  echo 'bluetoothctlがありません。ホスト側でsudo apt install bluezを実行してください。' >&2
  exit 1
fi

case "$kart_mode" in
  interactive)
    cat <<'HELP'
DualSense: Create＋PS長押しでペアリングモード。
bluetoothctl内で次を順に実行してください:
  power on
  agent NoInputNoOutput
  default-agent
  scan on
  pair <DualSenseのMAC>
  trust <DualSenseのMAC>
  connect <DualSenseのMAC>
  scan off
  quit
HELP
    exec bluetoothctl
    ;;
  --scan)
    bluetoothctl --timeout 10 power on
    exec bluetoothctl --timeout 15 scan on
    ;;
  --status) exec bluetoothctl --timeout 10 info "$2" ;;
  --pair|--connect)
    bluetoothctl --timeout 10 power on
    if [[ "$kart_mode" == --pair ]]; then
      bluetoothctl --agent NoInputNoOutput --timeout 30 pair "$2"
      bluetoothctl --timeout 10 trust "$2"
    fi
    bluetoothctl --timeout 30 connect "$2"
    kart_info=$(bluetoothctl --timeout 10 info "$2")
    printf '%s\n' "$kart_info"
    if [[ "$kart_info" != *'Connected: yes'* ]]; then
      echo '接続を確認できません。PSボタン・ホストのBluetoothサービス・rfkill状態を確認してください。' >&2
      exit 1
    fi
    ;;
esac
